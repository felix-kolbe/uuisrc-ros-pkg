#!/usr/bin/env python

import os
import sys
from threading import Thread

import math
from math import pi, radians, degrees

import gtk
import pygtk
pygtk.require("2.0")
import gobject

import csv
import xml.dom.minidom

import roslib; roslib.load_manifest('schunk_gui')
import rospy

from std_msgs.msg import Empty, Int8
from sensor_msgs.msg import JointState
from metralabs_msgs.msg import SchunkStatus

import tf


RANGE = 10000
VELOCITY_CMD_TOPIC="/schunk/move_all_velocity"
POSITION_CMD_TOPIC="/schunk/move_all_position"
JOINT_STATE_TOPIC="/joint_states"
SCHUNK_STATUS_TOPIC="/schunk/status"

class RosCommunication():
    def __init__(self):
        description = rospy.get_param("schunk_description", None)   # used by many schunk packages
        if description is None:
            description = rospy.get_param("robot_description", None)    # used by most packages
        assert description is not None, "Neither robot_description nor schunk_description given"
        
        robot = xml.dom.minidom.parseString(description).getElementsByTagName('robot')[0]
        self.joint_name_to_config_dict = {}
        self.joint_name_to_index_dict = {}
        self.joint_names_list = []    # joint names in real order
        self.currentJointStates = JointState()
        self.currentJointStates_jointIndex_to_msgIndex_dict = {}
        self.currentSchunkStatus = SchunkStatus()
        self.currentSchunkStatus_jointIndex_to_msgIndex_dict = {}
        self.dependent_joints = rospy.get_param("dependent_joints", {})
       
        if rospy.has_param("~tip_name"):
            print "YEEEE"
       
        try: 
            self.__tip = rospy.get_param("~tip_name")
        except KeyError:
            rospy.logwarn("No tip name specified, end effector position won't work")
            self.__tip = "none"
        
        try: 
            self.__root = rospy.get_param("~root_name")
        except KeyError:
            rospy.logwarn("No root name specified, end effector position won't work")
            self.__root = "none"
        
        # Find all non-fixed non-mimicking joints
        self.numModules = 0
        for child in robot.childNodes:
            if child.nodeType is child.TEXT_NODE:
                continue
            if child.localName == 'joint':
                jtype = child.getAttribute('type')
                if jtype == 'fixed':
                    continue
                    
                # encoding needed for most lookups like "self.roscomms.joint_names_list[module]"
                name = child.getAttribute('name').encode('ascii')
                if jtype == 'continuous':
                    minval = -pi
                    maxval = pi
                else:
                    limit = child.getElementsByTagName('limit')[0]
                    minval = float(limit.getAttribute('lower'))
                    maxval = float(limit.getAttribute('upper'))

                if name in self.dependent_joints or len(child.getElementsByTagName('mimic')) != 0:
                    continue

                if minval > 0 or maxval < 0:
                    zeroval = (maxval + minval)/2
                else:
                    zeroval = 0

                joint = {'min':minval, 'max':maxval, 'zero':zeroval, 'value':zeroval }
                self.joint_name_to_config_dict[name] = joint    # store joint
                self.joint_names_list.append(name)    # store joints name
                self.joint_name_to_index_dict[name] = self.numModules    # store index for joint
                
                rospy.loginfo("Registered joint with index '%d' and name '%s'.", self.numModules, name)
                
                self.numModules += 1

        # Setup all of the pubs and subs
        self.velocityPub = rospy.Publisher(VELOCITY_CMD_TOPIC, JointState)
        self.positionPub = rospy.Publisher(POSITION_CMD_TOPIC, JointState)
        self.jointSub = rospy.Subscriber(JOINT_STATE_TOPIC, JointState, self.jointStateUpdate)
        self.statusSub = rospy.Subscriber(SCHUNK_STATUS_TOPIC, SchunkStatus, self.schunkStatusUpdate)
        
        # Members that will be filled by the gui for commanding
        self.targetVelocity = JointState()
        self.targetPosition = JointState()
        self.setVelocity = False
        self.setPosition = False
        
        self.ackJoint = False
        self.ackNumber =0
        self.refJoint = False
        self.refNumber =0
        self.ackAll = False
        self.refAll = False
        self.maxCurrents = False
        self.emergencyStop = False

#        self.targetCurrent = JointState() # TODO: Set the current controls with the effort field (in SchunkRos also) 

        # A tf listener so that we can find the position of the end effector without service calls to
        # kinematics node
        self.tfListener = tf.TransformListener()

        
        
    def jointStateUpdate(self, data):
        """ Store new joint states data and calculate the index lookup dict as the message might not be sorted.
        
        In other words: When the joint has real index x, which index does it have in this message?
        So no states consuming method should sort or search in the msg name array anymore!
        """
        self.currentJointStates = data
        
        # get name_to_index dict for message
        self.currentJointStates_jointIndex_to_msgIndex_dict = {}
        for msg_i in range(len(self.currentJointStates.name)):
            msg_name = self.currentJointStates.name[msg_i]
            try:
                name_i = self.joint_name_to_index_dict[msg_name]
                self.currentJointStates_jointIndex_to_msgIndex_dict[name_i] = msg_i
            except KeyError:
                # message removed because this case is happening with mimicking joints
                # rospy.logwarn("JointStatus message contains a joint I don't know from the robot_description: %s.", msg_name)
                pass
    
    def schunkStatusUpdate(self, data):
        """ Store new schunk status data and calculate the index lookup dict as the message might not be sorted.
        
        In other words: When the joint has real index x, which index does it have in this message?
        So no status consuming method should sort or search in the msg name array anymore!
        """  
        self.currentSchunkStatus = data
        
        # get name_to_index dict for message
        self.currentSchunkStatus_jointIndex_to_msgIndex_dict = {}
        for msg_i in range(len(self.currentSchunkStatus.joints)):
            msg_name = self.currentSchunkStatus.joints[msg_i].jointName
            try:
                name_i = self.joint_name_to_index_dict[msg_name]
                self.currentSchunkStatus_jointIndex_to_msgIndex_dict[name_i] = msg_i
            except KeyError:
                rospy.logwarn("SchunkStatus message contains a joint I don't know from the robot_description: %s.", msg_name)


    # The actual communication loop
    def loop(self):
        hz = 10 # 10hz
        r = rospy.Rate(hz) 
        
        while not rospy.is_shutdown():
            self.targetPosition.header.stamp = rospy.Time.now()

            # Publish commands if wanted
            if self.setPosition:
                self.positionPub.publish(self.targetPosition)
                self.setPosition = False
            if self.setVelocity:
                self.velocityPub.publish(self.targetVelocity)
                self.setVelocity = False
            if self.ackJoint:
                print "/ack"
                rospy.Publisher("/schunk/ack", Int8).publish(self.ackNumber)
                self.ackJoint = False
            if self.refJoint:
                self.refJoint = False
                print "/ref"
                rospy.Publisher("/schunk/ref", Int8).publish(self.refNumber)
            if self.ackAll:
                print "/ackAll"
                rospy.Publisher("/schunk/ack_all", Empty).publish()
                self.ackAll = False
            if self.refAll:
                print "/refAll"
                rospy.Publisher("/schunk/ref_all", Empty).publish()
                self.refAll = False
            if self.maxCurrents:
                print "/currentsmaxall"
                rospy.Publisher("/schunk/set_current_max_all", Empty).publish()
                self.maxCurrents = False
            if self.emergencyStop:
                print "/emergency"
                rospy.Publisher("/schunk/emergency_stop", Empty).publish()
                self.emergencyStop = False
            
            r.sleep()
            
    def getEndPosition(self):
        frame_from = self.__root
        frame_to = self.__tip

        try:
            now = rospy.Time(0) # just get the latest rospy.Time.now()
            self.tfListener.waitForTransform(frame_from, frame_to, now, rospy.Duration(3.0))
            (trans,rot) = self.tfListener.lookupTransform(frame_from, frame_to, now)
        except (tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Can't get end effector transform!")
            return 0, 0, 0, 0, 0, 0, 0
        return trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]




class SchunkTextControl:
    def __init__(self):
        argc = len(sys.argv)
        argv = sys.argv
        
        # roscomms
        self.roscomms = RosCommunication()
        # run roscomms in a seperate thread
        self.roscommsThread = Thread(target=self.roscomms.loop)
        self.roscommsThread.start()
        
        # get number of modules
        self.numModules = self.roscomms.numModules
        
        # load gui
#        self.wTree = gtk.glade.XML("gui2.glade", "window1")
        self.wTree = gtk.Builder()
        self.wTree.add_from_file("gui3.glade") 

        # set some initial display messages
        self.wTree.get_object("labelNumModules").set_text(str(self.numModules))
        self.set_status_text("Yes, Master")
        self.commandWidget = self.wTree.get_object("command")
        
        # bindings
        bindings = {"on_window1_destroy":self.window_shutdown, 
                    "on_buttonClear_clicked":self.clear, 
                    "on_buttonExecute_clicked":self.execute, 
                    "on_command_changed":self.command_changed,
                    "on_tbHelp_toggled":self.tb_help,
                    "on_command_move_active":self.command_move_active,
                    "on_buttonSavePose_clicked":self.save_pose,
                    "on_buttonLoadPose_clicked":self.load_pose,
                    "on_buttonMoveAll_clicked":self.cb_move_all,
                    "on_buttonAckAll_clicked":self.cb_ack_all,
                    "on_buttonRefAll_clicked":self.cb_ref_all,
                    "on_tbEmergency_toggled":self.emergency_stop,
                    "on_buttonMoveVelAll_clicked":self.cb_move_vel_all,
                    "on_buttonCurMax_clicked":self.cb_currents_max,
                    "on_buttonVelStop_clicked":self.cb_stop_vel_all,
                    "on_radiobuttonJointAngleDegrees_toggled":self.degrees_or_radians,
                    "on_buttonAddJointsAnglesVector_clicked":self.add_joints_angles_vector,
                    "on_buttonDialogJointAnglesNameCancel_clicked":self.dialogJointsAnglesVectorCancel,
                    "on_buttonDialogJointAnglesNameOK_clicked":self.dialogJointsAnglesVectorOK,
                    "on_comboboxDisplayJointAngles_changed":self.update_labelDisplayJointAngles,
                    "on_buttonListJointsAnglesCopyCurrent_clicked":self.copy_to_joints_angles,
                    "on_buttonListJointsAnglesRemoveCurrent_clicked":self.remove_joints_angles_vector,
                    "on_buttonListJointsAnglesSave_clicked":self.save_listof_joints_angles,
                    "on_buttonListJointsAnglesLoad_clicked":self.load_listof_joints_angles,
                    "on_dialog1_delete_event":self.dialogJointsAnglesVector_catchDeleteEvent,
                    "on_entryJointsAnglesVectorName_changed":self.entryJointsAnglesVectorName_changed,
                    "on_buttonCopyCurrent_clicked":self.on_buttonCopyCurrent_clicked }
        #self.wTree.signal_autoconnect(bindings)
        self.wTree.connect_signals(bindings)
        # Text input field of comboboxentry command is a gtk.Entry object
        entry = self.commandWidget.get_children()[0]
        entry.connect("activate", self.command_enter_pressed, self.commandWidget)
        
        # make gui close from external request like rosnode kill
        rospy.on_shutdown(lambda: gtk.main_quit()) # lambda needed, pylint: disable=W0108

        # handle history
        self.historyLength = 100
        self.history = gtk.ListStore(gobject.TYPE_STRING)
        self.historyCounter = 0
        self.history_append("")
        self.load_history()
        self.commandWidget.set_model(self.history)
        self.commandWidget.set_text_column(0)
        self.commandWidget.set_active(self.historyCounter-1)
        
        # vocabulary and auto completion
        self.completion = gtk.EntryCompletion()
        self.vocabulary = gtk.ListStore(gobject.TYPE_STRING)
        #self.words = ["help", "info", "ack", "ref", "move", "curmax", "save", "load", "vel", "setvel", "setcur" ]
        self.words = ["help", "ack", "ref", "move", "vel", "curmax", "save", "load" ]
        self.vocabulary = self.add_words(self.words)
        self.completion.set_model(self.vocabulary)
        self.completion.set_minimum_key_length(1)
        self.completion.set_text_column(0)
        self.commandWidget.child.set_completion(self.completion)
        
        # set help box
        self.wTree.get_object("labelHelp").set_text(str(self.words))

        # pose limits of joints (deg/s)
        self.pose = []
        self.modules_maxlimits = []
        self.modules_minlimits = []
        self.limitsStrings = []
        for i in range(0,self.numModules):
            self.pose.append(0)
            moduleName = self.roscomms.joint_names_list[i]
            minLimit = self.roscomms.joint_name_to_config_dict[moduleName]["min"]
            minLimit = degrees(minLimit)
            minLimit = int(minLimit) - 1
            maxLimit = self.roscomms.joint_name_to_config_dict[moduleName]["max"]
            maxLimit = degrees(maxLimit)
            maxLimit = int(maxLimit) + 1
            self.modules_minlimits.append(minLimit)
            self.modules_maxlimits.append(maxLimit)
            string = str(minLimit) + " to " + str(maxLimit)
            self.limitsStrings.append(string)
        
        # vel limits of joints (deg/s)
        self.modules_velmax = 90
        self.modules_velmin = -90
        
        # position fields
        posesframe_hbox = gtk.HBox(False, 6)
        posesframe_vboxes = []
        posesframe_labels = []
        self.posesframe_spinButtons = []
        for i in range (0,self.numModules):
            #name = "joint " + str(i) + ":"
            #name = str(i) + ":"
            name = str(i) + " (" + self.roscomms.joint_names_list[i] + "):"
            vbox = gtk.VBox(False, 0)
            posesframe_vboxes.append(vbox)
            label = gtk.Label(name)
            posesframe_labels.append(label)
            spinButton = gtk.SpinButton(digits=4)
            spinButton.set_range(self.modules_minlimits[i], self.modules_maxlimits[i])
            spinButton.set_increments(1, 5)
            spinButton.connect("activate", self.pose_spinButton_enter_pressed)
            tooltip = "Move Joint" + str(i) + " to given position"
            spinButton.set_tooltip_text(tooltip)
            self.posesframe_spinButtons.append(spinButton)
            vbox.add(label)
            vbox.add(spinButton)
            posesframe_hbox.add(vbox)
        self.wTree.get_object("posesFrame").add(posesframe_hbox)
        self.wTree.get_object("posesFrame").show_all()
        
        # velocity fields
        velframe_hbox = gtk.HBox(False, 6)
        velframe_vboxes = []
        velframe_labels = []
        self.velframe_spinButtons = []
        for i in range (0,self.numModules):
            name = str(i) + " (" + self.roscomms.joint_names_list[i] + "):"
            vbox = gtk.VBox(False, 0)
            velframe_vboxes.append(vbox)
            label = gtk.Label(name)
            velframe_labels.append(label)
            spinButton = gtk.SpinButton()
            spinButton.set_range(self.modules_velmin, self.modules_velmax)
            spinButton.set_increments(1, 5)
            spinButton.connect("activate", self.vel_spinButton_enter_pressed)
            tooltip = "Move Joint" + str(i) + " with given velocity"
            spinButton.set_tooltip_text(tooltip)
            self.velframe_spinButtons.append(spinButton)
            vbox.add(label)
            vbox.add(spinButton)
            velframe_hbox.add(vbox)
        self.wTree.get_object("velFrame").add(velframe_hbox)
        self.wTree.get_object("velFrame").show_all()
        
        # flags fields
        flagsTitles = ["Position", "Referenced", "MoveEnd", "Brake", "Warning", "Current", "Moving", "PosReached", "Error", "Error code"]
        self.flagsDict = {"Position":0, "Referenced":1, "MoveEnd":2, "Brake":3, "Warning":4, "Current":5, "Moving":6, "PosReached":7, "Error":8, "ErrorCode":9}
        self.tableFlags = gtk.Table(self.numModules+1, len(flagsTitles)+1, homogeneous=False)
        self.tableFlags.set_col_spacings(12)
        self.wTree.get_object("flagsFrame").add(self.tableFlags)
        label = gtk.Label("# (Name)")
        label.set_alignment(0, 0.5)
        self.tableFlags.attach(label, 0, 1, 0, 1)

        # table header row
        for i in range(len(flagsTitles)):
            label = gtk.Label(flagsTitles[i])
            self.tableFlags.attach(label, 1+i, 1+i+1, 0, 1) # skip first column

        # for each joint every column
        self.flags = []
        for i in range(self.numModules):
            # joint index
            label = gtk.Label(str(i) + " (" + self.roscomms.joint_names_list[i] + "):")
            #label.set_justify(gtk.JUSTIFY_LEFT)
            label.set_alignment(0, 0.5)
            self.tableFlags.attach(label, 0, 1, 1+i, 1+i+1) # need to skip first title row

            # joint flags
            flagsRow = []
            for j in range(len(flagsTitles)):
                label = gtk.Label(".")
                self.tableFlags.attach(label, 1+j, 1+j+1, 1+i, 1+i+1) # also skip first index column
                flagsRow.append(label)
            self.flags.append(flagsRow)
        self.wTree.get_object("flagsFrame").show_all()
                        
        # no argument full interface, also medium and mini modes
        if argc > 1:
            if (argv[1] == "medium") or (argv[1] == "mini"):
                self.wTree.get_object("aPoseFrame").hide()
                self.wTree.get_object("aVelFrame").hide()
            if argv[1] == "mini":
                self.wTree.get_object("aFlagsFrame").hide()
        w = self.wTree.get_object("window1")
        w.resize(*w.size_request())
        
        # in degrees
        self.inDegrees = self.wTree.get_object("radiobuttonJointAngleDegrees").get_active()
        
        # list of joints angles
        self.wTree.get_object("entryJointsAnglesVectorName").connect("activate", self.dialogJointsAnglesName_enter_pressed)
        self.listJointsAngles = []
        self.dictJointsAngles = {}
        self.combolistJointsAngles = self.wTree.get_object("combolistJointsAngles")
        self.comboboxJointsAngles = self.wTree.get_object("comboboxDisplayJointAngles")
        cell = gtk.CellRendererText()
        self.comboboxJointsAngles.pack_start(cell, True)
        self.comboboxJointsAngles.add_attribute(cell, "text", 0)
        self.dictJointsAngles_set_appropriate_buttons_sensitive()
        

    def window_shutdown(self, widget):
        # kill gtk thread
        gtk.main_quit()

        # kill ros thread
        rospy.signal_shutdown("Because I said so!")
        
        # Wait for roscomm thread to stop
        self.roscommsThread.join()


    def set_status_text_info(self, status_string):
        self.set_status_text('Info: '+status_string, '#000000')

    def set_status_text_error(self, status_string):
        self.set_status_text('Error: '+status_string, '#FF0000')
        
    def set_status_text_warning(self, status_string):
        self.set_status_text('Warning: '+status_string, '#FF00FF')
    
    def set_status_text(self, status_string, color_string=None):
        """
        Each argument can be set to None to be ignored and only change the other value.
        """
        if status_string is not None:
            self.wTree.get_object("status").set_text(status_string)
        if color_string is not None:
            self.wTree.get_object("status").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse(color_string))


    def parser(self, string):
        tokens = string.split()
        if tokens[0] == "ack":
            self.ack(tokens)
        elif tokens[0] == "ref":
            self.ref(tokens)
        elif tokens[0] == "move":
            self.move(tokens)
        elif tokens[0] == "vel":
            self.move_vel(tokens)
        elif tokens[0] == "curmax":
            self.currents_max(tokens)
        elif tokens[0] == "help":
            self.help()
        else:
            self.command_not_found(tokens[0])


    def command_enter_pressed(self, entry, combo):
        self.wTree.get_object("buttonExecute").activate()


    def clear(self, widget):
        self.commandWidget.set_active(self.historyCounter-1)


    def execute(self, widget):
        commandStr = self.commandWidget.get_active_text()
        if commandStr != "":
            #iterator = self.store.get_iter_from_string("")
            #self.store.insert_before(iterator, [commandStr])
            self.history_append(commandStr)
            self.parser(commandStr)
            self.commandWidget.set_active(self.historyCounter-1)


    def emergency_stop(self, widget):
        if widget.get_active():
            # STOP
            self.roscomms.emergencyStop = True
            self.wTree.get_object("aPoseFrame").set_sensitive(False)
            self.wTree.get_object("aVelFrame").set_sensitive(False)
            self.wTree.get_object("aFlagsFrame").set_sensitive(False)
            self.wTree.get_object("vboxCommand").set_sensitive(False)
            self.wTree.get_object("image1").set_from_file("go75.png")
            self.set_status_text("Astalavista baby. No way Master Yianni's fault", '#FF0000')
        else:
            # GO
            #self.roscomms.emergencyStop = False
            #self.ack("ack all".split())
            for i in range(0,self.numModules):
                command = "ack " + str(i) 
                self.ack(command.split())
                while self.roscomms.ackJoint == True:
                    pass
            self.wTree.get_object("aPoseFrame").set_sensitive(True)
            self.wTree.get_object("aVelFrame").set_sensitive(True)
            self.wTree.get_object("aFlagsFrame").set_sensitive(True)
            self.wTree.get_object("vboxCommand").set_sensitive(True)
            self.wTree.get_object("image1").set_from_file("stop75.png")
            self.set_status_text("I am back, Master", '#000000')


    def load_history(self):
        try:
            f = open("history", "r")
            lines = f.readlines()
            f.close()
            lines = lines[-self.historyLength:]
            for line in lines:
                line = line.rstrip("\n")
                self.history_append(line, False)
        except IOError:
            pass


    def history_append_to_file(self, string):
        f = open("history", "a")
        f.write(string + "\n")
        f.close()


    def history_append(self, string, tofile=True):
        self.history.insert(self.historyCounter-1, [string])
        self.historyCounter += 1
        if tofile and (string != ""):
            self.history_append_to_file(string)


    def command_move_active(self, widget, arg2):
        # following should bring the cursor to the end but it does not unless you hit the up arrow twice and this works only on the top item
        widget.child.set_position(-1)


    def save_pose(self, widget):
        # get the joint angles from the robot or from the pose display
        #eg.
        jointAngles = []
        for spinButton in self.posesframe_spinButtons:
            print spinButton.get_value()
            jointAngles.append(spinButton.get_value())
        dialog = gtk.FileChooserDialog(title="Save pose (.pos)",
                                       action=gtk.FILE_CHOOSER_ACTION_SAVE,
                                       buttons=(gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL, 
                                                gtk.STOCK_SAVE, gtk.RESPONSE_OK))
        dialog.set_default_response(gtk.RESPONSE_OK)
        dialog.set_do_overwrite_confirmation(True)
        response = dialog.run()
        if response == gtk.RESPONSE_OK:
            filename = dialog.get_filename()
            writer = csv.writer(open(filename, "wb"))
            writer.writerow(jointAngles)
        elif response == gtk.RESPONSE_CANCEL:
            pass
        dialog.destroy()
        

    def load_pose(self, widget):
        dialog = gtk.FileChooserDialog(title="Load pose (.pos)", 
                                       action=gtk.FILE_CHOOSER_ACTION_OPEN, 
                                       buttons=(gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL,
                                                 gtk.STOCK_OPEN, gtk.RESPONSE_OK))
        dialog.set_default_response(gtk.RESPONSE_OK)
        dialog.select_filename("default.pose")
        response = dialog.run()
        if response == gtk.RESPONSE_OK:
            filename = dialog.get_filename()
            reader = csv.reader(open(filename, "rb"))
            for row in reader:
                for i in range(0, min(len(self.pose), len(row)) ):  # do not try to load more row items than joints, or more joints than row items
                    try:
                        self.pose[i] = float(row[i])
                    except:
                        print "error in loading pose"
                        return
            self.update_pose_display()
        elif response == gtk.RESPONSE_CANCEL:
            pass
        dialog.destroy()


    def add_joints_angles_vector(self, widget):
        name = self.find_unique_name_for_joints_angles_vector()
        dialogText = self.wTree.get_object("entryJointsAnglesVectorName") 
        dialogText.set_text(name)
        self.wTree.get_object("labelDialogJointsAnglesErrorMessage").set_text("")
        if self.wTree.get_object("comboboxDisplayJointAngles").get_active() >= 0:
            self.wTree.get_object("radiobuttonDialogAddJointsAnglesAfter").set_sensitive(True)
            self.wTree.get_object("radiobuttonDialogAddJointsAnglesBefore").set_sensitive(True)
            name = self.wTree.get_object("comboboxDisplayJointAngles").get_active_text()
            index = self.dictJointsAngles[name]
            text = str(self.listJointsAngles[index][0]) + ": " + str(self.listJointsAngles[index][1])
            self.wTree.get_object("labelDialogAddJointsAnglesIndex").set_text(text)
        else:
            self.wTree.get_object("radiobuttonDialogAddJointsAnglesAfter").set_sensitive(False)
            self.wTree.get_object("radiobuttonDialogAddJointsAnglesBefore").set_sensitive(False)
            self.wTree.get_object("labelDialogAddJointsAnglesIndex").set_text("")
        self.wTree.get_object("dialog1").show()      


    def entryJointsAnglesVectorName_changed(self, widget):
        name = self.wTree.get_object("entryJointsAnglesVectorName").get_text()
        if name in self.dictJointsAngles:
            self.wTree.get_object("labelDialogJointsAnglesErrorMessage").set_text("Label already exists")
            self.wTree.get_object("labelDialogJointsAnglesErrorMessage").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
        elif name == "":
            self.wTree.get_object("labelDialogJointsAnglesErrorMessage").set_text("Label is empty")
            self.wTree.get_object("labelDialogJointsAnglesErrorMessage").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
        else:        
            self.wTree.get_object("labelDialogJointsAnglesErrorMessage").set_text("")


    def dialogJointsAnglesVectorCancel(self, widget):
        self.wTree.get_object("dialog1").hide()


    def dialogJointsAnglesVectorOK(self, widget):
        name = self.wTree.get_object("entryJointsAnglesVectorName").get_text()
        if name in self.dictJointsAngles:
            self.wTree.get_object("labelDialogJointsAnglesErrorMessage").set_text("Label already exists")
            self.wTree.get_object("labelDialogJointsAnglesErrorMessage").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
        elif name == "":
            self.wTree.get_object("labelDialogJointsAnglesErrorMessage").set_text("Label is empty")
            self.wTree.get_object("labelDialogJointsAnglesErrorMessage").modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
        else:
            jointsAngles = []
            for spinButton in self.posesframe_spinButtons:
                jointsAngles.append(spinButton.get_value())
            if self.wTree.get_object("radiobuttonDialogAddJointsAnglesEnd").get_active():
                index = len(self.listJointsAngles)
            elif self.wTree.get_object("radiobuttonDialogAddJointsAnglesAfter").get_active():
                index = self.dictJointsAngles[self.wTree.get_object("comboboxDisplayJointAngles").get_active_text()] + 1
            elif self.wTree.get_object("radiobuttonDialogAddJointsAnglesBefore").get_active():
                index = self.dictJointsAngles[self.wTree.get_object("comboboxDisplayJointAngles").get_active_text()]
            else:
                index = len(self.listJointsAngles)
            line = [name, jointsAngles]
            self.dictJointsAngles[name] = index
            self.listJointsAngles.insert(index, line)
            self.combolistJointsAngles.insert(index, [name])
            self.dictJointsAngles_set_appropriate_buttons_sensitive()
            self.wTree.get_object("dialog1").hide()
    
    
    def dialogJointsAnglesName_enter_pressed(self, entry):
        self.wTree.get_object("buttonDialogJointAnglesNameOK").activate()
    

    def dialogJointsAnglesVector_catchDeleteEvent(self, widget, data=None):
        widget.hide()
        return True


    def update_labelDisplayJointAngles(self, widget):
        try:
            label = self.wTree.get_object("labelDisplayJointsAngles")
            name = self.comboboxJointsAngles.get_active_text()
            index = self.dictJointsAngles[name]
            angles = self.listJointsAngles[index][1]
            
            pezz_text = ' , '.join(["%.2f" %i for i in angles])
#            label.set_text(str(angles))
            label.set_text(pezz_text)
            self.dictJointsAngles_set_appropriate_buttons_sensitive()
        except:
            pass
    
    
    def copy_to_joints_angles(self, widget):
        try:
            name = self.comboboxJointsAngles.get_active_text()
            index = self.dictJointsAngles[name]
            angles = self.listJointsAngles[index][1]
            for i in range(self.numModules):
                value = angles[i]
                self.posesframe_spinButtons[i].set_value(value)
        except:
            print "Error occured in function <copy_to_joints_angles>"


    def remove_joints_angles_vector(self, widget):
        comboIndex = self.comboboxJointsAngles.get_active()
        if comboIndex >= 0:
            name = self.comboboxJointsAngles.get_active_text()
            try:
                index = self.dictJointsAngles[name]
                del self.listJointsAngles[index]
                del self.dictJointsAngles[name]
                for k in self.dictJointsAngles.iterkeys():
                    if self.dictJointsAngles[k] > index:
                        self.dictJointsAngles[k] -= 1
                treestore = self.combolistJointsAngles
                treeiter = treestore.iter_nth_child(None, comboIndex)
                self.combolistJointsAngles.remove(treeiter)
                self.wTree.get_object("labelDisplayJointsAngles").set_text("")
            except:
                print "bad"
                return
        self.dictJointsAngles_set_appropriate_buttons_sensitive()


    def save_listof_joints_angles(self, widget):        
        dialog = gtk.FileChooserDialog(title="Save list of joints angles (.lsa)",
                                       action=gtk.FILE_CHOOSER_ACTION_SAVE,
                                       buttons=(gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL, 
                                                gtk.STOCK_SAVE, gtk.RESPONSE_OK))
        dialog.set_default_response(gtk.RESPONSE_OK)
        dialog.set_do_overwrite_confirmation(True)
        response = dialog.run()
        if response == gtk.RESPONSE_OK:
            filename = dialog.get_filename()
            try:
                w = csv.writer(open(filename, "wb"), delimiter=':', quoting=csv.QUOTE_NONE)
                w.writerows(self.listJointsAngles)
            except:
                print "failed to write to file %s (save_listof_joints_angles)", filename
            #pickle.dump(self.dictJointsAngles, open(filename, "wb"))
        elif response == gtk.RESPONSE_CANCEL:
            pass
        dialog.destroy()


    def load_listof_joints_angles(self, widget):
        dialog = gtk.FileChooserDialog(title="Load pose", 
                                       action=gtk.FILE_CHOOSER_ACTION_OPEN, 
                                       buttons=(gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL,
                                                 gtk.STOCK_OPEN, gtk.RESPONSE_OK))
        dialog.set_default_response(gtk.RESPONSE_OK)
        dialog.select_filename("default.list")
        response = dialog.run()
        if response == gtk.RESPONSE_OK:
            filename = dialog.get_filename()
            #self.dictJointsAngles = pickle.load(open(filename))
            try:
                # each line looks like: "cam down:[0.0, 30.0, 75.0, -135.0, 0.0]"
                r = csv.reader(open(filename, "rb"), delimiter=':', quoting=csv.QUOTE_NONE)
                del self.listJointsAngles[:]
                for row in r:
                    key = row[0]
                    values = map(float, row[1].strip("[]").split(","))
                    line = [key, values]
                    self.listJointsAngles.append(line)
                self.combolistJointsAngles.clear()
                for i in range(len(self.listJointsAngles)):
                    key = str(self.listJointsAngles[i][0])
                    self.dictJointsAngles[key] = i
                    self.combolistJointsAngles.append([key])            
                self.wTree.get_object("labelDisplayJointsAngles").set_text("")                    
            except:
                print "failed to load from file %s (load_listof_joints_angles)", filename
        elif response == gtk.RESPONSE_CANCEL:
            pass
        self.dictJointsAngles_set_appropriate_buttons_sensitive()
        dialog.destroy()


    def find_unique_name_for_joints_angles_vector(self):
        i = 0
        while True:
            name = "joints_angles_" + str(i)
            if name in self.dictJointsAngles:
                i += 1
            else:
                return name


    def dictJointsAngles_set_appropriate_buttons_sensitive(self):
        value = (self.comboboxJointsAngles.get_active() >= 0)
        self.wTree.get_object("buttonListJointsAnglesCopyCurrent").set_sensitive(value)
        self.wTree.get_object("buttonListJointsAnglesRemoveCurrent").set_sensitive(value)


    def add_words(self, words):
        vocabulary = gtk.ListStore(gobject.TYPE_STRING)
        for word in words:
            vocabulary.append([word])
        return vocabulary


    def command_changed(self, widget):
        tokens = widget.get_active_text().split()
        if tokens != []:
            self.set_status_text(None, '#000000')  # TODO: check if needed or following call can be replaced with _info
        try:
            if tokens[0] == "move":
                try:
                    module = int(tokens[1])
                    try:
                        string = "Range (deg/s): " + self.limitsStrings[module]
                        self.set_status_text(string)
                        try:
                            value = int(tokens[2])
                            if value > self.modules_maxlimits[module] or value < self.modules_minlimits[module]:
                                self.set_status_text_warning("Let me see you licking your elbow mate")
                        except:
                            pass
                    except:
                        self.set_status_text_warning("module does not exist")
                except:
                    self.set_status_text("")
                    
            elif tokens[0] == "vel":
                try:
                    module = int(tokens[1])
                    try:
                        # this is actually the joint angle limits and not the velocity
                        # but it helps to detect whether the module entered exists
                        moduleExists = self.modules_maxlimits[module]  # @UnusedVariable
                        string = "Range (deg/s): " + str(self.modules_velmin) + " to " + str(self.modules_velmax)
                        self.set_status_text(string)
                        try:
                            value = int(tokens[2])
                            if value > self.modules_velmax or value < self.modules_velmin:
                                self.set_status_text_warning("Hope you have a safe distance mate")
                        except:
                            pass
                    except:
                        self.set_status_text_warning("module does not exist")
                except:
                    self.set_status_text("")
            else:
                try:
                    module = int(tokens[1])
                    if (module >= self.numModules) or (module < 0):
                        self.set_status_text_warning("module does not exist")
                except:
                    self.set_status_text("")
        except:
            pass


    def tb_help(self, widget):
        label = self.wTree.get_object("labelHelp")
        if widget.get_active():
            label.show()
        else:
            label.hide()


    def help(self):
        self.wTree.get_object("tbHelp").set_active(True)


    def update_pose_display(self):
        for i in range(0,len(self.pose)):
            try:
                self.posesframe_spinButtons[i].set_value(self.pose[i])
            except:
                print "error message on " + str(i)
                return


    def cb_ack_all(self, widget):
        command = "ack all"
        tokens = command.split()
        self.ack(tokens)
        # yes I know it could be
        #self.ack("ack all".split())


    def ack(self, tokens):
        try:
            module = tokens[1]
            if module == "all":
                self.roscomms.ackAll = True
                return
            try:
                module = int(module)
                if module >= 0 and module < self.numModules:
                    self.roscomms.ackNumber = module
                    self.roscomms.ackJoint = True
                else:
                    self.set_status_text_error("ack failed. Module does not exist")
            except:
                self.set_status_text_error("move velocity failed. Module does not exist")
        except:
            self.roscomms.ackAll = True


    def cb_ref_all(self, widget):
        command = "ref all"
        tokens = command.split()
        self.ref(tokens)
        #or yes I know it could be
        #self.ref("ref all".split())


    def ref(self, tokens):
        try:
            module = tokens[1]
            if module == "all":
                self.roscomms.refAll = True
                return
            try:
                module = int(module)
                if module >= 0 and module < self.numModules:
                    self.roscomms.refNumber = module
                    self.roscomms.refJoint = True
                else:
                    self.set_status_text_error("ref failed. Module does not exist")
            except:
                self.set_status_text_error("ref failed. Module does not exist")
        except:
            self.set_status_text_error("ref failed. Need to specify module id or 'all'")


    def cb_move_all(self, widget):
        self.move_all()


    def pose_spinButton_enter_pressed(self, widget):
        module = self.posesframe_spinButtons.index(widget)
        self.posesframe_spinButtons[module].update()
        value = float(self.posesframe_spinButtons[module].get_value())
        command = "move " + str(module) + " " + str(value)
        tokens = command.split()
        self.move(tokens)


    def move(self, tokens):
        try:
            module = tokens[1]
            if module == "all":
                self.move_all()
                return
            try:
                module = int(module)
                if module >= 0 and module < self.numModules:
                    try:
                        value = tokens[2]
                        if self.inDegrees:
                            valueCheckLimit = int(value)
                        else:
                            valueCheckLimit = degrees(float(value))
                            valueCheckLimit = int(valueCheckLimit)
                        if valueCheckLimit > self.modules_maxlimits[module] or valueCheckLimit < self.modules_minlimits[module]:
                            self.set_status_text_error("I told you I can't lick my elbow. Move failed")
                            return
                        try:
                            value = float(value)
                            if self.inDegrees:
                                value = radians(value)
                            self.roscomms.targetPosition.name=[]
                            self.roscomms.targetPosition.name.append(self.roscomms.joint_names_list[module])
                            self.roscomms.targetPosition.position = [value]
                            #print self.roscomms.targetPosition
                            self.roscomms.setPosition = True
                        except:
                            print "move failed: not valid value"
                    except:
                        # move from spinbutton if tokens[2] not given
                        value = radians(float(self.posesframe_spinButtons[module].get_value()))
                        self.roscomms.targetPosition.name=[]
                        self.roscomms.targetPosition.name.append(self.roscomms.joint_names_list[module])
                        self.roscomms.targetPosition.position = [value]
                        #print self.roscomms.targetPosition
                        self.roscomms.setPosition = True 
                else:
                    self.set_status_text_error("move failed. Module does not exist")
            except:
                self.set_status_text_error("move failed. Module does not exist")
        except:
            self.set_status_text_error("move failed. Need to specify module id or 'all'")      


    def move_all(self):
        self.roscomms.targetPosition.name=[]
        self.roscomms.targetPosition.position=[]
        for module in range(0,self.numModules):
            name = self.roscomms.joint_names_list[module]
            self.roscomms.targetPosition.name.append(name)
            value = float(self.posesframe_spinButtons[module].get_value())
            if self.inDegrees: # convert to radians, else it is already in radians
                value = radians(value)
            self.roscomms.targetPosition.position.append(value)
            #print roscomms.targetPosition
            self.roscomms.setPosition = True


    def cb_currents_max(self, widget):
        command = "curmax"
        tokens = command.split()
        self.currents_max(tokens)


    def currents_max(self, tokens):
        try:
            module = tokens[1]
            if module == "all":
                self.roscomms.maxCurrents = True
                return
            try:
                module = int(module)
                if module >= 0 and module < self.numModules:
                    pass
                else:
                    self.set_status_text_error("currents max failed. Module does not exist")
            except:
                self.set_status_text_error("currents max failed. Module does not exist")
        except:
            self.roscomms.maxCurrents = True
    

    def cb_move_vel_all(self, widget):
        self.move_vel_all()

        
    def cb_stop_vel_all(self, widget):
        self.roscomms.targetVelocity.name=[]
        self.roscomms.targetVelocity.velocity=[]
        for module in range(0,self.numModules):
            name = self.roscomms.joint_names_list[module]
            self.roscomms.targetVelocity.name.append(name)
            value = 0.0
            self.roscomms.targetVelocity.velocity.append(value)
            self.roscomms.setVelocity = True
#        for i in range(0,self.numModules):
#            command = "vel " + str(i) + " 0"
#            tokens = command.split()
#            self.move_vel(tokens)

            
    def move_vel(self, tokens):
        try:
            module = tokens[1]
            if module == "all":
                self.move_vel_all()
                return
            try:
                module = int(module)
                if module >= 0 and module < self.numModules:
                    try:
                        value = tokens[2]
                        if int(value) > self.modules_velmax or int(value) < self.modules_velmin:
                            self.set_status_text_error("Can't go at speed of light. Move velocity failed")
                            return
                        try:
                            value = radians(float(value))
                            self.roscomms.targetVelocity.name=[]
                            self.roscomms.targetPosition.name.append(self.roscomms.joint_names_list[module])
                            self.roscomms.targetVelocity.velocity = [value]
                            self.roscomms.setVelocity = True
                        except:
                            print "move_vel failed: not valid value"
                    except:
                        # move from spinbutton if tokens[2] not given
                        value = radians(float(self.velframe_spinButtons[module].get_value()))
                        self.roscomms.targetVelocity.name=[]
                        self.roscomms.targetPosition.name.append(self.roscomms.joint_names_list[module])
                        self.roscomms.targetVelocity.velocity = [value]
                        self.roscomms.setVelocity = True 
                else:
                    self.set_status_text_error("move velocity failed. Module does not exist")
            except:
                self.set_status_text_error("move velocity failed. Module does not exist")
        except:
            self.set_status_text_error("move velocity failed. Need to specify module id or 'all'")
      

    def move_vel_all(self):
        self.roscomms.targetVelocity.name=[]
        self.roscomms.targetVelocity.velocity=[]
        for i in range(0,self.numModules):
            name = self.roscomms.joint_names_list[i]
            self.roscomms.targetVelocity.name.append(name)
            value = radians(float(self.velframe_spinButtons[i].get_value()))
            self.roscomms.targetVelocity.velocity.append(value)
            self.roscomms.setVelocity = True


    def vel_spinButton_enter_pressed(self, widget):
        module = self.velframe_spinButtons.index(widget)
        self.velframe_spinButtons[module].update()
        value = float(self.velframe_spinButtons[module].get_value())
        command = "vel " + str(module) + " " + str(value)
        tokens = command.split()
        self.move_vel(tokens)


    def degrees_or_radians(self, widget):
        self.inDegrees = widget.get_active()
        self.wTree.get_object("hboxListJointsVectors").set_sensitive(self.inDegrees)
        self.wTree.get_object("buttonAddJointsAnglesVector").set_sensitive(self.inDegrees)
        if self.inDegrees:
            #self.wTree.get_object("labelJointAngles").set_text("Joint angles (deg)")
            for i in range(0,self.numModules):
                value = float(self.posesframe_spinButtons[i].get_value())
                value = degrees(value)
                self.posesframe_spinButtons[i].set_range(self.modules_minlimits[i], self.modules_maxlimits[i])
                self.posesframe_spinButtons[i].set_value(value)
                self.posesframe_spinButtons[i].update()
        else:
            #self.wTree.get_object("labelJointAngles").set_text("Joint angles (rad)")
            for i in range(0,self.numModules):
                value = float(self.posesframe_spinButtons[i].get_value())
                value = radians(value)
                self.posesframe_spinButtons[i].set_range(radians(self.modules_minlimits[i]), radians(self.modules_maxlimits[i]))
                self.posesframe_spinButtons[i].set_value(value)
                self.posesframe_spinButtons[i].update()
   

    def update_flags(self, *args):
        for module_i in range(self.numModules):
            ## joint state
            try:
                # lookup index in message for current module
                msg_i = self.roscomms.currentJointStates_jointIndex_to_msgIndex_dict[module_i]
                
                label = self.flags[module_i][self.flagsDict["Position"]]
                flagRadians = self.roscomms.currentJointStates.position[msg_i]            
                flag = degrees(flagRadians)
                if (flag < 0.05) and (flag > -0.05):
                    flag = 0.0            
                string = "%.2f / %.3f" % (flag, flagRadians)
                label.set_text(string)
                #flag = round(flag, 2)
                #label.set_text(str(flag))
            except KeyError:
                self.set_status_text_error("Joint '"+self.roscomms.joint_names_list[module_i]+"' not found in JointState message!")
            
            ## schunk status
            try:
                # lookup index in message for current module
                msg_i = self.roscomms.currentSchunkStatus_jointIndex_to_msgIndex_dict[module_i]

                label = self.flags[module_i][self.flagsDict["Referenced"]]
                flag = self.roscomms.currentSchunkStatus.joints[msg_i].referenced
                label.set_text(str(flag))
                if not flag:
                    label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
                else:
                    label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#000000'))
                    
                label = self.flags[module_i][self.flagsDict["MoveEnd"]]
                flag = self.roscomms.currentSchunkStatus.joints[msg_i].moveEnd
                label.set_text(str(flag))
                if not flag:
                    label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
                else:
                    label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#000000'))

                label = self.flags[module_i][self.flagsDict["Brake"]]
                flag = self.roscomms.currentSchunkStatus.joints[msg_i].brake
                label.set_text(str(flag))
                if not flag:
                    label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
                else:
                    label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#000000'))


                label = self.flags[module_i][self.flagsDict["Warning"]]
                flag = self.roscomms.currentSchunkStatus.joints[msg_i].warning
                label.set_text(str(flag))
                if flag:
                    label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
                else:
                    label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#000000'))

                label = self.flags[module_i][self.flagsDict["Current"]]
                flag = self.roscomms.currentSchunkStatus.joints[msg_i].current
                string = "%.2f" % flag
                label.set_text(string)
                #flag = round(flag,2)
                #label.set_text(str(flag))

                label = self.flags[module_i][self.flagsDict["Moving"]]
                flag = self.roscomms.currentSchunkStatus.joints[msg_i].moving
                label.set_text(str(flag))
                if flag:
                    label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
                else:
                    label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#000000'))

                label = self.flags[module_i][self.flagsDict["PosReached"]]
                flag = self.roscomms.currentSchunkStatus.joints[msg_i].posReached
                label.set_text(str(flag))
                if not flag:
                    label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
                else:
                    label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#000000'))
     
                label = self.flags[module_i][self.flagsDict["Error"]]
                flag = self.roscomms.currentSchunkStatus.joints[msg_i].error
                label.set_text(str(flag))
                if flag:
                    label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
                else:
                    label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#000000'))
                    
                label = self.flags[module_i][self.flagsDict["ErrorCode"]]
                flag = self.roscomms.currentSchunkStatus.joints[msg_i].errorCode
                label.set_text(str(flag))
                if flag != 0:
                    label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#FF0000'))
                else:
                    label.modify_fg(gtk.STATE_NORMAL, gtk.gdk.color_parse('#000000'))
            except:
                self.set_status_text_error("Joint '"+self.roscomms.joint_names_list[module_i]+"' not found in SchunkStatus message!")
            
        return True


    def on_buttonCopyCurrent_clicked(self, widget):
        for module_i in range(self.numModules):
            try:
                # lookup index in message for current module
                msg_i = self.roscomms.currentJointStates_jointIndex_to_msgIndex_dict[module_i]
                
                posRadians = float(self.roscomms.currentJointStates.position[msg_i])
                if self.inDegrees:
                    posDegrees = degrees(posRadians)
                    if (posDegrees < 0.05) and (posDegrees > -0.05):
                        posDegrees = 0.0
#Neither work                        
#                    string = "%.4f" % posDegrees
#                    posDegrees = float(string)
# or
#                    posDegrees = round(posDegrees, 4)
                    self.posesframe_spinButtons[module_i].set_value(posDegrees)
                else:
#                    posRadians = round(posRadians, 4)
                    self.posesframe_spinButtons[module_i].set_value(posRadians)
            except KeyError:
                self.set_status_text_error("Joint '"+self.roscomms.joint_names_list[module_i]+"' not found in joint state message!")


    def update_pose(self, *args):
        value = self.roscomms.getEndPosition()
        pose = []
        for v in value:
            pose.append(v)
        for i in range(0,len(pose)):
            if pose[i] < 0.005 and pose[i] > -0.005:
                pose[i] = 0.0
        
        value = quaternion_to_euler(pose[3], pose[4], pose[5], pose[6])
        rpy = []
        for v in value:
            rpy.append(degrees(v))
        for i in range(0,len(rpy)):
            if rpy[i] < 0.005 and rpy[i] > -0.005:
                rpy[i] = 0.0

        msg = "%.2f" % pose[0]
        self.wTree.get_object("poseX").set_text(msg)
        msg = "%.2f" % pose[1]
        self.wTree.get_object("poseY").set_text(msg)
        msg = "%.2f" % pose[2]
        self.wTree.get_object("poseZ").set_text(msg)
        msg = "%.2f" % rpy[0]
        self.wTree.get_object("poseRoll").set_text(msg)
        msg = "%.2f" % rpy[1]
        self.wTree.get_object("posePitch").set_text(msg)
        msg = "%.2f" % rpy[2]
        self.wTree.get_object("poseYaw").set_text(msg)
        msg = "%.2f" % pose[3]
        self.wTree.get_object("poseQx").set_text(msg)
        msg = "%.2f" % pose[4]
        self.wTree.get_object("poseQy").set_text(msg)
        msg = "%.2f" % pose[5]
        self.wTree.get_object("poseQz").set_text(msg)
        msg = "%.2f" % pose[6]
        self.wTree.get_object("poseQw").set_text(msg)

        return True


    def command_not_found(self, token):
        msg = "Ich spreche nicht Deutch. Was ist '" +  token + "'? Druckte 'Hilfe' fur Vokabelliste"
        self.set_status_text_error(msg)

# delete not needed any more
#    def get_limits_strings(self):
#        limitsStrings = {}
#        for i in range(0,self.numModules):
#            min = self.modules_minlimits[i]
#            max = self.modules_maxlimits[i]
#            string = str(min) + " to " + str(max)
#            limitsStrings[i] = string
#        return limitsStrings


def quaternion_to_euler(qx,qy,qz,qw):
    heading = math.atan2(2*qy*qw-2*qx*qz , 1 - 2*qy*qy - 2*qz*qz)
    attitude = math.asin(2*qx*qy + 2*qz*qw)
    bank = math.atan2(2*qx*qw-2*qy*qz , 1 - 2*qx*qx - 2*qz*qz)
    if math.fabs(qx*qy + qz*qw - 0.5) < 0.001: # (north pole):
        heading = 2 * math.atan2(qx,qw)
        bank = 0
    if math.fabs(qx*qy + qz*qw + 0.5) < 0.001: # (south pole):
        heading = -2 * math.atan2(qx,qw)   
        bank = 0

    return heading, attitude, bank


if __name__ == "__main__":
    wd = os.path.dirname(sys.argv[0])
    try:
        os.chdir(wd)
    except OSError:
        print "Working directory does not exist. Check for mispellings"
        sys.exit(1)
    
    gtk.gdk.threads_init()
    rospy.init_node('schunk_gui')
    gui = SchunkTextControl()
    #Thread(target=gui.roscomms.loop).start() # statement is in the constructor of SchunkTextControl, either there or here
    gobject.timeout_add(100, gui.update_flags)
    gobject.timeout_add(100, gui.update_pose)
    gtk.main()
    rospy.spin()
