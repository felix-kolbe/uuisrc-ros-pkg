
#include <boost/thread.hpp>
#include <boost/noncopyable.hpp>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <dynamic_reconfigure/server.h>
#include <metralabs_ros/ScitosG5Config.h>

#include <urdf/model.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>

#include <metralabs_ros/movePosition.h>
#include <metralabs_ros/idAndFloat.h>
#include <metralabs_ros/SchunkStatus.h>

#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/JointState.h>

#include <geometry_msgs/TransformStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <robot/RangeData.h>
using namespace MetraLabs::robotic::robot;

#include "ScitosBase.h"
#include "PowerCube.h"

#define SCHUNK_NOT_AMTEC 0

#define DEG_TO_RAD(d)	((d)*M_PI/180.0)
#define RAD_TO_DEG(r)	((r)*180.0/M_PI)

#define FEATURE_LASER	( "EBC0_Enable24V" )
#define FEATURE_ARM		( "EBC1_Enable24V" )
#define FEATURE_SONAR	( "SonarsActive" )

#define SLOWEST_REASONABLE_JOINT_SPEED	0.015f // the slowest reasonable speed for our joints (IMHO)
#define TOO_SLOW	0.002f //





class TrajectoryExecuter : private boost::noncopyable {

public:

	TrajectoryExecuter(ros::NodeHandle& nh_, std::string action_server_name) :
		as_(nh_, action_server_name, boost::bind(&TrajectoryExecuter::executeCB, this, _1), false),
		action_name_(action_server_name)
	{
		run = false;
		arm = NULL;
	}

	void init(ros::NodeHandle& n, PowerCube* p_arm, std::map<std::string, unsigned int>& nmap) {

		arm = p_arm;
		nameToNumber = nmap;

		state_publisher = n.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>("state", 1);

		std::map<std::string, unsigned int>::iterator it;
		for (it = nameToNumber.begin(); it != nameToNumber.end(); ++it) {
			state_msg.joint_names.push_back((*it).first );
		}
		state_msg.desired.positions.resize(nameToNumber.size());
		state_msg.desired.velocities.resize(nameToNumber.size());
		state_msg.desired.accelerations.resize(nameToNumber.size());
		state_msg.actual.positions.resize(nameToNumber.size());
		state_msg.actual.velocities.resize(nameToNumber.size());
		state_msg.error.positions.resize(nameToNumber.size());
		state_msg.error.velocities.resize(nameToNumber.size());

		as_.start();
	}

	void main() {
		while (true) {
			{
				boost::unique_lock<boost::mutex> lock(mut);
				waiting = true;
				bool gotit = false;
				while (!run){
					try {
						boost::this_thread::interruption_point();
						gotit = cond.timed_wait(lock, boost::posix_time::milliseconds(100));
					}
					catch(const boost::thread_interrupted&) {
						ROS_WARN("Trajectory thread was interrupted and returns now.");
						arm->pc_normal_stop();
						return;
					}
					if (! gotit) {
						//Let's make people happy by publishing a state message
						for (unsigned int joint_i = 0; joint_i<nameToNumber.size(); ++joint_i) {
							//fill in the message
							state_msg.desired = state_msg.actual;
							state_msg.actual.positions[joint_i] = RAD_TO_DEG(arm->mManipulator.getModules().at(joint_i).status_pos);
							state_msg.actual.velocities[joint_i] = 0;
							state_msg.actual.time_from_start = ros::Duration(0);
						}

						//finally publish the state message
						state_msg.header.stamp = ros::Time::now();
						state_publisher.publish(state_msg);
					}

				}
			}
			//I hope the mutex is unlocked now
			{
				boost::unique_lock<boost::mutex> lock(mut);
				waiting = false;
			}

			ROS_INFO("Trajectory thread is starting its job");
			follow_trajectory();
			arm->pc_normal_stop();

			{
				//Self deactivate
				boost::unique_lock<boost::mutex> lock(mut);
				run = false;
			}
			ROS_INFO("Trajectory thread has finished its job");
		}
		return; //just to be sure the thread is terminated
	}

	void start(const trajectory_msgs::JointTrajectory& newtraj) {
		{
			boost::unique_lock<boost::mutex> lock(mut);
			if (run) {
				ROS_WARN("You need to stop a trajectory before issuing a new command. Ignoring the new command");
				return;
			}

			traj = newtraj;
			run = true;
		}
		ROS_INFO("Waking up the thread");
		cond.notify_one();
	}

	void stop() {
		boost::unique_lock<boost::mutex> lock(mut);
		run = false;
	}

	bool running() {
		boost::unique_lock<boost::mutex> lock(mut);
		return run;
	}

	bool is_waiting() {
		boost::unique_lock<boost::mutex> lock(mut);
		return waiting;
	}

private:

	/* At this time only velocity values are followed. */
	void follow_trajectory() {
		//I am not entirely sure that this code actually looks like real-time
		ros::Time trajStartTime = ros::Time::now();
		for (unsigned int step = 0; step < traj.points.size(); ++step) {
			trajectory_msgs::JointTrajectoryPoint& point = traj.points[step];

			ROS_INFO_STREAM("Trajectory thread executing step "<<step);
			//now apply speed to each joint
			for (unsigned int joint_i = 0; joint_i < traj.joint_names.size(); ++joint_i) {

				unsigned int id = nameToNumber[traj.joint_names[joint_i]];
				float velRad = point.velocities[joint_i];
				float velDeg = RAD_TO_DEG(velRad);

				ROS_INFO_STREAM("Moving module "<<id<<" with "<<velRad<<" rad/s = "<<velDeg<<" deg/s");
#if SCHUNK_NOT_AMTEC != 0
				arm->pc_move_velocity(id, velDeg);
#else
				arm->pc_move_velocity(id, velRad); // amtec protocol already in rad
#endif

				//fill in the message
				state_msg.desired = state_msg.actual;
#if SCHUNK_NOT_AMTEC != 0
				state_msg.actual.positions[joint_i] = DEG_TO_RAD(arm->mManipulator.getModules().at(joint_i).status_pos);
#else
				state_msg.actual.positions[joint_i] = arm->mManipulator.getModules().at(joint_i).status_pos;
#endif
				state_msg.actual.velocities[joint_i] = point.velocities[joint_i];
				state_msg.actual.time_from_start = point.time_from_start;
			}

			//finally publish the state message
			state_msg.header.stamp = ros::Time::now();
			state_publisher.publish(state_msg);


			//wait for the right time and check if it has to die
			while (ros::Time::now() < trajStartTime + point.time_from_start) {
				; // <- Now things are really dirty
				{ //This is tricky... a block as the only body of a loop!
					boost::unique_lock<boost::mutex> lock(mut);
					if (!run) {
						ROS_INFO("Trajectory thread has been commanded to stop");
						return;
					}
				}
			}
		}
	}

private: // by Felix
	PowerCube* arm;
	std::map<std::string, unsigned int> nameToNumber;
	trajectory_msgs::JointTrajectory traj;
	bool run;
	bool waiting;
	boost::condition_variable cond;
	boost::mutex mut;
	pr2_controllers_msgs::JointTrajectoryControllerState state_msg;
	ros::Publisher state_publisher;


	// from action tutorial
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
	std::string action_name_;
	// create messages that are used to published feedback/result
	control_msgs::FollowJointTrajectoryFeedback feedback_;
	control_msgs::FollowJointTrajectoryResult result_;

public:
	void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
	{
		bool success = true;
		ros::Time trajStartTime = ros::Time::now();
		const trajectory_msgs::JointTrajectory& traj = goal->trajectory;

		// init feedback message
		feedback_.joint_names = traj.joint_names;
		int num_of_joints = traj.joint_names.size();
		feedback_.desired.positions.resize(num_of_joints);
		feedback_.desired.velocities.resize(num_of_joints);
		feedback_.desired.accelerations.resize(num_of_joints);
		feedback_.actual.positions.resize(num_of_joints);
		feedback_.actual.velocities.resize(num_of_joints);
		feedback_.actual.accelerations.resize(num_of_joints);
		feedback_.error.positions.resize(num_of_joints);
		feedback_.error.velocities.resize(num_of_joints);
		feedback_.error.accelerations.resize(num_of_joints);

		// for each trajectory step...
		unsigned int steps = traj.points.size();
		for (unsigned int step_i = 0; step_i < steps; ++step_i) {

			// check that preempt has not been requested by the client
			if (as_.isPreemptRequested() || !ros::ok()) {
				ROS_INFO("%s: Preempted", action_name_.c_str());
				// set the action state to preempted
				as_.setPreempted();
				success = false;
				arm->pc_normal_stop();
				break;
			}


			// get the step target
			const trajectory_msgs::JointTrajectoryPoint& trajStep = traj.points[step_i];
			feedback_.desired = trajStep;

			ROS_INFO_STREAM("Trajectory thread executing step "<<step_i<<"+1/"<<steps<<" until "<<trajStep.time_from_start<<" s / "<<(trajStartTime+trajStep.time_from_start));

			// for each joint in step...
			for (unsigned int joint_i = 0; joint_i < traj.joint_names.size(); ++joint_i) {

				int id = nameToNumber[traj.joint_names[joint_i]];

				float accRad = trajStep.accelerations[joint_i];
				float velRad = trajStep.velocities[joint_i];
				float posRad = trajStep.positions[joint_i];

//				ROS_INFO_STREAM("Moving module "<<id<<" with "<</*setiosflags(ios::fixed)<<*/ velRad<<" rad/s and "<<accRad<<" rad/s*s to "<<posRad<<" rad");


/// don't go below +- def
#define SIGN_TOLERANT_MAX(value, def)	( (value)==0 ? 0 : \
											( \
												(value)>0 ? \
													std::max((value),  (def)) : \
													std::min((value), -(def)) \
											) \
										)
/// don't ...
#define SIGN_TOLERANT_CUT(value, lim)	( (value)==0 ? 0 : \
											( \
												(value)>0 ? \
													( (value) >  (lim) ? (value) : (0) ) : \
													( (value) < -(lim) ? (value) : (0) ) \
											) \
										)

//				velRad = SIGN_TOLERANT_CUT(velRad, TOO_SLOW);

//				velRad = SIGN_TOLERANT_MAX(velRad, SLOWEST_REASONABLE_JOINT_SPEED);

//				accRad = SIGN_TOLERANT_MAX(accRad, 0.43f);

				accRad = 0.21;

				if(step_i == steps-1) {
					velRad = SLOWEST_REASONABLE_JOINT_SPEED;
				}

//				if(std::abs(velRad) < TOO_SLOW) {
//					ROS_INFO_STREAM("Skipping module "<<id<<" due to too low velocity");
//				}
//				else {
				ROS_INFO_STREAM("Moving module "<<id<<" with "<</*setiosflags(ios::fixed)<<*/ velRad<<" rad/s and "<<accRad<<" rad/s*s to "<<posRad<<" rad");

#if SCHUNK_NOT_AMTEC != 0
				// TODO schunk option of trajectory action callback
//				arm->pc_move_velocity(id, velDeg);
#else
				arm->pc_set_target_acceleration(id, accRad);
				arm->pc_set_target_velocity(id, velRad);
				arm->pc_move_position(id, posRad);
//				arm->pc_move_velocity(id, velRad);	TODO choose this if the velocity and time are calculated precisely

//				if(step_i == steps-1) {
//					ROS_INFO("LAST STEP, moving to position instead of with velocity.");
//					arm->pc_move_position(id, posRad);
//				}
//				else {
////					ROS_INFO("Velocity move.");
//					arm->pc_move_velocity(id, velRad);
//				}
#endif
//				}

				// fill the joint individual feedback part
				feedback_.actual.positions[id] = arm->mManipulator.getModules().at(id).status_pos;
				feedback_.actual.velocities[id] = arm->mManipulator.getModules().at(id).status_vel;
//				feedback_.actual.accelerations[id] = arm->mManipulator.getModules().at(id).status_acc / max_acceleration?; TODO
			}

			// publish the feedback
			feedback_.header.stamp = ros::Time::now(); // TODO needed?
			feedback_.actual.time_from_start = ros::Time::now() - trajStartTime;
			as_.publishFeedback(feedback_);

			// sleep until step time has passed
			ros::Time::sleepUntil(trajStartTime + trajStep.time_from_start);
		}//for each trajectory step...

		if(success) {
			ROS_INFO("Trajectory done, waiting for joints to stop...");
			bool all_joints_stopped;
			do {
				ros::WallDuration(0.06).sleep();
				all_joints_stopped = true;
				for (unsigned int joint_i = 0; joint_i < traj.joint_names.size(); ++joint_i) {
					uint8_t moving, brake, foo;
					float foofl;
					arm->getModuleStatus(joint_i, foo, moving, foo, foo, foo, brake, foo, foo, foo, foofl);
//					if(moving) {
					if(!brake) {
						all_joints_stopped = false;
						break;
					}
//					if(arm->mManipulator.getModules().at(joint_i).status_vel == 0)
//						all_joints_stopped = false;
				}
			} while (!all_joints_stopped);

			result_.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
			ROS_INFO("%s: Succeeded", action_name_.c_str());
			// set the action state to succeeded
			as_.setSucceeded(result_);
		}
	}

};

void start_thread(TrajectoryExecuter *exec) {
	exec->main();
}

class SchunkServer : private boost::noncopyable {
private:
	PowerCube m_powerCube;
	sensor_msgs::JointState m_currentJointState;
	metralabs_ros::SchunkStatus m_schunkStatus;
	ros::NodeHandle m_node;
	std::vector<boost::shared_ptr<urdf::Joint> > m_joints;
	urdf::Model m_armModel;	// A parsing of the model description
	ros::Publisher m_currentJointStatePublisher;
	ros::Publisher m_schunkStatusPublisher;
	std::map<std::string, unsigned int> m_nameToNumber;

	TrajectoryExecuter m_executer;
	boost::thread m_executerThread;


	ros::Subscriber targetJointStateSubscriberPositionControl;
	ros::Subscriber targetJointStateSubscriberVelocityControl;

	ros::Subscriber emergency;
	ros::Subscriber stop;
	ros::Subscriber firstRef;
	ros::Subscriber ack;
	ros::Subscriber ackAll;
	ros::Subscriber ref;
	ros::Subscriber refAll;
	ros::Subscriber current;
	ros::Subscriber currentsMaxAll;
	ros::Subscriber movePosition;
	ros::Subscriber moveVelocity;
	ros::Subscriber targetVelocity;
	ros::Subscriber targetAcceleration;

	ros::Subscriber command;


public:

	SchunkServer(ros::NodeHandle &node, std::string action_server_name) :
		m_node(node),
		m_executer(node, action_server_name)
	{
		init();
	}

	void init() {
		// Initialise the arm model parser and find out what non-fixed joints are present
		m_armModel.initParam("robot_description");
		std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator mapElement;
		for (mapElement = m_armModel.joints_.begin(); mapElement != m_armModel.joints_.end(); ++mapElement) {
			boost::shared_ptr<urdf::Joint> joint = mapElement->second;
			ROS_DEBUG_STREAM("Joint: Name="<<joint->name<<" MimicPtr="<<joint->mimic);
			if (joint->type != urdf::Joint::FIXED &&
					joint->mimic == 0)	// ignore virtual joints mimicking a physical one
				m_joints.push_back(joint);
		}
		ROS_INFO("URDF specifies %d non-fixed non-mimicking joints.", m_joints.size());

		// Initialise the powercube
		m_powerCube.init();

		// Check if reinitialisation is needed
		while(m_powerCube.modulesNum != m_joints.size()) {
			ROS_WARN("Reinitialising robot arm... found %d joints but need %d", m_powerCube.modulesNum, m_joints.size());
			m_powerCube.~PowerCube();
			ros::WallDuration(1).sleep(); // let arm start up
			new (&m_powerCube) PowerCube;
			m_powerCube.init();
		}

		// Check that the required modules are present.
		ROS_WARN("Didn't check that the modules in the description match modules in reality.");

		// Set up the joint state publisher with the joint names
		// This assumes that the joints are ordered on the robot in the same order as the URDF!!!
		m_currentJointState.name.resize(m_joints.size());
		m_currentJointState.position.resize(m_joints.size());
		m_currentJointState.velocity.resize(m_joints.size());
		for (unsigned int i = 0; i < m_joints.size(); ++i) {
			m_currentJointState.name[i] = m_joints[i]->name;
			m_nameToNumber[m_joints[i]->name] = i;
			ROS_INFO("%d is mapping to %s", i, m_joints[i]->name.c_str());
		}
		m_currentJointStatePublisher = m_node.advertise<sensor_msgs::JointState>("/schunk/position/joint_states", 1);

		// Set up the schunk status publisher
		m_schunkStatusPublisher = m_node.advertise<metralabs_ros::SchunkStatus>("/schunk/status", 1);
		for (uint i = 0; i < m_joints.size(); ++i) {
			metralabs_ros::SchunkJointStatus status;
			status.jointName = m_joints[i]->name;
			m_schunkStatus.joints.push_back(status);
		}

		m_executer.init(m_node, &m_powerCube, m_nameToNumber);

		ROS_INFO("Starting the trajectory executer thread");
		m_executerThread = boost::thread(start_thread, &m_executer);


		/*
		 * /schunk/position/joint_state -> publishes joint state for kinematics modules
		 * /schunk/target_pc/joint_state <- for received position control commands
		 * /schunk/target_vc/joint_state  <- for receiving velocity control commands
		 * /schunk/status -> to publish all the statuses
		 */

		ROS_INFO("Subscribing schunk topics...");

		targetJointStateSubscriberPositionControl = m_node.subscribe("/schunk/target_pc/joint_states", 1, &SchunkServer::cb_targetJointStatePositionControl, this);
		targetJointStateSubscriberVelocityControl = m_node.subscribe("/schunk/target_vc/joint_states", 1, &SchunkServer::cb_targetJointStateVelocityControl, this);

		// those topics which must be received multiple times (for each joint) got a 10 for their message buffer
		emergency = m_node.subscribe("/emergency", 1, &SchunkServer::cb_emergency, this);
		stop = m_node.subscribe("/stop", 1, &SchunkServer::cb_stop, this);
		firstRef = m_node.subscribe("/firstRef", 1, &SchunkServer::cb_firstRef, this);
		ack = m_node.subscribe("/ack", 10, &SchunkServer::cb_ack, this);
		ackAll = m_node.subscribe("/ackAll", 1, &SchunkServer::cb_ackAll, this);
		ref = m_node.subscribe("/ref", 10, &SchunkServer::cb_ref, this);
		refAll = m_node.subscribe("/refAll", 1, &SchunkServer::cb_refAll, this);
		current = m_node.subscribe("/current", 10, &SchunkServer::cb_current, this);
		currentsMaxAll = m_node.subscribe("/currentsMaxAll", 1, &SchunkServer::cb_currentsMaxAll, this);
		movePosition = m_node.subscribe("/movePosition", 10, &SchunkServer::cb_movePosition, this);
		moveVelocity = m_node.subscribe("/moveVelocity", 10, &SchunkServer::cb_moveVelocity, this);
		targetVelocity = m_node.subscribe("/targetVelocity", 10, &SchunkServer::cb_targetVelocity, this);
		targetAcceleration = m_node.subscribe("/targetAcceleration", 10, &SchunkServer::cb_targetAcceleration, this);

		command = m_node.subscribe("command", 1, &SchunkServer::cb_commandTrajectory, this);


		ROS_INFO("SchunkServer Ready");

	}

	~SchunkServer() {
		m_executer.stop();
		m_executerThread.interrupt();
		m_executerThread.join();
	}

	void cb_emergency(const std_msgs::Bool::ConstPtr& dummy) 	{
		ROS_INFO("emergency");
		m_powerCube.pc_emergency_stop();
	}

	void cb_stop(const std_msgs::Bool::ConstPtr& dummy) 	{
		ROS_INFO("stop");
		m_powerCube.pc_normal_stop();
	}

	void cb_firstRef(const std_msgs::Bool::ConstPtr& dummy) 	{
		ROS_INFO("first ref");
		m_powerCube.pc_first_ref();
	}

	void cb_ack(const std_msgs::Int8::ConstPtr& id) 	{
		ROS_INFO("cb_ack: [%d]", id->data);
		m_powerCube.pc_ack(id->data);
	}

	void cb_ackAll(const std_msgs::Bool::ConstPtr& dummy) 	{
		ROS_INFO("cb_ackall");
		m_powerCube.pc_ack();
	}

	void cb_ref(const std_msgs::Int8::ConstPtr& id)	{
		ROS_INFO("cb_ref: [%d]", id->data);
		m_powerCube.pc_ref(id->data);
	}

	void cb_refAll(const std_msgs::Bool::ConstPtr& dummy)	{
		ROS_INFO("cb_refall");
		m_powerCube.pc_ref();
	}

	void cb_current(const metralabs_ros::idAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_current: [%d, %f]", data->id, data->value);
		m_powerCube.pc_set_current(data->id, data->value);
	}

	void cb_currentsMaxAll(const std_msgs::Bool::ConstPtr& i)	{
		ROS_INFO("cb_currentsMax");
		m_powerCube.pc_set_currents_max();
	}

	void cb_movePosition(const metralabs_ros::idAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_movePosition: [%d, %f]", data->id, data->value);
		m_powerCube.pc_move_position(data->id, data->value);
	}

	void cb_moveVelocity(const metralabs_ros::idAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_moveVelocity: [%d, %f]", data->id, data->value);
		if (data->value == 0.0)
			m_powerCube.pc_normal_stop(data->id);
		else
			m_powerCube.pc_move_velocity(data->id, data->value);
	}

	void cb_targetVelocity(const metralabs_ros::idAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_targetVelocity: [%d, %f]", data->id, data->value);
		m_powerCube.pc_set_target_velocity(data->id, data->value);
	}

	void cb_targetAcceleration(const metralabs_ros::idAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_targetAcceleration: [%d, %f]", data->id, data->value);
		m_powerCube.pc_set_target_acceleration(data->id, data->value);
	}


	void publishCurrentJointState () {
		m_currentJointState.header.stamp = ros::Time::now();
		for (unsigned int i = 0; i < m_currentJointState.name.size(); ++i) {
#if SCHUNK_NOT_AMTEC != 0
			SCHUNKMotionManipulator::ModuleConfig modCfg = m_powerCube.mManipulator.getModules().at(i);
			m_currentJointState.position[i]=DEG_TO_RAD(modCfg.status_pos);
			m_currentJointState.velocity[i]=DEG_TO_RAD(modCfg.status_vel);
#else
			AmtecManipulator::ModuleConfig modCfg = m_powerCube.mManipulator.getModules().at(i);
			m_currentJointState.position[i]=modCfg.status_pos; // amtec protocol already in rad
			m_currentJointState.velocity[i]=modCfg.status_vel;
#endif
		}
		m_currentJointStatePublisher.publish(m_currentJointState);
	}

	void publishSchunkStatus() {
		std::vector<metralabs_ros::SchunkJointStatus>::iterator it;
		uint moduleNumber;
		for (it = m_schunkStatus.joints.begin(); it != m_schunkStatus.joints.end(); ++it) {
			moduleNumber = m_nameToNumber[it->jointName];
			m_powerCube.getModuleStatus(moduleNumber, it->referenced, it->moving, it->progMode, it->warning,
					it->error, it->brake, it->moveEnd, it->posReached, it->errorCode, it->current);
		}
		m_schunkStatusPublisher.publish(m_schunkStatus);
	}

	void cb_targetJointStatePositionControl(const sensor_msgs::JointState::ConstPtr& data) {
		// The "names" member says how many joints, the other members may be empty.
		// Not all the joints have to be specified in the message
		// and not all types must be filled
#if SCHUNK_NOT_AMTEC != 0
		float rad_to_degrees_needed = RAD_TO_DEG(1);
#else
		float rad_to_degrees_needed = 1;
#endif

		for (uint i = 0; i < data.get()->name.size(); ++i) {
			string joint_name = data.get()->name[i];
			int joint_number = m_nameToNumber[joint_name];
			ROS_INFO_STREAM("cb_PositionControl: joint "<<joint_name<<" (#"<<joint_number<<")");

			m_powerCube.pc_set_currents_max();
			if (data.get()->position.size()!=0) {
				double pos = data.get()->position[i] / rad_to_degrees_needed;
				ROS_INFO_STREAM(" to position "<<pos);
				m_powerCube.pc_move_position(joint_number, pos);
			}
			if (data.get()->velocity.size()!=0) {
				double vel = data.get()->velocity[i] / rad_to_degrees_needed;
				ROS_INFO_STREAM(" with velocity "<<vel);
				m_powerCube.pc_set_target_velocity(joint_number, vel);
			}
			if (data.get()->effort.size()!=0) {
				double eff = data.get()->effort[i];
				ROS_INFO_STREAM(" with effort "<<eff);
				m_powerCube.pc_set_current(joint_number, eff);
			}

		}
	}

	void cb_targetJointStateVelocityControl(const sensor_msgs::JointState::ConstPtr& data) {
#if SCHUNK_NOT_AMTEC != 0
		float rad_to_degrees_needed = RAD_TO_DEG(1);
#else
		float rad_to_degrees_needed = 1;
#endif

		// The "names" member says how many joints, the other members may be empty.
		// Not all the joints have to be specified in the message
		// and not all types must be filled
//		m_powerCube.pc_ack();
		for (uint i = 0; i < data.get()->name.size(); ++i) {
			string joint_name = data.get()->name[i];
			int joint_number = m_nameToNumber[joint_name];
			ROS_INFO_STREAM("cb_VelocityControl: joint "<<joint_name<<" (#"<<joint_number<<")");

//			m_powerCube.pc_set_currents_max();

			if (data.get()->velocity.size() != 0) {
				double vel = data.get()->velocity[i] / rad_to_degrees_needed;
				ROS_INFO_STREAM(" with velocity "<<vel);
				if (vel == 0.0)
					m_powerCube.pc_normal_stop(joint_number);
				else
					m_powerCube.pc_move_velocity(joint_number, vel);
			}
			if (data.get()->effort.size()!=0) {
				double eff = data.get()->effort[i];
				ROS_INFO_STREAM(" with effort "<<eff);
				m_powerCube.pc_set_current(joint_number, eff);
			}

		}
	}

	void cb_commandTrajectory(const trajectory_msgs::JointTrajectory traj) {
		ROS_INFO("Schunk Server: received a new trajectory");
		m_executer.stop();
		while (! m_executer.is_waiting() ) {
			ROS_INFO("Waiting for the controller to be ready..");
		}
		m_executer.start(traj);
	}

};



class RosScitosBase : private boost::noncopyable {

public:
	RosScitosBase(ros::NodeHandle& n, ScitosBase* base) :
		m_node(n),
		remote_control_next_timeout(),
		DEAD_REMOTE_CONTROL_TIMEOUT(0.9),
		activate_remote_control_timeout(true)
	{
		m_base = base;
		m_odomPublisher = m_node.advertise<nav_msgs::Odometry> ("odom", 50);
		m_sonarPublisher = m_node.advertise<sensor_msgs::Range> ("sonar", 50);
		m_commandSubscriber = m_node.subscribe("cmd_vel", 1, &RosScitosBase::driveCommandCallback, this);
		m_bumperResetSubscriber = m_node.subscribe("bumper_reset", 1, &RosScitosBase::bumperResetCallback, this);
	}
	
	void loop() {
		// dead remote control check
		if(activate_remote_control_timeout && ros::Time::now() > remote_control_next_timeout) {
			ROS_INFO("remote control timeout after nonzero command, stopping robot");
			m_base->set_velocity(0, 0);
			activate_remote_control_timeout = false;
		}


		// The odometry position and velocities of the robot
		double x, y, th, vx, vth;
		m_base->get_odometry(x,y,th,vx,vth);

		ros::Time currentTime = ros::Time::now();
		// since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);


		/// odometry tf

		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = currentTime;
		odom_trans.header.frame_id = "/odom";
		odom_trans.child_frame_id = "/base_link";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		m_odom_broadcaster.sendTransform(odom_trans);


		// send velocity data again loop
		m_base->loop();


		/// odometry data

		nav_msgs::Odometry odom_msg;
		odom_msg.header.stamp = currentTime;
		odom_msg.header.frame_id = "/odom";
		odom_msg.child_frame_id = "/base_link";

		//set the position
		odom_msg.pose.pose.position.x = x;
		odom_msg.pose.pose.position.y = y;
		odom_msg.pose.pose.position.z = 0.0;
		odom_msg.pose.pose.orientation = odom_quat;

		//set the velocity
		odom_msg.twist.twist.linear.x = vx;
		odom_msg.twist.twist.linear.y = 0;
		odom_msg.twist.twist.angular.z = vth;

		//publish the message
		m_odomPublisher.publish(odom_msg);


		/// enable or disable sonar if someone or no one is listening
		static bool currentSonarStateIsActive = true;
		if(currentSonarStateIsActive && m_sonarPublisher.getNumSubscribers() == 0) {
			m_base->setFeature(FEATURE_SONAR, false);
			currentSonarStateIsActive = false;
		} else if(!currentSonarStateIsActive && m_sonarPublisher.getNumSubscribers() != 0) {
			m_base->setFeature(FEATURE_SONAR, true);
			currentSonarStateIsActive = true;
		}

		if(currentSonarStateIsActive) {
			static const RangeData::Config* sonarConfig = 0;

			// load config once
			if (sonarConfig == 0) {
				m_base->get_sonar_config(sonarConfig);		// TODO what if nonzero rubbish is read?
//				std::cout << "sonarConfig was 0 and now we read: " << sonarConfig << std::endl;
			}
			// if config is loaded, proceed..
			if (sonarConfig != 0) {
				const RangeData::ConfigCircularArc* sonarConfigCircular =
						dynamic_cast<const RangeData::ConfigCircularArc*>(sonarConfig);

				/// sonar tf

				// calculate transforms (once)
				static std::vector<tf::Transform> sonarTransforms;
				static bool sonarTransformsCalculated = false;

				if(!sonarTransformsCalculated) {
					sonarTransformsCalculated = true;
					float angle = sonarConfigCircular->first_sensor_angle;
					for (unsigned int i = 0; i < sonarConfigCircular->sensor_cnt; ++i) {
						float x = sonarConfigCircular->offset_from_center * std::cos(angle) -0.075;
						float y = sonarConfigCircular->offset_from_center * std::sin(angle);
						tf::Transform* transform = new tf::Transform(
								tf::Quaternion(0, 0, angle),
								tf::Vector3(x, y, 0.25));
						sonarTransforms.push_back(*transform);
						angle += sonarConfigCircular->sensor_angle_dist;
					}
					// broadcast all transforms once
					std::vector<tf::Transform>::iterator it = sonarTransforms.begin();
					for (int i = 0; it != sonarTransforms.end(); ++it) {
						char targetframe[20];
						sprintf(targetframe, "/sonar/sonar_%02d", i++);
						m_odom_broadcaster.sendTransform(
								tf::StampedTransform(*it, ros::Time::now(), "/base_link", targetframe)
							);
					}
				}



				/// sonar data

				std::vector<RangeData::Measurement> measurements;
				m_base->get_sonar(measurements);

				sensor_msgs::Range sonar_msg;
				sonar_msg.header.stamp = currentTime;
				sonar_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
				sonar_msg.field_of_view = sonarConfigCircular->cone_angle;  // DEG_TO_RAD(15);	// from manual
				sonar_msg.min_range = 0.2;	// from manual
				sonar_msg.max_range = 3;	// from manual

				// each time send only one sonar data and one transform to reduce repeating messages
				{
					static int nextSonarToSend = 0;
					char nextTargetframe[20];
					sprintf(nextTargetframe, "/sonar/sonar_%02d", nextSonarToSend);

					// range data
//					ROS_DEBUG("Sonar #%d: value: %1.3f status: %d", nextSonarToSend, measurements.at(nextSonarToSend).range, measurements.at(nextSonarToSend).err.std);
					switch(measurements.at(nextSonarToSend).err.std) {
						case RangeData::RANGE_OKAY:
						case RangeData::RANGE_OBJECT_SOMEWHERE_IN_RANGE:
						case RangeData::RANGE_PROBABLY_OKAY:
						case RangeData::RANGE_PROBABLY_OBJECT_SOMEWHERE_IN_RANGE:
							sonar_msg.range = measurements.at(nextSonarToSend).range;
							break;
						case RangeData::RANGE_NO_OBJECT_WITHIN_RANGE:
						case RangeData::RANGE_PROBABLY_NO_OBJECT_WITHIN_RANGE:
						case RangeData::RANGE_INVALID_MEASUREMENT:
						case RangeData::RANGE_ERR_SENSOR_BROKEN:
						case RangeData::RANGE_MASKED:
						default:
							;// TODO maybe send a zero-range message to make the old value obsolete, if that isn't useful
							sonar_msg.range = 0;
					}
					sonar_msg.header.frame_id = nextTargetframe;
					m_sonarPublisher.publish(sonar_msg);

					// resend also one transform (the according, why not)
					m_odom_broadcaster.sendTransform( tf::StampedTransform(
							sonarTransforms.at(nextSonarToSend), ros::Time::now(), "/base_link", nextTargetframe ) );

					++nextSonarToSend %= measurements.size();
				}


	//		this is code to send all sonar measurements at one time
	//			std::vector<RangeData::Measurement>::iterator itM = measurements.begin();
	//			for(int i=0; itM != measurements.end(); ++itM) {
	////				std::cout<<itM->range<<" ";
	//
	//				char targetframe[20];
	//				sprintf(targetframe, "/sonar/sonar_%2d", i++);
	//				sonar.header.frame_id = targetframe;
	//				sonar.range = itM->range+1;
	//
	//				//publish the message
	//				m_sonarPublisher.publish(sonar);
	//			}

			} // if sonar config loaded
		} // if sonar active

	}

//	private string objectToString(object o) {
//		std::stringstream ss;
//	}
	void loop_diagnostics(ros::Publisher &diagnosticsPublisher) {
		float pVoltage;
		float pCurrent;
		int16_t pChargeState;
		int16_t pRemainingTime;
		int16_t pChargerStatus;
		m_base->get_batteryState(pVoltage, pCurrent, pChargeState, pRemainingTime, pChargerStatus);

		diagnostic_msgs::DiagnosticStatus batteryStatus;
		batteryStatus.level = diagnostic_msgs::DiagnosticStatus::OK;
		batteryStatus.name = "Battery";
		batteryStatus.message = "undefined";
		batteryStatus.hardware_id = "0a4fcec0-27ef-497a-93ba-db39808ec1af";

// TODO do me parameters
#define 	VOLTAGE_ERROR_LEVEL	23		// and below
#define 	VOLTAGE_WARN_LEVEL	24		// and below
#define 	VOLTAGE_MID_LEVEL	26		// and below // above means HIGH_LEVEL
#define 	VOLTAGE_FULL_LEVEL	28.8	// and above
#define 	CHARGER_PLUGGED 	1

		if(pVoltage < VOLTAGE_ERROR_LEVEL && pChargerStatus != CHARGER_PLUGGED)
			batteryStatus.level = diagnostic_msgs::DiagnosticStatus::ERROR;
		else if(pVoltage < VOLTAGE_WARN_LEVEL && pChargerStatus != CHARGER_PLUGGED)
			batteryStatus.level = diagnostic_msgs::DiagnosticStatus::WARN;

		// build text message
		batteryStatus.message = "High";
		if(pVoltage < VOLTAGE_MID_LEVEL)
			batteryStatus.message = "Mid";
		if(pVoltage < VOLTAGE_WARN_LEVEL)
			batteryStatus.message = "Low";
		if(pVoltage < VOLTAGE_ERROR_LEVEL)
			batteryStatus.message = "Depleted";

		batteryStatus.message += pChargerStatus == CHARGER_PLUGGED ? ", charging" : ", discharging";

		if(pVoltage >= VOLTAGE_FULL_LEVEL)
			batteryStatus.message = "Fully charged";


		batteryStatus.values.resize(5);
		std::stringstream ss;
		batteryStatus.values[0].key = "Voltage";
		ss << pVoltage << " V";
		batteryStatus.values[0].value = ss.str();

		ss.str("");
		batteryStatus.values[1].key = "Current";
		ss << pCurrent << " A";
		batteryStatus.values[1].value = ss.str();

		ss.str("");
		batteryStatus.values[2].key = "ChargeState";
		ss << pChargeState << " %";
		batteryStatus.values[2].value = ss.str();

		ss.str("");
		batteryStatus.values[3].key = "RemainingTime";
		ss << pRemainingTime << " min";
		batteryStatus.values[3].value = ss.str();

		batteryStatus.values[4].key = "ChargerStatus";
		batteryStatus.values[4].value = pChargerStatus == CHARGER_PLUGGED ? "plugged" : "unplugged";

		/// combine and publish statii as array
		diagnostic_msgs::DiagnosticArray diagArray;
		diagArray.status.push_back(batteryStatus);
		diagnosticsPublisher.publish(diagArray);

	}

	void dynamic_reconfigure_callback(metralabs_ros::ScitosG5Config &config, uint32_t level) {
		// I wrote this macro because I couldn't find a way to read the configs parameters generically,
		// and with this macro the actual feature name only has to be named once. Improvements welcome.
		#define MAKRO_SET_FEATURE(NAME)	\
			ROS_INFO("Setting feature %s to %s", #NAME, config.NAME?"True":"False"); \
			m_base->setFeature(#NAME, config.NAME)

		MAKRO_SET_FEATURE(EBC0_Enable5V);
		MAKRO_SET_FEATURE(EBC0_Enable12V);
		MAKRO_SET_FEATURE(EBC0_Enable24V);
		MAKRO_SET_FEATURE(EBC1_Enable5V);
		MAKRO_SET_FEATURE(EBC1_Enable12V);
		MAKRO_SET_FEATURE(EBC1_Enable24V);
		MAKRO_SET_FEATURE(FreeRunMode);
		MAKRO_SET_FEATURE(SonarsActive);
		MAKRO_SET_FEATURE(StatusDisplayKnobLock);
		MAKRO_SET_FEATURE(StatusDisplayLED);

		ROS_DEBUG("Now reading again: (why is this rubbish?)");	// TODO fix me
		metralabs_ros::ScitosG5Config config_read;
		get_features(config_read);
	}

	void get_features(metralabs_ros::ScitosG5Config &config) {
		// I wrote this macro because I couldn't find a way to read the configs parameters generically,
		// and with this macro the actual feature name only has to be named once. Improvements welcome.
		#define MAKRO_GET_FEATURE(NAME)	\
			ROS_DEBUG("Current hardware feature %s is %s", #NAME, config.NAME?"True":"False"); \
			config.NAME = m_base->getFeature<typeof(config.NAME)>(std::string(#NAME))

		MAKRO_GET_FEATURE(EBC0_Enable5V);
		MAKRO_GET_FEATURE(EBC0_Enable12V);
		MAKRO_GET_FEATURE(EBC0_Enable24V);
		MAKRO_GET_FEATURE(EBC1_Enable5V);
		MAKRO_GET_FEATURE(EBC1_Enable12V);
		MAKRO_GET_FEATURE(EBC1_Enable24V);
		MAKRO_GET_FEATURE(FreeRunMode);
		MAKRO_GET_FEATURE(SonarsActive);
		MAKRO_GET_FEATURE(StatusDisplayKnobLock);
		MAKRO_GET_FEATURE(StatusDisplayLED);
	}

private:
	ros::NodeHandle m_node;
	ScitosBase* m_base;
	tf::TransformBroadcaster m_odom_broadcaster;
	ros::Publisher m_odomPublisher;
	ros::Publisher m_sonarPublisher;
	ros::Subscriber m_commandSubscriber;
	ros::Subscriber m_bumperResetSubscriber;

	ros::Time remote_control_next_timeout;
	ros::Duration DEAD_REMOTE_CONTROL_TIMEOUT;
	bool activate_remote_control_timeout;

private:
	void driveCommandCallback(const geometry_msgs::TwistConstPtr& msg) {
		m_base->set_velocity(msg->linear.x, msg->angular.z);
		ROS_DEBUG("Received some speeds [%f %f]", msg->linear.x, msg->angular.z);
		activate_remote_control_timeout = msg->linear.x != 0 || msg->angular.z != 0;
		remote_control_next_timeout = ros::Time::now() + DEAD_REMOTE_CONTROL_TIMEOUT;
	}

	void bumperResetCallback(const std_msgs::EmptyConstPtr& dummy) {
		ROS_INFO("Resetting bumper");
		m_base->reset_bumper();
	}
};


void diagnosticsPublishingLoop(ros::NodeHandle& n, RosScitosBase& ros_scitos,
		ros::Publisher &diagnosticsPublisher, ros::Rate loop_rate)
{
	while (n.ok()) {
		ros_scitos.loop_diagnostics(diagnosticsPublisher);
		// This will adjust as needed per iteration
		loop_rate.sleep();
	}
}

void dynamicReconfigureUpdaterLoop(ros::NodeHandle& n, RosScitosBase &ros_scitos,
		dynamic_reconfigure::Server<metralabs_ros::ScitosG5Config> &dynamicReconfigureServer,
		boost::recursive_mutex &mutex, ros::Rate loop_rate)
{

	metralabs_ros::ScitosG5Config config;

	while (n.ok()) {
		ros_scitos.get_features(config);// update config to current hardware state
		boost::recursive_mutex::scoped_lock lock(mutex);
		dynamicReconfigureServer.updateConfig(config);
		lock.unlock();
		loop_rate.sleep();
	}

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "metralabs_ros");
	ros::NodeHandle n;
	ros::NodeHandle n_private("~");

	std::string action_server_name = "schunk/follow_joint_trajectory";


	/// read parameters

	bool disable_arm;
	n_private.param("disable_arm", disable_arm, false);

	string scitos_config_file;
	n_private.param<string>("scitos_config_file", scitos_config_file,
			"/opt/MetraLabs/MLRobotic/etc/config/SCITOS-G5_without_Head_config.xml");

	ros::Duration(0.9).sleep(); // wait to let the running me close its scitos connection


	/// initialize robot base & node components

	ROS_INFO("Starting robot base...");
	ScitosBase base(scitos_config_file.c_str(), argc, argv);

	base.setFeature(FEATURE_SONAR, false);

	if(!disable_arm) {
		ros::Duration(0.5).sleep(); // let ScitosBase connect to robot
		base.setFeature(FEATURE_ARM, true);
	}

	ROS_INFO("Starting ros base connector...");
	RosScitosBase ros_scitos(n, &base);


	/// intialize diagnostics

	ROS_INFO("Starting diagnostics...");

	ros::Publisher diagnosticsPublisher = n.advertise<diagnostic_msgs::DiagnosticArray> ("/diagnostics", 50);
	boost::thread(diagnosticsPublishingLoop, n, boost::ref(ros_scitos), boost::ref(diagnosticsPublisher), ros::Rate(2));


	/// intialize dynamic_reconfigure

	ROS_INFO("Starting dynamic_reconfigure...");

	boost::recursive_mutex dyn_reconf_mutex;
	dynamic_reconfigure::Server<metralabs_ros::ScitosG5Config> dynamicReconfigureServer(dyn_reconf_mutex);

	// update config to current hardware state
	metralabs_ros::ScitosG5Config config;
	ros_scitos.get_features(config);	// The first time the read config is totally wrong, maybe it's the previous state from rosparam
//	ROS_INFO_STREAM("This is what the first read gives... " << config.EBC1_Enable24V);
//	ros_scitos.get_features(config);	// It's no timing issue as far as I tested it. 		TODO fix or proof as stable..
	if(!disable_arm) {
		config.EBC1_Enable24V = true;
	}
	// mutex needed for dynreconf.updateConfig, see non existent manual; eh I mean source
	boost::recursive_mutex::scoped_lock lock(dyn_reconf_mutex);

//	ROS_INFO_STREAM("Updating with current config from hardware... " << config.EBC1_Enable24V);
	dynamicReconfigureServer.updateConfig(config);
	lock.unlock();

	// init reconfigure publisher
	boost::thread(dynamicReconfigureUpdaterLoop, n, boost::ref(ros_scitos), boost::ref(dynamicReconfigureServer),
			boost::ref(dyn_reconf_mutex), ros::Rate(2));

	// init reconfigure callback
	dynamic_reconfigure::Server<metralabs_ros::ScitosG5Config>::CallbackType f;
	f = boost::bind(&RosScitosBase::dynamic_reconfigure_callback, boost::ref(ros_scitos), _1, _2);
	dynamicReconfigureServer.setCallback(f);


	/// intialize robot arm

	ROS_INFO("Starting ros schunk connector...");
	SchunkServer schunkServer(n, action_server_name);


	/// start main loop

	ROS_INFO("Initializing done, starting loop");
	ros::Rate loop_rate(30);
	while (n.ok()) {
		// TODO: Consider replacing spinOnce with AsyncSpinner to uncouple callbacks. But check thread-safety before!
		ros::spinOnce();

		schunkServer.publishCurrentJointState();
		schunkServer.publishSchunkStatus();

		ros_scitos.loop();

		// This will adjust as needed per iteration
		if(!loop_rate.sleep())
			ROS_WARN("ScitosServer loop missed its desired interval of %.3f seconds: the loop actually took %.3f.",
				loop_rate.expectedCycleTime().toSec(), loop_rate.cycleTime().toSec());
	}

	base.setFeature(FEATURE_ARM, false);
	base.setFeature(FEATURE_SONAR, false);

	return 0;
}

