
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
#include <metralabs_ros/ScitosG5Bumper.h>

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
		as_(nh_, action_server_name, boost::bind(&TrajectoryExecuter::trajectoryActionCallback, this, _1), false),
		action_name_(action_server_name)
	{
		run_ = false;
		waiting_ = false;
		arm_ = NULL;
	}

	void init(ros::NodeHandle& n, PowerCube* p_arm, std::map<std::string, unsigned int>& nmap) {

		arm_ = p_arm;
		joints_name_to_number_map_ = nmap;

		state_publisher_ = n.advertise<pr2_controllers_msgs::JointTrajectoryControllerState>("state", 1);

		std::map<std::string, unsigned int>::iterator it;
		for (it = joints_name_to_number_map_.begin(); it != joints_name_to_number_map_.end(); ++it) {
			state_msg_.joint_names.push_back(it->first);
		}

		int num_of_joints = joints_name_to_number_map_.size();
		state_msg_.desired.positions.resize(num_of_joints);
		state_msg_.desired.velocities.resize(num_of_joints);
		state_msg_.desired.accelerations.resize(num_of_joints);
		state_msg_.actual.positions.resize(num_of_joints);
		state_msg_.actual.velocities.resize(num_of_joints);
		state_msg_.error.positions.resize(num_of_joints);
		state_msg_.error.velocities.resize(num_of_joints);

		as_.start();
	}

	void main() {
		while (true) {
			{
				boost::unique_lock<boost::mutex> lock(mut_);
				waiting_ = true;
				bool gotit = false;
				while (!run_){
					if(!nh_.ok()) {
						return;
					}
					try {
						boost::this_thread::interruption_point();
						gotit = cond_.timed_wait(lock, boost::posix_time::milliseconds(100));
					}
					catch(const boost::thread_interrupted&) {
						ROS_INFO("Trajectory thread was interrupted and returns, stopping the arm.");
						arm_->normalStopAll();
						return;
					}
					if (! gotit) {
						//Let's make people happy by publishing a state message
						for (unsigned int joint_i = 0; joint_i<joints_name_to_number_map_.size(); ++joint_i) {
							//fill in the message
							state_msg_.desired = state_msg_.actual;
							state_msg_.actual.positions[joint_i] = RAD_TO_DEG(arm_->getManipulator().getModules().at(joint_i).status_pos);
							state_msg_.actual.velocities[joint_i] = 0;
							state_msg_.actual.time_from_start = ros::Duration(0);
						}

						//finally publish the state message
						state_msg_.header.stamp = ros::Time::now();
						state_publisher_.publish(state_msg_);
					}

				}
			}
			//I hope the mutex is unlocked now
			{
				boost::unique_lock<boost::mutex> lock(mut_);
				waiting_ = false;
			}

			ROS_INFO("Trajectory thread is starting its job");
			followTrajectory();
			arm_->normalStopAll();

			{
				//Self deactivate
				boost::unique_lock<boost::mutex> lock(mut_);
				run_ = false;
			}
			ROS_INFO("Trajectory thread has finished its job");
		}
		return; //just to be sure the thread is terminated
	}

	void start(const trajectory_msgs::JointTrajectory& newtraj) {
		{
			boost::unique_lock<boost::mutex> lock(mut_);
			if (run_) {
				ROS_WARN("You need to stop a trajectory before issuing a new command. Ignoring the new command");
				return;
			}

			traj_ = newtraj;
			run_ = true;
		}
		ROS_INFO("Waking up the thread");
		cond_.notify_one();
	}

	void stop() {
		boost::unique_lock<boost::mutex> lock(mut_);
		run_ = false;
	}

	bool isRunning() {
		boost::unique_lock<boost::mutex> lock(mut_);
		return run_;
	}

	bool isWaiting() {
		boost::unique_lock<boost::mutex> lock(mut_);
		return waiting_;
	}


	void trajectoryActionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
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
				arm_->normalStopAll();
				break;
			}


			// get the step target
			const trajectory_msgs::JointTrajectoryPoint& trajStep = traj.points[step_i];
			feedback_.desired = trajStep;

			ROS_INFO_STREAM("Trajectory thread executing step "<<step_i<<"+1/"<<steps<<" until "<<trajStep.time_from_start<<" s / "<<(trajStartTime+trajStep.time_from_start));

			// for each joint in step...
			for (unsigned int joint_i = 0; joint_i < traj.joint_names.size(); ++joint_i) {

				int id = joints_name_to_number_map_[traj.joint_names[joint_i]];

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
//				arm_->moveVelocity(id, velDeg);
#else
				arm_->setTargetAcceleration(id, accRad);
				arm_->setTargetVelocity(id, velRad);
				arm_->movePosition(id, posRad);
//				arm_->moveVelocity(id, velRad);	TODO choose this if the velocity and time are calculated precisely

//				if(step_i == steps-1) {
//					ROS_INFO("LAST STEP, moving to position instead of with velocity.");
//					arm_->movePosition(id, posRad);
//				}
//				else {
////					ROS_INFO("Velocity move.");
//					arm_->moveVelocity(id, velRad);
//				}
#endif
//				}

				// fill the joint individual feedback part
				feedback_.actual.positions[id] = arm_->getManipulator().getModules().at(id).status_pos;
				feedback_.actual.velocities[id] = arm_->getManipulator().getModules().at(id).status_vel;
//				feedback_.actual.accelerations[id] = arm_->manipulator_.getModules().at(id).status_acc / max_acceleration?; TODO cannot know acceleration status
			}

			// publish the feedback
			feedback_.header.stamp = ros::Time::now();
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
					arm_->getModuleStatus(joint_i, foo, moving, foo, foo, foo, brake, foo, foo, foo, foofl);
//					if(moving) {
					if(!brake) {
						all_joints_stopped = false;
						break;
					}
//					if(arm_->manipulator_.getModules().at(joint_i).status_vel == 0)
//						all_joints_stopped = false;
				}
			} while (!all_joints_stopped);

			result_.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
			ROS_INFO("%s: Succeeded", action_name_.c_str());
			// set the action state to succeeded
			as_.setSucceeded(result_);
		}
	}


	// from action tutorial
protected:
	ros::NodeHandle nh_;
	actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
	std::string action_name_;
	// create messages that are used to published feedback/result
	control_msgs::FollowJointTrajectoryFeedback feedback_;
	control_msgs::FollowJointTrajectoryResult result_;


private:
	/* At this time only velocity values are followed. */
	void followTrajectory() {
		//I am not entirely sure that this code actually looks like real-time
		ros::Time trajStartTime = ros::Time::now();
		for (unsigned int step = 0; step < traj_.points.size(); ++step) {
			trajectory_msgs::JointTrajectoryPoint& point = traj_.points[step];

			ROS_INFO_STREAM("Trajectory thread executing step "<<step);
			//now apply speed to each joint
			for (unsigned int joint_i = 0; joint_i < traj_.joint_names.size(); ++joint_i) {

				unsigned int id = joints_name_to_number_map_[traj_.joint_names[joint_i]];
				float velRad = point.velocities[joint_i];
				float velDeg = RAD_TO_DEG(velRad);

				ROS_INFO_STREAM("Moving module "<<id<<" with "<<velRad<<" rad/s = "<<velDeg<<" deg/s");
#if SCHUNK_NOT_AMTEC != 0
				arm_->moveVelocity(id, velDeg);
#else
				arm_->moveVelocity(id, velRad); // amtec protocol already in rad
#endif

				//fill in the message
				state_msg_.desired = state_msg_.actual;
#if SCHUNK_NOT_AMTEC != 0
				state_msg_.actual.positions[joint_i] = DEG_TO_RAD(arm_->manipulator_.getModules().at(joint_i).status_pos);
#else
				state_msg_.actual.positions[joint_i] = arm_->getManipulator().getModules().at(joint_i).status_pos;
#endif
				state_msg_.actual.velocities[joint_i] = point.velocities[joint_i];
				state_msg_.actual.time_from_start = point.time_from_start;
			}

			//finally publish the state message
			state_msg_.header.stamp = ros::Time::now();
			state_publisher_.publish(state_msg_);


			//wait for the right time and check if it has to die
			while (ros::Time::now() < trajStartTime + point.time_from_start) {
				; // <- Now things are really dirty
				{ //This is tricky... a block as the only body of a loop!
					boost::unique_lock<boost::mutex> lock(mut_);
					if (!run_) {
						ROS_INFO("Trajectory thread has been commanded to stop");
						return;
					}
				}
			}
		}
	}

	PowerCube* arm_;
	std::map<std::string, unsigned int> joints_name_to_number_map_;
	trajectory_msgs::JointTrajectory traj_;
	bool run_;
	bool waiting_;
	boost::condition_variable cond_;
	boost::mutex mut_;
	pr2_controllers_msgs::JointTrajectoryControllerState state_msg_;
	ros::Publisher state_publisher_;
};


class SchunkServer : private boost::noncopyable {
public:
	SchunkServer(ros::NodeHandle &node, std::string action_server_name) :
		node_handle_(node),
		trajectory_executer_(node, action_server_name)
	{
		init();
	}

	void init() {
		// Initialise the arm model parser and find out what non-fixed joints are present
		arm_model_.initParam("robot_description");
		std::map<std::string, boost::shared_ptr<urdf::Joint> >::const_iterator mapElement;
		for (mapElement = arm_model_.joints_.begin(); mapElement != arm_model_.joints_.end(); ++mapElement) {
			boost::shared_ptr<urdf::Joint> joint = mapElement->second;
			ROS_DEBUG_STREAM("Joint: Name="<<joint->name<<" MimicPtr="<<joint->mimic);
			if (joint->type != urdf::Joint::FIXED &&
					joint->mimic == 0)	// ignore virtual joints mimicking a physical one
				joints_list_.push_back(joint);
		}
		ROS_INFO("URDF specifies %d non-fixed non-mimicking joints.", joints_list_.size());

		// Initialise the powercube
		power_cube_.init();

		// Check if reinitialisation is needed
		while(power_cube_.getModulesCount() != joints_list_.size()) {
			ROS_WARN("Reinitialising robot arm... found %d joints but need %d", power_cube_.getModulesCount(), joints_list_.size());
			power_cube_.~PowerCube();
			ros::WallDuration(1).sleep(); // let arm start up
			new (&power_cube_) PowerCube;
			power_cube_.init();
		}

		// Check that the required modules are present.
		ROS_WARN("Didn't check that the modules in the description match modules in reality.");

		// Set up the joint state publisher with the joint names
		// This assumes that the joints are ordered on the robot in the same order as the URDF!!!
		current_JointState_.name.resize(joints_list_.size());
		current_JointState_.position.resize(joints_list_.size());
		current_JointState_.velocity.resize(joints_list_.size());
		for (unsigned int i = 0; i < joints_list_.size(); ++i) {
			current_JointState_.name[i] = joints_list_[i]->name;
			joints_name_to_number_map_[joints_list_[i]->name] = i;
			ROS_INFO("%d is mapping to %s", i, joints_list_[i]->name.c_str());
		}
		current_JointState_publisher_ = node_handle_.advertise<sensor_msgs::JointState>("/schunk/position/joint_states", 1);

		// Set up the schunk status publisher
		current_SchunkStatus_publisher_ = node_handle_.advertise<metralabs_ros::SchunkStatus>("/schunk/status", 1);
		for (uint i = 0; i < joints_list_.size(); ++i) {
			metralabs_ros::SchunkJointStatus status;
			status.jointName = joints_list_[i]->name;
			current_SchunkStatus_.joints.push_back(status);
		}

		trajectory_executer_.init(node_handle_, &power_cube_, joints_name_to_number_map_);

		ROS_INFO("Starting the trajectory executer thread");
		trajectory_executer_thread_ = boost::thread(boost::bind(&TrajectoryExecuter::main, &trajectory_executer_));


		/*
		 * /schunk/position/joint_state -> publishes joint state for kinematics modules
		 * /schunk/target_pc/joint_state <- for received position control commands
		 * /schunk/target_vc/joint_state  <- for receiving velocity control commands
		 * /schunk/status -> to publish all the statuses
		 */

		ROS_INFO("Subscribing schunk topics...");

		// those topics which must be received multiple times (for each joint) got a 10 for their message buffer
		emergency_subscriber_ = node_handle_.subscribe("/emergency", 1, &SchunkServer::cb_emergency, this);
		stop_subscriber_ = node_handle_.subscribe("/stop", 1, &SchunkServer::cb_stop, this);
		first_ref_subscriber_ = node_handle_.subscribe("/firstRef", 1, &SchunkServer::cb_firstRef, this);
		ack_subscriber_ = node_handle_.subscribe("/ack", 10, &SchunkServer::cb_ack, this);
		ack_all_subscriber_ = node_handle_.subscribe("/ackAll", 1, &SchunkServer::cb_ackAll, this);
		ref_subscriber_ = node_handle_.subscribe("/ref", 10, &SchunkServer::cb_ref, this);
		ref_all_subscriber_ = node_handle_.subscribe("/refAll", 1, &SchunkServer::cb_refAll, this);
		target_current_subscriber_ = node_handle_.subscribe("/current", 10, &SchunkServer::cb_targetCurrent, this);
		target_currents_max_all_subscriber_ = node_handle_.subscribe("/currentsMaxAll", 1, &SchunkServer::cb_targetCurrentsMaxAll, this);
		move_position_subscriber_ = node_handle_.subscribe("/movePosition", 10, &SchunkServer::cb_movePosition, this);
		move_velocity_subscriber_ = node_handle_.subscribe("/moveVelocity", 10, &SchunkServer::cb_moveVelocity, this);
		target_velocity_subscriber_ = node_handle_.subscribe("/targetVelocity", 10, &SchunkServer::cb_targetVelocity, this);
		target_acceleration_subscriber_ = node_handle_.subscribe("/targetAcceleration", 10, &SchunkServer::cb_targetAcceleration, this);

		move_all_position_subscriber_ = node_handle_.subscribe("/schunk/target_pc/joint_states", 1, &SchunkServer::cb_moveAllPosition, this);
		move_all_velocity_subscriber_ = node_handle_.subscribe("/schunk/target_vc/joint_states", 1, &SchunkServer::cb_moveAllVelocity, this);
		command_subscriber_ = node_handle_.subscribe("command", 1, &SchunkServer::cb_commandTrajectory, this);


		ROS_INFO("SchunkServer Ready");

	}

	~SchunkServer() {
		trajectory_executer_.stop();
		trajectory_executer_thread_.interrupt();
		trajectory_executer_thread_.join();
	}

	void cb_emergency(const std_msgs::Bool::ConstPtr& dummy) 	{
		ROS_INFO("emergency");
		power_cube_.emergencyStop();
	}

	void cb_stop(const std_msgs::Bool::ConstPtr& dummy) 	{
		ROS_INFO("stop");
		power_cube_.normalStopAll();
	}

	void cb_firstRef(const std_msgs::Bool::ConstPtr& dummy) 	{
		ROS_INFO("first ref");
		power_cube_.firstRef();
	}

	void cb_ack(const std_msgs::Int8::ConstPtr& id) 	{
		ROS_INFO("cb_ack: [%d]", id->data);
		power_cube_.ack(id->data);
	}

	void cb_ackAll(const std_msgs::Bool::ConstPtr& dummy) 	{
		ROS_INFO("cb_ackall");
		power_cube_.ackAll();
	}

	void cb_ref(const std_msgs::Int8::ConstPtr& id)	{
		ROS_INFO("cb_ref: [%d]", id->data);
		power_cube_.ref(id->data);
	}

	void cb_refAll(const std_msgs::Bool::ConstPtr& dummy)	{
		ROS_INFO("cb_refall");
		power_cube_.refAll();
	}

	void cb_targetCurrent(const metralabs_ros::idAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_current: [%d, %f]", data->id, data->value);
		power_cube_.setTargetCurrent(data->id, data->value);
	}

	void cb_targetCurrentsMaxAll(const std_msgs::Bool::ConstPtr& dummy)	{
		ROS_INFO("cb_currentsMax");
		power_cube_.setCurrentsToMax();
	}

	void cb_movePosition(const metralabs_ros::idAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_movePosition: [%d, %f]", data->id, data->value);
		power_cube_.movePosition(data->id, data->value);
	}

	void cb_moveVelocity(const metralabs_ros::idAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_moveVelocity: [%d, %f]", data->id, data->value);
		if (data->value == 0.0)
			power_cube_.normalStop(data->id);
		else
			power_cube_.moveVelocity(data->id, data->value);
	}

	void cb_targetVelocity(const metralabs_ros::idAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_targetVelocity: [%d, %f]", data->id, data->value);
		power_cube_.setTargetVelocity(data->id, data->value);
	}

	void cb_targetAcceleration(const metralabs_ros::idAndFloat::ConstPtr& data)	{
		ROS_INFO("cb_targetAcceleration: [%d, %f]", data->id, data->value);
		power_cube_.setTargetAcceleration(data->id, data->value);
	}


	void publishCurrentJointState() {
		current_JointState_.header.stamp = ros::Time::now();
		for (unsigned int i = 0; i < current_JointState_.name.size(); ++i) {
#if SCHUNK_NOT_AMTEC != 0
			SCHUNKMotionManipulator::ModuleConfig modCfg = m_powerCube.manipulator_.getModules().at(i);
			current_JointState_.position[i]=DEG_TO_RAD(modCfg.status_pos);
			current_JointState_.velocity[i]=DEG_TO_RAD(modCfg.status_vel);
#else
			AmtecManipulator::ModuleConfig modCfg = power_cube_.getManipulator().getModules().at(i);
			current_JointState_.position[i]=modCfg.status_pos; // amtec protocol already in rad
			current_JointState_.velocity[i]=modCfg.status_vel;
#endif
		}
		current_JointState_publisher_.publish(current_JointState_);
	}

	void publishCurrentSchunkStatus() {
		std::vector<metralabs_ros::SchunkJointStatus>::iterator it;
		uint moduleNumber;
		for (it = current_SchunkStatus_.joints.begin(); it != current_SchunkStatus_.joints.end(); ++it) {
			moduleNumber = joints_name_to_number_map_[it->jointName];
			power_cube_.getModuleStatus(moduleNumber, it->referenced, it->moving, it->progMode, it->warning,
					it->error, it->brake, it->moveEnd, it->posReached, it->errorCode, it->current);
		}
		current_SchunkStatus_publisher_.publish(current_SchunkStatus_);
	}

	void cb_moveAllPosition(const sensor_msgs::JointState::ConstPtr& data) {
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
			int joint_number = joints_name_to_number_map_[joint_name];
			ROS_INFO_STREAM("cb_PositionControl: joint "<<joint_name<<" (#"<<joint_number<<")");

			power_cube_.setCurrentsToMax();
			if (data.get()->position.size()!=0) {
				double pos = data.get()->position[i] / rad_to_degrees_needed;
				ROS_INFO_STREAM(" to position "<<pos);
				power_cube_.movePosition(joint_number, pos);
			}
			if (data.get()->velocity.size()!=0) {
				double vel = data.get()->velocity[i] / rad_to_degrees_needed;
				ROS_INFO_STREAM(" with velocity "<<vel);
				power_cube_.setTargetVelocity(joint_number, vel);
			}
			if (data.get()->effort.size()!=0) {
				double eff = data.get()->effort[i];
				ROS_INFO_STREAM(" with effort "<<eff);
				power_cube_.setTargetCurrent(joint_number, eff);
			}

		}
	}

	void cb_moveAllVelocity(const sensor_msgs::JointState::ConstPtr& data) {
#if SCHUNK_NOT_AMTEC != 0
		float rad_to_degrees_needed = RAD_TO_DEG(1);
#else
		float rad_to_degrees_needed = 1;
#endif

		// The "names" member says how many joints, the other members may be empty.
		// Not all the joints have to be specified in the message
		// and not all types must be filled
//		m_powerCube.ackAll();
		for (uint i = 0; i < data.get()->name.size(); ++i) {
			string joint_name = data.get()->name[i];
			int joint_number = joints_name_to_number_map_[joint_name];
			ROS_INFO_STREAM("cb_VelocityControl: joint "<<joint_name<<" (#"<<joint_number<<")");

//			m_powerCube.setCurrentsToMax();

			if (data.get()->velocity.size() != 0) {
				double vel = data.get()->velocity[i] / rad_to_degrees_needed;
				ROS_INFO_STREAM(" with velocity "<<vel);
				if (vel == 0.0)
					power_cube_.normalStop(joint_number);
				else
					power_cube_.moveVelocity(joint_number, vel);
			}
			if (data.get()->effort.size()!=0) {
				double eff = data.get()->effort[i];
				ROS_INFO_STREAM(" with effort "<<eff);
				power_cube_.setTargetCurrent(joint_number, eff);
			}

		}
	}

	void cb_commandTrajectory(const trajectory_msgs::JointTrajectory traj) {
		ROS_INFO("Schunk Server: received a new trajectory");
		trajectory_executer_.stop();
		while (! trajectory_executer_.isWaiting() ) {
			ROS_INFO("Waiting for the controller to be ready..");
		}
		trajectory_executer_.start(traj);
	}

private:
	ros::NodeHandle node_handle_;
	PowerCube power_cube_;
	urdf::Model arm_model_;	// A parsing of the model description
	std::vector<boost::shared_ptr<urdf::Joint> > joints_list_;
	std::map<std::string, unsigned int> joints_name_to_number_map_;

	sensor_msgs::JointState current_JointState_;
	metralabs_ros::SchunkStatus current_SchunkStatus_;
	ros::Publisher current_JointState_publisher_;
	ros::Publisher current_SchunkStatus_publisher_;

	TrajectoryExecuter trajectory_executer_;
	boost::thread trajectory_executer_thread_;

	ros::Subscriber emergency_subscriber_;
	ros::Subscriber stop_subscriber_;
	ros::Subscriber first_ref_subscriber_;
	ros::Subscriber ack_subscriber_;
	ros::Subscriber ack_all_subscriber_;
	ros::Subscriber ref_subscriber_;
	ros::Subscriber ref_all_subscriber_;
	ros::Subscriber target_current_subscriber_;
	ros::Subscriber target_currents_max_all_subscriber_;
	ros::Subscriber move_position_subscriber_;
	ros::Subscriber move_velocity_subscriber_;
	ros::Subscriber target_velocity_subscriber_;
	ros::Subscriber target_acceleration_subscriber_;

	ros::Subscriber move_all_position_subscriber_;
	ros::Subscriber move_all_velocity_subscriber_;
	ros::Subscriber command_subscriber_;
};



class ROSScitosBase : private boost::noncopyable {

public:
	ROSScitosBase(ros::NodeHandle& n, ScitosBase& base) :
		node_handle_(n),
		scitos_base_(base),
		remote_control_next_timeout_(),
		DEAD_REMOTE_CONTROL_TIMEOUT(0.9),
		activate_remote_control_timeout_(true)
	{
		odom_publisher_ = node_handle_.advertise<nav_msgs::Odometry>("odom", 20);
		sonar_publisher_ = node_handle_.advertise<sensor_msgs::Range>("sonar", 50);
		bumper_publisher_ = node_handle_.advertise<metralabs_ros::ScitosG5Bumper>("bumper", 20);

		cmd_vel_subscriber_ = node_handle_.subscribe("cmd_vel", 1, &ROSScitosBase::driveCommandCallback, this);
		bumper_reset_subscriber_ = node_handle_.subscribe("bumper_reset", 1, &ROSScitosBase::bumperResetCallback, this);
	}
	
	void loop() {

		/// dead remote control check

		if(activate_remote_control_timeout_ && ros::Time::now() > remote_control_next_timeout_) {
			ROS_INFO("remote control timeout after nonzero command, stopping robot");
			scitos_base_.setVelocity(0, 0);
			activate_remote_control_timeout_ = false;
		}


		// forward velocity data to inner ScitosBase loop
		scitos_base_.loop();


		/// The odometry position and velocities of the robot
		double x, y, th, vx, vth;
		scitos_base_.getOdometry(x, y, th, vx, vth);
		ros::Time odom_time = ros::Time::now();

		// since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		/// odometry tf
		geometry_msgs::TransformStamped odom_tf;
		odom_tf.header.stamp = odom_time;
		odom_tf.header.frame_id = "/odom";
		odom_tf.child_frame_id = "/base_link";

		odom_tf.transform.translation.x = x;
		odom_tf.transform.translation.y = y;
		odom_tf.transform.translation.z = 0.0;
		odom_tf.transform.rotation = odom_quat;

		// send the transform
		tf_broadcaster_.sendTransform(odom_tf);

		/// odometry data
		nav_msgs::Odometry odom_msg;
		odom_msg.header.stamp = odom_time;
		odom_msg.header.frame_id = "/odom";
		odom_msg.child_frame_id = "/base_link";

		// set the position
		odom_msg.pose.pose.position.x = x;
		odom_msg.pose.pose.position.y = y;
		odom_msg.pose.pose.orientation = odom_quat;

		// set the velocity
		odom_msg.twist.twist.linear.x = vx;
		odom_msg.twist.twist.angular.z = vth;

		// publish the message
		odom_publisher_.publish(odom_msg);


		/// bumper data

		bool bumper_pressed, motor_stop;
		scitos_base_.getBumperState(bumper_pressed, motor_stop);

		metralabs_ros::ScitosG5Bumper bumper_msg;
		bumper_msg.header.stamp = ros::Time::now();
		bumper_msg.bumper_pressed = bumper_pressed;
		bumper_msg.motor_stop = motor_stop;

		bumper_publisher_.publish(bumper_msg);


		/// sonar

		/// enable or disable sonar if someone or no one is listening
		static bool sonar_is_requested = false;
		bool sonar_had_been_requested = sonar_is_requested;

		sonar_is_requested = sonar_publisher_.getNumSubscribers() != 0;
		if(sonar_is_requested != sonar_had_been_requested) {
			scitos_base_.setFeature(FEATURE_SONAR, sonar_is_requested);
		}

		if(sonar_is_requested) {
			static const RangeData::Config* sonarConfig = NULL;

			// load config once
			if (sonarConfig == NULL) {
				scitos_base_.getSonarConfig(sonarConfig);    // TODO what if nonzero rubbish is read?
//				std::cout << "sonarConfig was NULL and now we read: " << sonarConfig << std::endl;
			}
			// if config is loaded, proceed..
			if (sonarConfig != NULL) {
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
						tf::Quaternion quat;
						quat.setEuler(0, 0, angle);
						tf::Transform* transform = new tf::Transform(quat, tf::Vector3(x, y, 0.25));
						sonarTransforms.push_back(*transform);
						angle += sonarConfigCircular->sensor_angle_dist;
					}
					// broadcast all transforms once
					std::vector<tf::Transform>::iterator it = sonarTransforms.begin();
					for (int i = 0; it != sonarTransforms.end(); ++it) {
						char targetframe[20];
						sprintf(targetframe, "/sonar/sonar_%02d", i++);
						tf_broadcaster_.sendTransform(
								tf::StampedTransform(*it, ros::Time::now(), "/base_link", targetframe)
							);
					}
				}



				/// sonar data

				std::vector<RangeData::Measurement> measurements;
				scitos_base_.getSonar(measurements);

				sensor_msgs::Range sonar_msg;
				sonar_msg.header.stamp = odom_time;
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
					sonar_publisher_.publish(sonar_msg);

					// resend also one transform (the according, why not)
					tf_broadcaster_.sendTransform( tf::StampedTransform(
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
	//				sonar_publisher_.publish(sonar);
	//			}

			} // if sonar config loaded
		} // if sonar active

	}

	void loopDiagnostics(ros::Publisher& diagnostics_publisher) {
		float voltage;
		float current;
		int16_t charge_state;
		int16_t remaining_time;
		int16_t charger_status;
		scitos_base_.getBatteryState(voltage, current, charge_state, remaining_time, charger_status);

		diagnostic_msgs::DiagnosticStatus battery_status;
		battery_status.level = diagnostic_msgs::DiagnosticStatus::OK;
		battery_status.name = "Battery";
		battery_status.message = "undefined";
		battery_status.hardware_id = "0a4fcec0-27ef-497a-93ba-db39808ec1af";

// TODO do me parameters
#define 	VOLTAGE_ERROR_LEVEL	23		// and below
#define 	VOLTAGE_WARN_LEVEL	24		// and below
#define 	VOLTAGE_MID_LEVEL	26		// and below // above means HIGH_LEVEL
#define 	VOLTAGE_FULL_LEVEL	28.8	// and above
#define 	CHARGER_PLUGGED 	1

		if(voltage < VOLTAGE_ERROR_LEVEL && charger_status != CHARGER_PLUGGED)
			battery_status.level = diagnostic_msgs::DiagnosticStatus::ERROR;
		else if(voltage < VOLTAGE_WARN_LEVEL && charger_status != CHARGER_PLUGGED)
			battery_status.level = diagnostic_msgs::DiagnosticStatus::WARN;

		// build text message
		battery_status.message = "High";
		if(voltage < VOLTAGE_MID_LEVEL)
			battery_status.message = "Mid";
		if(voltage < VOLTAGE_WARN_LEVEL)
			battery_status.message = "Low";
		if(voltage < VOLTAGE_ERROR_LEVEL)
			battery_status.message = "Depleted";

		battery_status.message += charger_status == CHARGER_PLUGGED ? ", charging" : ", discharging";

		if(voltage >= VOLTAGE_FULL_LEVEL)
			battery_status.message = "Fully charged";


		battery_status.values.resize(5);
		std::stringstream ss;
		battery_status.values[0].key = "Voltage";
		ss << voltage << " V";
		battery_status.values[0].value = ss.str();

		ss.str("");
		battery_status.values[1].key = "Current";
		ss << current << " A";
		battery_status.values[1].value = ss.str();

		ss.str("");
		battery_status.values[2].key = "ChargeState";
		ss << charge_state << " %";
		battery_status.values[2].value = ss.str();

		ss.str("");
		battery_status.values[3].key = "RemainingTime";
		ss << remaining_time << " min";
		battery_status.values[3].value = ss.str();

		battery_status.values[4].key = "ChargerStatus";
		battery_status.values[4].value = charger_status == CHARGER_PLUGGED ? "plugged" : "unplugged";

		/// combine and publish statii as array
		diagnostic_msgs::DiagnosticArray diag_array;
		diag_array.status.push_back(battery_status);
		diagnostics_publisher.publish(diag_array);
	}

	void dynamicReconfigureCallback(metralabs_ros::ScitosG5Config& config, uint32_t level) {
		// I wrote this macro because I couldn't find a way to read the configs parameters generically,
		// and with this macro the actual feature name only has to be named once. Improvements welcome.
		#define MAKRO_SET_FEATURE(NAME)	\
			ROS_INFO("Setting feature %s to %s", #NAME, config.NAME?"True":"False"); \
			scitos_base_.setFeature(#NAME, config.NAME)

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
		getFeatures(config_read);
	}

	void getFeatures(metralabs_ros::ScitosG5Config& config) {
		// I wrote this macro because I couldn't find a way to read the configs parameters generically,
		// and with this macro the actual feature name only has to be named once. Improvements welcome.
		#define MAKRO_GET_FEATURE(NAME)	\
			ROS_DEBUG("Current hardware feature %s is %s", #NAME, config.NAME?"True":"False"); \
			config.NAME = scitos_base_.getFeature<typeof(config.NAME)>(std::string(#NAME))

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
	void driveCommandCallback(const geometry_msgs::TwistConstPtr& msg) {
		scitos_base_.setVelocity(msg->linear.x, msg->angular.z);
		ROS_DEBUG("Received some speeds [%f %f]", msg->linear.x, msg->angular.z);
		activate_remote_control_timeout_ = msg->linear.x != 0 || msg->angular.z != 0;
		remote_control_next_timeout_ = ros::Time::now() + DEAD_REMOTE_CONTROL_TIMEOUT;
	}

	void bumperResetCallback(const std_msgs::EmptyConstPtr& dummy) {
		ROS_INFO("Resetting bumper");
		scitos_base_.resetBumper();
	}

	ros::NodeHandle& node_handle_;
	ScitosBase& scitos_base_;
	tf::TransformBroadcaster tf_broadcaster_;

	ros::Publisher odom_publisher_;
	ros::Publisher sonar_publisher_;
	ros::Publisher bumper_publisher_;

	ros::Subscriber cmd_vel_subscriber_;
	ros::Subscriber bumper_reset_subscriber_;

	ros::Time remote_control_next_timeout_;
	const ros::Duration DEAD_REMOTE_CONTROL_TIMEOUT;
	bool activate_remote_control_timeout_;
};


void diagnosticsPublishingLoop(ros::NodeHandle& n, ROSScitosBase& ros_scitos,
		ros::Publisher &diagnosticsPublisher, ros::Rate loop_rate)
{
	while (n.ok()) {
		ros_scitos.loopDiagnostics(diagnosticsPublisher);
		// This will adjust as needed per iteration
		loop_rate.sleep();
	}
}

void dynamicReconfigureUpdaterLoop(ros::NodeHandle& n, ROSScitosBase &ros_scitos,
		dynamic_reconfigure::Server<metralabs_ros::ScitosG5Config> &dynamicReconfigureServer,
		boost::recursive_mutex &mutex, ros::Rate loop_rate)
{
	metralabs_ros::ScitosG5Config config;
	while (n.ok()) {
		ros_scitos.getFeatures(config);// update config to current hardware state
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
	ROSScitosBase ros_scitos(n, base);


	/// intialize diagnostics

	ROS_INFO("Starting diagnostics...");

	ros::Publisher diagnosticsPublisher = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 50);
	boost::thread(diagnosticsPublishingLoop, n, boost::ref(ros_scitos), boost::ref(diagnosticsPublisher), ros::Rate(2));


	/// intialize dynamic_reconfigure

	ROS_INFO("Starting dynamic_reconfigure...");

	boost::recursive_mutex dyn_reconf_mutex;
	dynamic_reconfigure::Server<metralabs_ros::ScitosG5Config> dynamicReconfigureServer(dyn_reconf_mutex);

	// update config to current hardware state
	metralabs_ros::ScitosG5Config config;
	ros_scitos.getFeatures(config);	// The first time the read config is totally wrong, maybe it's the previous state from rosparam
//	ROS_INFO_STREAM("This is what the first read gives... " << config.EBC1_Enable24V);
//	ros_scitos.getFeatures(config);	// It's no timing issue as far as I tested it. 		TODO fix or proof as stable..
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
	f = boost::bind(&ROSScitosBase::dynamicReconfigureCallback, &ros_scitos, _1, _2);
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
		schunkServer.publishCurrentSchunkStatus();

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

