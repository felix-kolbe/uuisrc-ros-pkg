#include "ScitosBase.h"
#include <iostream>

ScitosBase::ScitosBase(const char* config_file, int pArgc, char* pArgv[]) :
	odometry_handler_(this),
	sonar_handler_(this),
	battery_state_handler_(this)
{

	command_v_ = 0;
	command_w_ = 0;
	odom_x_ = 0;
	odom_y_ = 0;
	odom_theta_ = 0;
	odom_v_ = 0;
	odom_w_ = 0;

	battery_voltage_ = 0;
	battery_current_ = 0;
	battery_charge_state_ = 0;
	battery_remaining_time_ = 0;
	battery_charger_status_ = 0;

	sonar_config_ = NULL;

	using namespace MetraLabs::base;
	using namespace MetraLabs::robotic::base;
	using namespace MetraLabs::robotic::robot;

	Error tErr;

	///////////////////////////////////////////////////////////////////////////
	// Initialization

	// Create a general application object. This will also initialize
	// the library MetraLabsBase with the command line arguments.
	app_ = new Application(pArgc, pArgv);
	if (app_ == NULL) {
		fprintf(stderr, "FATAL: Can't create the application!\n");
		exit(-1);
	}

	// Get the class factory from the application.
	class_factory_ = app_->getClassFactory();
	if (class_factory_ == NULL) {
		fprintf(stderr, "FATAL: Cannot get the ClassFactory!\n");
		exit(-1);
	}

	// Load some parameters for the robot SCITOS-G5.
	ParameterNode tRobotCfg("RobotCfg");
	if ((tErr = tRobotCfg.readFromFile(config_file)) != OK) {
		fprintf(stderr, "FATAL: Can't read parameter file. Code: %s\n", getErrorString(tErr).c_str());
		exit(-1);
	}

	///////////////////////////////////////////////////////////////////////////

	// Get the blackboard
	blackboard_ = app_->getBlackboard();
	if (blackboard_ == NULL) {
		fprintf(stderr, "FATAL: Cannot get the Blackboard!\n");
		exit(-1);
	}

	///////////////////////////////////////////////////////////////////////////
	// Robot creation and start-up

	// Create the robot interface for SCITOS-G5.
	robot_ = createInstance<Robot>(class_factory_,
			"b07fb034-83c1-446c-b2df-0dd6aa46eef6");
	if (robot_ == NULL) {
		fprintf(stderr, "FATAL: Failed to create the robot. Abort!\n");
		exit(-1);
	}

	// Pre-Initialize the robot
	if ((tErr = robot_->preInitializeClient(&tRobotCfg)) != OK) {
		fprintf(stderr, "FATAL: Failed to pre-initialize the robot. Code: %s\n", getErrorString(tErr).c_str());
		exit(-1);
	}

	// Assign robot to blackboard
	robot_->setPhysicalName("Robot",                  "MyRobot");
	robot_->setPhysicalName("BatteryState",           "MyRobot.BatteryState");
	robot_->setPhysicalName("Drive.Odometry",         "MyRobot.Odometry");
	robot_->setPhysicalName("Drive.VelocityCmd",      "MyRobot.VelocityCmd");
	robot_->setPhysicalName("RangeFinder.Sonar.Data", "MyRobot.Sonar");
	robot_->setPhysicalName("Bumper.Bumper.Data",     "MyRobot.Bumper");
	robot_->setPhysicalName("Bumper.Bumper.ResetCmd", "MyRobot.BumperResetCmd");
	if ((tErr = robot_->assignToBlackboard(blackboard_, true)) != OK) {
		fprintf(stderr, "FATAL: Failed to assign the robot to the blackboard. Code: %s\n", getErrorString(tErr).c_str());
		exit(-1);
	}

	// Initialize the robot
	if ((tErr = robot_->initializeClient(&tRobotCfg)) != OK) {
		fprintf(stderr, "FATAL: Failed to initialize the robot. Code: %s\n", getErrorString(tErr).c_str());
		exit(-1);
	}

	///////////////////////////////////////////////////////////////////////////
	// Blackboard activation

	// Start the blackboard.
	if ((tErr = blackboard_->startBlackboard()) != OK) {
		fprintf(stderr, "FATAL: Failed to start the blackboard. Code: %s\n", getErrorString(tErr).c_str());
		exit(-1);
	}

	///////////////////////////////////////////////////////////////////////////
	// Odometry callback registration

	odometry_data_ = NULL;
	tErr = getDataFromBlackboard<BlackboardDataOdometry>(blackboard_,
			"MyRobot.Odometry", odometry_data_);
	if (tErr != OK) {
		fprintf(stderr, "FATAL: Failed to get the odometry data from the blackboard! Code: %s\n", getErrorString(tErr).c_str());
		exit(-1);
	}

	odometry_data_->addCallback(&odometry_handler_);

	///////////////////////////////////////////////////////////////////////////
	// Sonar callback registration

	sonar_data_ = NULL;
	tErr = getDataFromBlackboard<BlackboardDataRange>(blackboard_,
			"MyRobot.Sonar", sonar_data_);
	if (tErr != OK) {
		fprintf(stderr, "FATAL: Failed to get the Sonar data from the blackboard! Is it specified in the XML config? Code: %s\n", getErrorString(tErr).c_str());
//		exit(-1);  no, let the robot start wihtout sonar, even if it wasn't specified in the XML config.
	}
	else
		sonar_data_->addCallback(&sonar_handler_);

	//"RangeFinder.Sonar.Data": class BlackboardDataRange (UUID: fa0b925f-d394-4efd-b8ff-8386442d6234)


	///////////////////////////////////////////////////////////////////////////
	// Battery callback registration

	battery_state_data_ = NULL;
	tErr = getDataFromBlackboard<BlackboardDataBatteryState>(blackboard_,
			"MyRobot.BatteryState", battery_state_data_);
	if (tErr != OK) {
		fprintf(stderr, "FATAL: Failed to get the battery state data from the blackboard! Code: %s\n", getErrorString(tErr).c_str());
		exit(-1);
	}

	battery_state_data_->addCallback(&battery_state_handler_);


	///////////////////////////////////////////////////////////////////////////
	// BumperResetCmd data registration

	bumper_reset_cmd_ = NULL;

	tErr = getDataFromBlackboard<BlackboardDataUInt8>(blackboard_,
			"MyRobot.BumperResetCmd", bumper_reset_cmd_);
	if (tErr != OK) {
		fprintf(stderr, "FATAL: Failed to get the bumper reset command from the blackboard! Code: %s\n", getErrorString(tErr).c_str());
		exit(-1);
	}



	///////////////////////////////////////////////////////////////////////////

	// Start the robot.
	if ((tErr = robot_->startClient()) != OK) {
		fprintf(stderr, "FATAL: Failed to start the robot system. Code: %s\n", getErrorString(tErr).c_str());
		exit(-1);
	}

	velocity_data_ = NULL;
	tErr = getDataFromBlackboard<BlackboardDataVelocity>(blackboard_, 
			"MyRobot.VelocityCmd", velocity_data_);
	if (tErr != OK) {
		fprintf(stderr, "FATAL: Failed to get the velocity data from the blackboard! Code: %s\n", getErrorString(tErr).c_str());
		exit(-1);
	}

}


void ScitosBase::loop() {
    velocity_data_->setVelocity(command_v_, command_w_);
    velocity_data_->setModified();
}

void ScitosBase::setVelocity(double v, double w) {
    command_v_ = v;
    command_w_ = w;
}

void ScitosBase::publishOdometry(double x, double y, double theta, double v, double w) {
    odom_x_ = x;
    odom_y_ = y;
    odom_theta_ = theta;
    odom_v_ = v;
    odom_w_ = w;
}
void ScitosBase::getOdometry(double& x, double& y, double& theta, double& v, double& w) {
    x = odom_x_;
    y = odom_y_;
    theta = odom_theta_;
    v = odom_v_;
    w = odom_w_;
}

void ScitosBase::publishSonar(std::vector<RangeData::Measurement> measurements) {
	range_measurements_ = measurements;
}
void ScitosBase::getSonar(std::vector<RangeData::Measurement>& measurements) {
	measurements = range_measurements_;
}

void ScitosBase::publishSonarConfig(const RangeData::Config* sonar_config) {
	sonar_config_ = sonar_config;
}
void ScitosBase::getSonarConfig(const RangeData::Config*& sonar_config) {
	sonar_config = sonar_config_;
}


void ScitosBase::publishBatteryState(float pVoltage, float pCurrent, int16_t pChargeState,
		int16_t pRemainingTime, int16_t pChargerStatus) {
	battery_voltage_ = pVoltage;
	battery_current_ = pCurrent;
	battery_charge_state_ = pChargeState;
	battery_remaining_time_ = pRemainingTime;
	battery_charger_status_ = pChargerStatus;
}

void ScitosBase::getBatteryState(float& pVoltage, float& pCurrent, int16_t& pChargeState,
		int16_t& pRemainingTime, int16_t& pChargerStatus) {
	pVoltage = battery_voltage_;
	pCurrent = battery_current_;
	pChargeState = battery_charge_state_;
	pRemainingTime = battery_remaining_time_;
	pChargerStatus = battery_charger_status_;
}



ScitosBase::~ScitosBase() {
	Error tErr;
	if ((tErr = blackboard_->stopBlackboard()) != OK)
		fprintf(stderr, "ERROR: Failed to stop the blackboard. Code: %s\n", getErrorString(tErr).c_str());

	// Stop the robot.
	if ((tErr = robot_->stopClient()) != OK)
		fprintf(stderr, "ERROR: Failed to stop the robot system. Code: %s\n", getErrorString(tErr).c_str());

	// Destroy the robot
	if ((tErr = robot_->destroyClient()) != OK)
		fprintf(stderr, "ERROR: Failed to destroy the robot system. Code: %s\n", getErrorString(tErr).c_str());

	// Delete the application object
	delete app_;
}


//	FEATURE_BOOL = 0,  /**< A boolean feature. */
//	FEATURE_INT,       /**< A integer feature. */
//	FEATURE_FLOAT,     /**< A float feature. */
//	FEATURE_STRING     /**< A string feature. */

template<>
void ScitosBase::setFeature<bool>(std::string name, bool value) {
	robot_->setFeatureBool(name, value);
}
template<>
void ScitosBase::setFeature<int>(std::string name, int value) {
	robot_->setFeatureInt(name, value);
}
template<>
void ScitosBase::setFeature<float>(std::string name, float value) {
	robot_->setFeatureFloat(name, value);
}
template<>
void ScitosBase::setFeature<std::string>(std::string name, std::string value) {
	robot_->setFeatureString(name, value);
}


template<>
bool ScitosBase::getFeature(std::string name) {
	bool value;
	robot_->getFeatureBool(name, value);
	return value;
}
template<>
int ScitosBase::getFeature(std::string name) {
	int value;
	robot_->getFeatureInt(name, value);
	return value;
}
template<>
float ScitosBase::getFeature(std::string name) {
	float value;
	robot_->getFeatureFloat(name, value);
	return value;
}
template<>
std::string ScitosBase::getFeature(std::string name) {
	MString value;
	robot_->getFeatureString(name, value);
	return value;
}
