#include "ScitosBase.h"
#include <iostream>

ScitosBase::ScitosBase(const char* config_file, int pArgc, char* pArgv[]) :
	tOdometryHandler(this),
	tSonarHandler(this)
{

	tOdometryHandler.set_base(this);

	m_command_v = 0;
	m_command_w = 0;
	m_x = 0;
	m_y = 0;
	m_theta = 0;
	m_v = 0;
	m_w = 0;


	using namespace MetraLabs::base;
	using namespace MetraLabs::robotic::base;
	using namespace MetraLabs::robotic::robot;

	Error tErr;

	///////////////////////////////////////////////////////////////////////////
	// Initialization

	// Create a general application object. This will also initialize
	// the library MetraLabsBase with the command line arguments.
	tApp = new Application(pArgc, pArgv);
	if (tApp == NULL) {
		fprintf(stderr, "FATAL: Can't create the application!\n");
		exit(-1);
	}

	// Get the class factory from the application.
	tClassFactory = tApp->getClassFactory();
	if (tClassFactory == NULL) {
		fprintf(stderr, "FATAL: Cannot get the ClassFactory!\n");
		exit(-1);
	}

	// Load some parameters for the robot SCITOS-G5.
	ParameterNode tRobotCfg("RobotCfg");
	if (tRobotCfg.readFromFile(config_file) != OK)
	{
		fprintf(stderr, "FATAL: Can't read parameter file.\n");
		exit(-1);
	}

	///////////////////////////////////////////////////////////////////////////

	// Get the blackboard
	tBlackboard = tApp->getBlackboard();
	if (tBlackboard == NULL) {
		fprintf(stderr, "FATAL: Cannot get the Blackboard!\n");
		exit(-1);
	}

	///////////////////////////////////////////////////////////////////////////
	// Robot creation and start-up

	// Create the robot interface for SCITOS-G5.
	tRobot = createInstance<Robot>(tClassFactory,
			"b07fb034-83c1-446c-b2df-0dd6aa46eef6");
	if (tRobot == NULL) {
		fprintf(stderr, "FATAL: Failed to create the robot. Abort!\n");
		exit(-1);
	}

	// Pre-Initialize the robot
	if (tRobot->preInitializeClient(&tRobotCfg) != OK) {
		fprintf(stderr, "FATAL: Failed to pre-initialize the robot.\n");
		exit(-1);
	}

	// Assign robot to blackboard
	tRobot->setPhysicalName("Robot",                  "MyRobot");
	tRobot->setPhysicalName("BatteryState",           "MyRobot.BatteryState");
	tRobot->setPhysicalName("Drive.Odometry",         "MyRobot.Odometry");
	tRobot->setPhysicalName("Drive.VelocityCmd",      "MyRobot.VelocityCmd");
	tRobot->setPhysicalName("RangeFinder.Sonar.Data", "MyRobot.Sonar");
	tRobot->setPhysicalName("Bumper.Bumper.Data",     "MyRobot.Bumper");
	tRobot->setPhysicalName("Bumper.Bumper.ResetCmd", "MyRobot.BumperResetCmd");
	if (tRobot->assignToBlackboard(tBlackboard, true) != OK) {
		fprintf(stderr, "FATAL: Failed to assign the robot to the blackboard.\n");
		exit(-1);
	}

	// Initialize the robot
	if (tRobot->initializeClient(&tRobotCfg) != OK) {
		fprintf(stderr, "FATAL: Failed to initialize the robot.\n");
		exit(-1);
	}

	///////////////////////////////////////////////////////////////////////////
	// Blackboard activation

	// Start the blackboard.
	if (tBlackboard->startBlackboard() != OK) {
		fprintf(stderr, "FATAL: Failed to start the blackboard.\n");
		exit(-1);
	}

	///////////////////////////////////////////////////////////////////////////
	// Odometry callback registration

	tOdometryData = NULL;
	tErr = getDataFromBlackboard<BlackboardDataOdometry>(tBlackboard,
			"MyRobot.Odometry", tOdometryData);
	if (tErr != OK) {
		fprintf(stderr, "FATAL: Failed to get the odometry data from the blackboard!\n");
		exit(-1);
	}

	tOdometryData->addCallback(&tOdometryHandler);

	///////////////////////////////////////////////////////////////////////////
	// Sonar callback registration

	tSonarData = NULL;
	tErr = getDataFromBlackboard<BlackboardDataRange>(tBlackboard,
			"MyRobot.Sonar", tSonarData);
	if (tErr != OK) {
		fprintf(stderr, "FATAL: Failed to get the Sonar data from the blackboard! Is it specified in the XML config?\n");
//		exit(-1);  no, let the robot start wihtout sonar, if it wasn't specified in the XML config.
	}
	else
		tSonarData->addCallback(&tSonarHandler);

	//"RangeFinder.Sonar.Data": class BlackboardDataRange (UUID: fa0b925f-d394-4efd-b8ff-8386442d6234)


	///////////////////////////////////////////////////////////////////////////

	// Start the robot.
	if (tRobot->startClient() != OK) {
		fprintf(stderr, "FATAL: Failed to start the robot system.\n");
		exit(-1);
	}

	tVelocityData = NULL;
	tErr = getDataFromBlackboard<BlackboardDataVelocity>(tBlackboard, 
			"MyRobot.VelocityCmd", tVelocityData);
	if (tErr != OK) {
		fprintf(stderr, "FATAL: Failed to get the velocity data from the blackboard!\n");
		exit(-1);
	}

}


void ScitosBase::loop() {
    tVelocityData->setVelocity(m_command_v, m_command_w);
    tVelocityData->setModified();
}

void ScitosBase::set_velocity(double v, double w) {
    m_command_v = v;
    m_command_w = w;
}

void ScitosBase::publish_odometry(double x, double y, double theta, double v, double w) {
    m_x = x;
    m_y = y;
    m_theta = theta;
    m_v = v;
    m_w = w;
}
void ScitosBase::get_odometry(double& x, double& y, double& theta, double& v, double& w) {
    x = m_x;
    y = m_y;
    theta = m_theta;
    v = m_v;
    w = m_w;
}

void ScitosBase::publish_sonar(std::vector<RangeData::Measurement> measurements) {
	mRangeMeasurements = measurements;
}
void ScitosBase::get_sonar(std::vector<RangeData::Measurement>& measurements) {
	measurements = mRangeMeasurements;
}

void ScitosBase::publish_sonar_config(const RangeData::Config* sonarConfig) {
	mSonarConfig = sonarConfig;
}
void ScitosBase::get_sonar_config(const RangeData::Config*& sonarConfig) {
	sonarConfig = mSonarConfig;
}


ScitosBase::~ScitosBase() {
    if (tBlackboard->stopBlackboard() != OK)
		fprintf(stderr, "ERROR: Failed to stop the blackboard.\n");

    // Stop the robot.
    if (tRobot->stopClient() != OK)
		fprintf(stderr, "ERROR: Failed to stop the robot system.\n");

    // Destroy the robot
    if (tRobot->destroyClient() != OK)
		fprintf(stderr, "ERROR: Failed to destroy the robot system.\n");
}


//	FEATURE_BOOL = 0,  /**< A boolean feature. */
//	FEATURE_INT,       /**< A integer feature. */
//	FEATURE_FLOAT,     /**< A float feature. */
//	FEATURE_STRING     /**< A string feature. */

template<>
void ScitosBase::setFeature<bool>(std::string name, bool value) {
	tRobot->setFeatureBool(name, value);
}
template<>
void ScitosBase::setFeature<int>(std::string name, int value) {
	tRobot->setFeatureInt(name, value);
}
template<>
void ScitosBase::setFeature<float>(std::string name, float value) {
	tRobot->setFeatureFloat(name, value);
}
template<>
void ScitosBase::setFeature<std::string>(std::string name, std::string value) {
	tRobot->setFeatureString(name, value);
}


template<>
bool ScitosBase::getFeature(std::string name) {
	bool value;
	tRobot->getFeatureBool(name, value);
	return value;
}
template<>
int ScitosBase::getFeature(std::string name) {
	int value;
	tRobot->getFeatureInt(name, value);
	return value;
}
template<>
float ScitosBase::getFeature(std::string name) {
	float value;
	tRobot->getFeatureFloat(name, value);
	return value;
}
template<>
std::string ScitosBase::getFeature(std::string name) {
	MString value;
	tRobot->getFeatureString(name, value);
	return value;
}
