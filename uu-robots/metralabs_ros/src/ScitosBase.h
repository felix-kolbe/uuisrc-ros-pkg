#ifndef __SCITOSBASE__
#define __SCITOSBASE__

#include <boost/noncopyable.hpp>

#include <MetraLabsBase.h>
#include <config/MLRobotic_config.h>
#include <base/Application.h>
#include <robot/Robot.h>

using namespace MetraLabs::base;
using namespace MetraLabs::robotic::base;
using namespace MetraLabs::robotic::robot;


class ScitosBase : public BlackboardDataUpdateCallback, private boost::noncopyable {

public:
	ScitosBase(const char*, int pArgc, char* pArgv[]);
	~ScitosBase();

	void publishOdometry(double x, double y, double theta, double v, double w);
	void getOdometry(double& x, double& y, double& theta, double& v, double& w);

	void publishSonar(std::vector<RangeData::Measurement> measurements);
	void getSonar(std::vector<RangeData::Measurement>& measurements);
	void publishSonarConfig(const RangeData::Config* sonarConfig);
	void getSonarConfig(const RangeData::Config*& sonarConfig);

	void publishBatteryState(float pVoltage, float pCurrent, int16_t pChargeState,
			int16_t pRemainingTime, int16_t pChargerStatus);
	void getBatteryState(float& pVoltage, float& pCurrent, int16_t& pChargeState,
			int16_t& pRemainingTime, int16_t& pChargerStatus);

	void publishBumperState(bool pBumperPressed, bool pMotorStop);
	void getBumperState(bool& pBumperPressed, bool& pMotorStop);

	void setVelocity(double v, double w);
	void loop();

	void resetBumper() {
		bumper_reset_cmd_->set(0, true, true, MetraLabs::base::MTime::now());
	}

	template<typename FeatureType>
	void setFeature(std::string name, FeatureType value);

	template<typename FeatureType>
	FeatureType getFeature(std::string name);


	virtual void dataChanged(const BlackboardData* pData) {
		if(pData == odometry_data_) {
			odometryCallbackHandler();
		} else if(pData == sonar_data_) {
			sonarCallbackHandler();
		} else if(pData == battery_state_data_) {
			batteryStateCallbackHandler();
		} else if(pData == bumper_data_) {
			bumperDataCallbackHandler();
		}
	}

private:
	void odometryCallbackHandler() {
		MTime tTime;
		Pose tPose;
		Velocity tVelocity;
		float tMileage;

		odometry_data_->readLock();
		odometry_data_->getData(tPose, tVelocity, tMileage);
		odometry_data_->readUnlock();

		publishOdometry(tPose.getX(), tPose.getY(), tPose.getPhi(),
					tVelocity.getVelocityTranslational(),
					tVelocity.getVelocityRotational());
	}

	void sonarCallbackHandler() {
			MTime tTime;

			sonar_data_->readLock();
			const RangeData::Vector& tRangeData = sonar_data_->getRangeData();
			const RangeData::Config* tSonarConfig = sonar_data_->getConfig();
			sonar_data_->readUnlock();

			const std::vector<RangeData::Measurement> tRangeMeasurements = tRangeData;

			publishSonar(tRangeMeasurements);
			publishSonarConfig(tSonarConfig);
	}

	void batteryStateCallbackHandler() {
		MTime tTime;

		battery_state_data_->readLock();
		publishBatteryState(
				battery_state_data_->getVoltage(),
				battery_state_data_->getCurrent(),
				battery_state_data_->getChargeState(),
				battery_state_data_->getRemainingTime(),
				battery_state_data_->getChargerStatus()
				);
		battery_state_data_->readUnlock();
	}

	void bumperDataCallbackHandler() {
		MTime tTime;

		bool bumper_pressed = false;
		bool motor_stop = false;

#define BUMPER_CODE_PUSHED 0x12
#define BUMPER_CODE_LOCKED 0x02

		bumper_data_->readLock();
		BumperData::Vector bumperValues = bumper_data_->getBumperData();
		bumper_data_->readUnlock();

		for (BumperData::Vector::const_iterator it = bumperValues.begin(); it != bumperValues.end(); ++it) {
			if (*it == BUMPER_CODE_PUSHED) {
				bumper_pressed = true;
				motor_stop = true;
				break;  // no next bumper part would change any value
			}
			else if (*it == BUMPER_CODE_LOCKED) {
				motor_stop = true;
			}
		}

		publishBumperState(bumper_pressed, motor_stop);
	}


private:
	Application* app_;
	ClassFactory* class_factory_;
	Blackboard* blackboard_;
	Robot* robot_;

	BlackboardDataOdometry* odometry_data_;
	BlackboardDataRange* sonar_data_;
	BlackboardDataBatteryState* battery_state_data_;
	BlackboardDataBumper* bumper_data_;

	BlackboardDataVelocity* velocity_cmd_;
	BlackboardDataUInt8* bumper_reset_cmd_;

	double odom_x_;
	double odom_y_;
	double odom_theta_;
	double odom_v_;
	double odom_w_;

	double command_v_;
	double command_w_;

	std::vector<RangeData::Measurement> range_measurements_;
	const RangeData::Config* sonar_config_;

	float battery_voltage_;
	float battery_current_;
	int16_t battery_charge_state_;
	int16_t battery_remaining_time_;
	int16_t battery_charger_status_;

	bool bumper_pressed_;
	bool motor_stop_;
};

#endif
