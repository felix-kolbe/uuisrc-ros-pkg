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


class ScitosBase : private boost::noncopyable {

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

	void setVelocity(double v, double w);
	void loop();

	void resetBumper() {
		bumper_reset_cmd_->set(0, true, true, MetraLabs::base::MTime::now());
	}

	template<typename FeatureType>
	void setFeature(std::string name, FeatureType value);

	template<typename FeatureType>
	FeatureType getFeature(std::string name);


private:
	class OdometryCallbackHandler : public BlackboardDataUpdateCallback
	{
	public:
		OdometryCallbackHandler(ScitosBase* base) : BlackboardDataUpdateCallback() {
			m_base = base;
		}

		void set_base(ScitosBase* base) {
			m_base = base;
		}

	private:
		// Implementation of BlackboardDataUpdateCallback
		void dataChanged(const BlackboardData* pData) {
			const BlackboardDataOdometry* tOdometryData = dynamic_cast<const BlackboardDataOdometry*>(pData);
			if (tOdometryData != NULL) {
				MTime tTime;
				Pose tPose;
				Velocity tVelocity;
				float tMileage;

				tOdometryData->getData(tPose, tVelocity, tMileage);
				m_base->publishOdometry(tPose.getX(), tPose.getY(), tPose.getPhi(),
							tVelocity.getVelocityTranslational(),
							tVelocity.getVelocityRotational());

			}
		}

		ScitosBase* m_base;
	};

private:
	class SonarCallbackHandler : public BlackboardDataUpdateCallback
	{
	public:
		SonarCallbackHandler(ScitosBase* base) : BlackboardDataUpdateCallback() {
			m_base = base;
		}

		void set_base(ScitosBase* base) {
			m_base = base;
		}

	private:
		// Implementation of BlackboardDataUpdateCallback
		void dataChanged(const BlackboardData* pData) {
			const BlackboardDataRange* tSonarData = dynamic_cast<const BlackboardDataRange*>(pData);
			if (tSonarData != NULL) {
				MTime tTime;
				const RangeData::Vector& tRangeData = tSonarData->getRangeData();

				const std::vector<RangeData::Measurement> tRangeMeasurements = tRangeData;

//				const RangeData::Config* sonar_config_ = sonar_data_->getConfig();
//
//				RangeData::Config* sonar_config_ = sonar_data_->getConfig();

				m_base->publishSonar(tRangeMeasurements);
				m_base->publishSonarConfig(tSonarData->getConfig());
			}
		}

		ScitosBase* m_base;
	};


private:
	class BatteryStateCallbackHandler : public BlackboardDataUpdateCallback
	{
	public:
		BatteryStateCallbackHandler(ScitosBase* base) : BlackboardDataUpdateCallback() {
			m_base = base;
		}

		void set_base(ScitosBase* base) {
			m_base = base;
		}

	private:
		// Implementation of BlackboardDataUpdateCallback
		void dataChanged(const BlackboardData* pData) {
			const BlackboardDataBatteryState* tBatteryStateData = dynamic_cast<const BlackboardDataBatteryState*>(pData);
			if (tBatteryStateData != NULL) {
				MTime tTime;

				m_base->publishBatteryState(
						tBatteryStateData->getVoltage(),
						tBatteryStateData->getCurrent(),
						tBatteryStateData->getChargeState(),
						tBatteryStateData->getRemainingTime(),
						tBatteryStateData->getChargerStatus()
						);
			}
		}

		ScitosBase* m_base;
	};

private:
	Application* app_;
	ClassFactory* class_factory_;
	Blackboard* blackboard_;
	Robot* robot_;

	BlackboardDataOdometry* odometry_data_;
	BlackboardDataVelocity* velocity_data_;
	BlackboardDataRange* sonar_data_;
	BlackboardDataBatteryState* battery_state_data_;
	BlackboardDataUInt8* bumper_reset_cmd_;

	OdometryCallbackHandler odometry_handler_;
	SonarCallbackHandler sonar_handler_;
	BatteryStateCallbackHandler battery_state_handler_;

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

};

#endif
