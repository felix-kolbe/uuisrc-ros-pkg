#ifndef __SCITOSBASE__
#define __SCITOSBASE__

#include <MetraLabsBase.h>
#include <config/MLRobotic_config.h>
#include <base/Application.h>
#include <robot/Robot.h>

using namespace MetraLabs::base;
using namespace MetraLabs::robotic::base;
using namespace MetraLabs::robotic::robot;


class ScitosBase {

	public:
	ScitosBase(const char*, int pArgc, char* pArgv[]);
	~ScitosBase();

	void publish_odometry(double x, double y, double theta, double v, double w);
	void get_odometry(double& x, double& y, double& theta, double& v, double& w);

	void publish_sonar(std::vector<RangeData::Measurement> measurements);
	void get_sonar(std::vector<RangeData::Measurement>& measurements);
	void publish_sonar_config(const RangeData::Config* sonarConfig);
	void get_sonar_config(const RangeData::Config*& sonarConfig);

	void publish_batteryState(float pVoltage, float pCurrent, int16_t pChargeState,
			int16_t pRemainingTime, int16_t pChargerStatus);
	void get_batteryState(float& pVoltage, float& pCurrent, int16_t& pChargeState,
			int16_t& pRemainingTime, int16_t& pChargerStatus);

	void set_velocity(double v, double w);
	void loop();

	void reset_bumper() {
		tBumperResetCmd->set(0, true, true, MetraLabs::base::MTime::now());
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
				m_base->publish_odometry(tPose.getX(), tPose.getY(), tPose.getPhi(),
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

//				const RangeData::Config* mSonarConfig = tSonarData->getConfig();
//
//				RangeData::Config* mSonarConfig = tSonarData->getConfig();

				m_base->publish_sonar(tRangeMeasurements);
				m_base->publish_sonar_config(tSonarData->getConfig());
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

				m_base->publish_batteryState(
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
	Application* tApp;
	ClassFactory* tClassFactory;
	Blackboard* tBlackboard;
	Robot* tRobot;

	BlackboardDataOdometry* tOdometryData;
	BlackboardDataVelocity* tVelocityData;
	BlackboardDataRange* tSonarData;
	BlackboardDataBatteryState* tBatteryStateData;
	BlackboardDataUInt8* tBumperResetCmd;

	OdometryCallbackHandler tOdometryHandler;
	SonarCallbackHandler tSonarHandler;
	BatteryStateCallbackHandler tBatteryStateHandler;

	double m_x;
	double m_y;
	double m_theta;
	double m_v;
	double m_w;

	double m_command_v;
	double m_command_w;

	std::vector<RangeData::Measurement> mRangeMeasurements;
	const RangeData::Config* mSonarConfig;

	float m_pVoltage;
	float m_pCurrent;
	int16_t m_pChargeState;
	int16_t m_pRemainingTime;
	int16_t m_pChargerStatus;


};

#endif
