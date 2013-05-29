#ifndef POWERCUBE_H_
#define POWERCUBE_H_

#include <iostream>
#include <string>

#include <boost/noncopyable.hpp>

#include <MetraLabsBase.h>
#include <hardware/SCHUNKMotionManipulator.h>
#include <hardware/AmtecManipulator.h>
#include "ros/ros.h"

using namespace std;
using namespace MetraLabs::base;
using namespace MetraLabs::robotic::base;
using namespace MetraLabs::robotic::hardware;


//#define SCHUNK_NOT_AMTEC 0 	moved to ScitosServer

class PowerCube : private boost::noncopyable {

#if SCHUNK_NOT_AMTEC != 0
typedef SCHUNKMotionManipulator ManipulatorType;
#else
typedef AmtecManipulator ManipulatorType;
#endif


public:

	PowerCube();


	void init();

	int emergencyStop();
	int normalStopAll();
	int normalStop(int id);
	int firstRef();
	int ackAll();
	int ack(int id);
	int refAll();
	int ref(int id);
	int setTargetCurrent(int id, float i);
	int setCurrentsToMax();
	int movePosition(int id, float angle);
	int movePositionDuration(int id, float angle, uint16_t msecs);
	int moveVelocity(int id, float v);
	int setTargetVelocity(int id, float v); // only 4 position control
	int setTargetAcceleration(int id, float a); // only

	int movePositions(float angles[]);

	void getModuleStatus(int moduleID, uint8_t& referenced, uint8_t& moving, uint8_t& progMode, uint8_t& warning,
			uint8_t& error, uint8_t& brake, uint8_t& moveEnd, uint8_t& posReached, uint8_t& errorCode, float& current);


	const ManipulatorType& getManipulator() const {
		return manipulator_;
	}

	unsigned int getModulesCount() const {
		return modules_count_;
	}

private:
	ManipulatorType manipulator_;
	unsigned int modules_count_;
};
#endif
