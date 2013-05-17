#ifndef POWERCUBE_H_
#define POWERCUBE_H_

#include <iostream>
#include <string>

#include <MetraLabsBase.h>
#include <hardware/SCHUNKMotionManipulator.h>
#include <hardware/AmtecManipulator.h>
#include "ros/ros.h"

using namespace std;
using namespace MetraLabs::base;
using namespace MetraLabs::robotic::base;
using namespace MetraLabs::robotic::hardware;


//#define SCHUNK_NOT_AMTEC 0 	moved to ScitosServer

class PowerCube {

public:

	PowerCube();


	void init();

	int pc_emergency_stop();
	int pc_normal_stop();
	int pc_normal_stop(int id);
	int pc_first_ref();
	int pc_ack();
	int pc_ack(int id);
	int pc_ref();
	int pc_ref(int id);
	int pc_set_current(int id, float i);
	int pc_set_currents_max();
	int pc_move_position(int id, float angle);
	int pc_move_position_duration(int id, float angle, uint16_t msecs);
	int pc_move_velocity(int id, float v);
	int pc_set_target_velocity(int id, float v); // only 4 position control
	int pc_set_target_acceleration(int id, float a); // only

	int pc_move_positions(float angles[]);

	void getModuleStatus(int moduleID, uint8_t& referenced, uint8_t& moving, uint8_t& progMode, uint8_t& warning,
			uint8_t& error, uint8_t& brake, uint8_t& moveEnd, uint8_t& posReached, uint8_t& errorCode, float& current);


#if SCHUNK_NOT_AMTEC != 0
	SCHUNKMotionManipulator mManipulator;
#else
	AmtecManipulator mManipulator;
#endif

	unsigned int modulesNum;
};
#endif
