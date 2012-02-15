#include <iostream>
#include "PowerCube.h"

using namespace std;

#if SCHUNK_NOT_AMTEC != 0
#define ID_OFFSET	100
#else
#define ID_OFFSET	20
#endif


PowerCube::PowerCube() {
//	init();

}

void PowerCube::init() {
	ROS_INFO("Searching SCHUNK motion modules. Please wait...");
	Error tErr = mManipulator.initialize();
	if (tErr != OK) {
		ROS_ERROR("Failed to initialise the SCHUNK Manipulator.");
		exit(1);
	}
	mmModules = mManipulator.getModules();
	modulesNum = mManipulator.getModules().size();

	ROS_INFO("Found %d SCHUNK modules", modulesNum);
}

int PowerCube::pc_emergency_stop() {
	for (unsigned int id = 0; id < mManipulator.getModules().size() - 1; id++) // the gripper is not stopped
#if SCHUNK_NOT_AMTEC != 0
		mManipulator.emergencyStop(id + ID_OFFSET);
#else
		mManipulator.halt(id + ID_OFFSET);
#endif
	return 0;
}

int PowerCube::pc_normal_stop() {
	for (unsigned int id = 0; id < modulesNum; id++)
		pc_normal_stop(id);
	return 0;
}

int PowerCube::pc_normal_stop(int id) {
#if SCHUNK_NOT_AMTEC != 0
  mManipulator.stop(id + ID_OFFSET);
#else
  mManipulator.softStop(id + ID_OFFSET);

#endif
  return 0;
}

int PowerCube::pc_first_ref() {
	pc_ack();
	sleep(1);
	pc_ref();
	sleep(15);
	pc_ack();
	sleep(1);
	pc_ref();
	sleep(15);
	return 0;
}

int PowerCube::pc_ack() {
	// for not initialising the gripper use mmModules.size()-1 instead of mmModules.size()
#if SCHUNK_NOT_AMTEC != 0
	const SCHUNKMotionManipulator::ModuleConfig *moduleConfig;
#else
	const AmtecManipulator::ModuleConfig *moduleConfig;
#endif
	for (size_t i = 0; i < mmModules.size() - 1; i++) {
#if SCHUNK_NOT_AMTEC != 0
		mManipulator.getModuleConfig(i + ID_OFFSET, moduleConfig);
		if (moduleConfig->error_code!=217) 	// only do an ackall on the non emergency stoped
			this->pc_ack(i);
#else
		mManipulator.getModuleConfig(i + ID_OFFSET, moduleConfig);
		// TODO test status if needed at all and ack or reset
//		this->pc_ack(i); TODO was this really the disturbing thing?
#endif
	}

	return 0;
}

int PowerCube::pc_ack(int id) {
#if SCHUNK_NOT_AMTEC != 0
	mManipulator.ack(id + ID_OFFSET);
#else
	mManipulator.reset(id + ID_OFFSET); // TODO reset?
#endif
	return 0;
}

int PowerCube::pc_ref() {
	// for not initialising the gripper use mmModules.size()-1 instead of mmModules.size()
	for (size_t id = 0; id < mManipulator.getModules().size()-1; id++) { // dont ref the gripper anymore
		pc_ack(id);
		mManipulator.ref(id + ID_OFFSET);
	}
	return 0;
}

int PowerCube::pc_ref(int id) {
	pc_ack(id);
	mManipulator.ref(id + ID_OFFSET);
	pc_ack(id);
	return 0;
}

int PowerCube::pc_set_current(int id, float i) {
#if SCHUNK_NOT_AMTEC != 0
	mManipulator.setTargetCurrent(id + ID_OFFSET, i);
#else
	// TODO set current through modulconfig?
#endif
	return 0;
}

int PowerCube::pc_set_currents_max() {
	for (size_t id = 0; id < mManipulator.getModules().size(); id++)
		pc_set_current(id, mManipulator.getModules().at(id).max_current);
	return 0;
}

int PowerCube::pc_set_target_velocity(int id, float v) {
	mManipulator.setTargetVelocity(id + ID_OFFSET, v);
	return 0;
}

int PowerCube::pc_move_position(int id, float angle) {
	pc_ack(id);
#if SCHUNK_NOT_AMTEC != 0
	mManipulator.movePos(id + ID_OFFSET, angle);
#else
	mManipulator.execMotion(id + ID_OFFSET, AmtecManipulator::MOTION_FRAMP_MODE, angle, 0);
#endif
	return 0;
}

int PowerCube::pc_move_velocity(int id, float v) {
	//this->pc_ack(id);
	/* Set target current to max? -1 maybe */
	//this->pManipulator->setTargetCurrent(id, 25);
	//this->pc_ack(id);
#if SCHUNK_NOT_AMTEC != 0
	mManipulator.moveVel(id + ID_OFFSET, v);
#else
	mManipulator.execMotion(id + ID_OFFSET, AmtecManipulator::MOTION_FVEL_MODE, v, 0);
#endif
	return 0;
}

int PowerCube::pc_set_target_acceleration(int id, float a) {
	mManipulator.setTargetAcceleration(id, a);
	return 0;
}

int PowerCube::pc_move_positions(float angles[5])
{
	// for not using the gripper (crash the camera) use 7
	for (unsigned int id = 0; id < modulesNum; id++)
		pc_move_position(id, angles[id]);

	return 0;
}
void PowerCube::getModuleStatus(int moduleID, uint8_t *referenced, uint8_t *moving,	uint8_t *progMode,	uint8_t *warning, 	uint8_t *error, uint8_t *brake, uint8_t *moveEnd, uint8_t *posReached, uint8_t *errorCode, float *current ){
#if SCHUNK_NOT_AMTEC != 0
	const SCHUNKMotionManipulator::ModuleConfig *moduleConfig;
	mManipulator.getModuleConfig(moduleID + ID_OFFSET, moduleConfig);

	*brake = moduleConfig->status_flags.flags.brake==1;
	*error = moduleConfig->status_flags.flags.error==1;
	*moveEnd = moduleConfig->status_flags.flags.move_end==1;
	*moving = moduleConfig->status_flags.flags.moving==1;
	*posReached = moduleConfig->status_flags.flags.pos_reached==1;
	*progMode = moduleConfig->status_flags.flags.prog_mode==1;
	*referenced = moduleConfig->status_flags.flags.referenced==1;
	*warning = moduleConfig->status_flags.flags.warning==1;
	*errorCode = moduleConfig->error_code;
	*current = moduleConfig->norm_current;

#else
	const AmtecManipulator::ModuleConfig *moduleConfig;
	mManipulator.getModuleConfig(moduleID + ID_OFFSET, moduleConfig);
	*brake = moduleConfig->status_flags.flags.brake==1;
	*error = moduleConfig->status_flags.flags.error==1;
	*moveEnd = moduleConfig->status_flags.flags.halted==1; // TODO check if used correctly
	*moving = moduleConfig->status_flags.flags.motion==1;
	*posReached = moduleConfig->status_flags.flags.brake==0; // no better equivalent // TODO really?
	*progMode = 0; // no equivalent
	*referenced = moduleConfig->status_flags.flags.home_ok==1;
	*warning = moduleConfig->status_flags.flags.cur_limit==1;
	*errorCode = 0; // TODO no equivalent, check if needed by caller
	*current = moduleConfig->status_cur;

#endif

}
