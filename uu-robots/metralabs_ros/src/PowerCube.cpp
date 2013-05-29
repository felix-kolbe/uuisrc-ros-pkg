#include <iostream>
#include "PowerCube.h"

using namespace std;

#if SCHUNK_NOT_AMTEC != 0
#define ID_OFFSET	100
#else
#define ID_OFFSET	20
#endif


PowerCube::PowerCube() :
		modules_count_(0)
{
}

void PowerCube::init() {
#if SCHUNK_NOT_AMTEC != 0
	ROS_INFO("Searching SCHUNK motion modules. Please wait...");
#else
	ROS_INFO("Searching AMTEC motion modules. Please wait...");
#endif
	Error tErr = manipulator_.initialize();
	if (tErr != OK) {
		ROS_ERROR("Failed to initialise the SCHUNK Manipulator. Code: %s\n", getErrorString(tErr).c_str());
		exit(1);
	}
	modules_count_ = manipulator_.getModules().size();

#if SCHUNK_NOT_AMTEC != 0
	ROS_INFO("Found %d SCHUNK modules", modules_count_);
#else
	ROS_INFO("Found %d AMTEC modules", modules_count_);
#endif

	ackAll(); // needed to ref the gripper
}

int PowerCube::emergencyStop() {
	for (unsigned int id = 0; id < modules_count_; ++id)
	{
		if(id == modules_count_-1)
			ROS_WARN("emergency: ignoring module %d", id+ID_OFFSET);
		else {
			ROS_WARN("emergency stopping module %d", id+ID_OFFSET);
#if SCHUNK_NOT_AMTEC != 0
			manipulator_.emergencyStop(id + ID_OFFSET);
#else
			manipulator_.halt(id + ID_OFFSET);
#endif
		}
	}
	return 0;
}

int PowerCube::normalStopAll() {
	for (unsigned int id = 0; id < modules_count_; ++id)
		normalStop(id);
	return 0;
}

int PowerCube::normalStop(int id) {
#if SCHUNK_NOT_AMTEC != 0
	manipulator_.stop(id + ID_OFFSET);
#else
	manipulator_.softStop(id + ID_OFFSET);
#endif
	return 0;
}

int PowerCube::firstRef() {
#if SCHUNK_NOT_AMTEC != 0
	ackAll();
	sleep(1);
	ref();
	sleep(15);
	ackAll();
	sleep(1);
	ref();
	sleep(15);
#else
	// TODO if there needs to be sth here
#endif
	return 0;
}

int PowerCube::ackAll() {
	for (unsigned int id = 0; id < modules_count_; ++id) {
#if SCHUNK_NOT_AMTEC != 0
		const SCHUNKMotionManipulator::ModuleConfig *moduleConfig;
		manipulator_.getModuleConfig(id + ID_OFFSET, moduleConfig);
		if (moduleConfig->error_code!=217) 	// only do an ackall on the non emergency stopped
			this->ackAll(id);
#else
		const AmtecManipulator::ModuleConfig *moduleConfig;
		manipulator_.getModuleConfig(id + ID_OFFSET, moduleConfig);

		// TODO cleanly integrate this fix for our gripper.
		if(moduleConfig->linear) { // suspect a gripper that is safe to reference
			if(!moduleConfig->status_flags.flags.home_ok) { // if not referenced, do so
				ref(id); // start reference
				ros::Duration(4).sleep(); // wait to finish reference
			}
			if(id + ID_OFFSET == 25) { // set movable speeds in case too low ones were set
				setTargetAcceleration(id, 0.04);	// set gripper acceleration
				setTargetVelocity(id, 0.04);		// and velocity to a nice value
			}
		}

		// TODO test status if needed at all and ack or reset		// TODO i don't know whats about these lines
//		this->ack(i); TODO was this really the disturbing thing?
#endif
	}

	return 0;
}

int PowerCube::ack(int id) {
#if SCHUNK_NOT_AMTEC != 0
	manipulator_.ack(id + ID_OFFSET);
#else
	manipulator_.reset(id + ID_OFFSET); // TODO reset?
#endif
	return 0;
}

int PowerCube::refAll() {
	for (unsigned int id = 0; id < modules_count_; ++id) {
		if(id == modules_count_-1)
			;// dont ref the gripper anymore
		else {
			ack(id);
			manipulator_.ref(id + ID_OFFSET);
		}
	}
	return 0;
}

int PowerCube::ref(int id) {
	ack(id);
	manipulator_.ref(id + ID_OFFSET);
											// TODO cleanly integrate this fix for our gripper.
											if(id + ID_OFFSET == 25) {
												setTargetAcceleration(id, 0.04); // set gripper acceleration
												setTargetVelocity(id, 0.04);	// and velocity to a nice value
											}
//	ack(id);  TODO is this disturbing the gripper?
	return 0;
}

int PowerCube::setTargetCurrent(int id, float i) {
#if SCHUNK_NOT_AMTEC != 0
	manipulator_.setTargetCurrent(id + ID_OFFSET, i);
#else
	// TODO set current through modulconfig?
//	const AmtecManipulator::ModuleConfig cfg;
//	manipulator_.getModuleConfig(id + ID_OFFSET, &cfg);
//	no setter.. ?
#endif
	return 0;
}

int PowerCube::setCurrentsToMax() {
	for (unsigned int id = 0; id < modules_count_; ++id)
		setTargetCurrent(id, manipulator_.getModules().at(id).max_current);
	return 0;
}

int PowerCube::setTargetVelocity(int id, float v) {
	manipulator_.setTargetVelocity(id + ID_OFFSET, v);
	return 0;
}

int PowerCube::movePosition(int id, float angle) {
	ack(id);
#if SCHUNK_NOT_AMTEC != 0
	manipulator_.movePos(id + ID_OFFSET, angle);
#else
	manipulator_.execMotion(id + ID_OFFSET, AmtecManipulator::MOTION_FRAMP_MODE, angle, 0);
#endif
	return 0;
}

int PowerCube::movePositionDuration(int id, float angle, uint16_t msecs) {
	ack(id);
#if SCHUNK_NOT_AMTEC != 0
	// TODO not supported?
#else
	manipulator_.execMotion(id + ID_OFFSET, AmtecManipulator::MOTION_FSTEP_MODE, angle, msecs);
#endif
	return 0;
}

int PowerCube::moveVelocity(int id, float v) {
	//this->ack(id);
	/* Set target current to max? -1 maybe */
	//this->pManipulator->setTargetCurrent(id, 25);
	//this->ack(id);
#if SCHUNK_NOT_AMTEC != 0
	manipulator_.moveVel(id + ID_OFFSET, v);
#else
	manipulator_.execMotion(id + ID_OFFSET, AmtecManipulator::MOTION_FVEL_MODE, v, 0);
#endif
	return 0;
}

int PowerCube::setTargetAcceleration(int id, float a) {
	manipulator_.setTargetAcceleration(id + ID_OFFSET, a);
	return 0;
}

int PowerCube::movePositions(float angles[5])
{
	// for not using the gripper (crash the camera) use 7
	for (unsigned int id = 0; id < modules_count_; ++id)
		movePosition(id, angles[id]);

	return 0;
}
void PowerCube::getModuleStatus(int moduleID,
		uint8_t& referenced, uint8_t& moving, uint8_t& progMode, uint8_t& warning, uint8_t& error,
		uint8_t& brake, uint8_t& moveEnd, uint8_t& posReached, uint8_t& errorCode, float& current ) {
#if SCHUNK_NOT_AMTEC != 0
	const SCHUNKMotionManipulator::ModuleConfig *moduleConfig;
	manipulator_.getModuleConfig(moduleID + ID_OFFSET, moduleConfig);

	brake = moduleConfig->status_flags.flags.brake==1;
	error = moduleConfig->status_flags.flags.error==1;
	moveEnd = moduleConfig->status_flags.flags.move_end==1;
	moving = moduleConfig->status_flags.flags.moving==1;
	posReached = moduleConfig->status_flags.flags.pos_reached==1;
	progMode = moduleConfig->status_flags.flags.prog_mode==1;
	referenced = moduleConfig->status_flags.flags.referenced==1;
	warning = moduleConfig->status_flags.flags.warning==1;
	errorCode = moduleConfig->error_code;
	current = moduleConfig->norm_current;

#else
	const AmtecManipulator::ModuleConfig *moduleConfig;
	manipulator_.getModuleConfig(moduleID + ID_OFFSET, moduleConfig);
	brake = moduleConfig->status_flags.flags.brake==1;
	error = moduleConfig->status_flags.flags.error==1;
	moveEnd = moduleConfig->status_flags.flags.halted==1; // TODO check if used correctly
	moving = moduleConfig->status_flags.flags.motion==1;
	posReached = moduleConfig->status_flags.flags.brake==0; // TODO // no better equivalent // really?
	progMode = 0; // no equivalent
	referenced = moduleConfig->status_flags.flags.home_ok==1;
	warning = moduleConfig->status_flags.flags.cur_limit==1;
	errorCode = 0; // TODO no equivalent, check if needed by caller
	current = moduleConfig->status_cur;

#endif

}
