#include "motor_controller/LysanderMotor.h"

LysanderMotor::LysanderMotor(ros::NodeHandle &nh, urdf::Model *urdf_model)
	: WRDifferentialDrive(nh, urdf_model)
	, controlLoopMaxAllowedDurationDeviation_(1.0)
	, nh_(nh)
	, urdf_model_(urdf_model) {
	controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
	controlLoopHz_ = 5.0; //#####
	expectedControlLoopDuration_ = ros::Duration(1 / controlLoopHz_);
}
LysanderMotor::~LysanderMotor() {

}

void LysanderMotor::controlLoop() {
	ros::Rate rate(controlLoopHz_);
	while(ros::ok()) {
		update();
		rate.sleep();
	}
}

void LysanderMotor::read(const ros::Time& time, const ros::Duration& period) {
}


void LysanderMotor::update() {
	clock_gettime(CLOCK_MONOTONIC, &now_);
	elapsedTime_ = ros::Duration(now_.tv_sec - 
      							 lastTime_.tv_sec +
      							 (now_.tv_nsec - lastTime_.tv_nsec) / kBILLION);
	lastTime_ = now_;
	const double controlLoopCycleDurationDeviation = (elapsedTime_ - expectedControlLoopDuration_).toSec();

	if (controlLoopCycleDurationDeviation > controlLoopMaxAllowedDurationDeviation_) {
		ROS_WARN_STREAM("[LysanderMotor::update] Control loop was too slow by "
						<< controlLoopCycleDurationDeviation
						<< ", actual loop time: "
						<< elapsedTime_
						<< ", allowed deviation: "
						<< controlLoopMaxAllowedDurationDeviation_);
	}

	read(ros::Time(now_.tv_sec, now_.tv_nsec), elapsedTime_);
	controller_manager_->update(ros::Time(now_.tv_sec, now_.tv_nsec), elapsedTime_);
	write(ros::Time(now_.tv_sec, now_.tv_nsec), elapsedTime_);
}


void LysanderMotor::write(const ros::Time& time, const ros::Duration& period) {

}


const double LysanderMotor::kBILLION = 1000000000.0;
