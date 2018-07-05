#include "motor_controller/LysanderMotor.h"

LysanderMotor::LysanderMotor(ros::NodeHandle &nh, urdf::Model *urdf_model)
	: WRDifferentialDrive(nh, urdf_model)
	, nh_(nh)
	, urdf_model_(urdf_model) {
	controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
	controlLoopHz_ = 100.0; //#####
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
	
}


void LysanderMotor::write(const ros::Time& time, const ros::Duration& period) {

}

