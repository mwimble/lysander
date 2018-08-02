#include "ros/ros.h"

#include "motor_controller/LysanderMotor.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "WRDifferentialDrive_node");
	ros::NodeHandle nh;

	ROS_INFO("[WRDifferentialDrive_node] starting spinner");
	ros::AsyncSpinner spinner(50);
	spinner.start();

  	boost::shared_ptr<LysanderMotor> hw;

	bool simulate = false; //#####

  	if (simulate) {
    	//###hw.reset(new btr::SimHWInterface(nh));
  	} else {
    	hw.reset(new LysanderMotor(nh));
  	}

  	ROS_INFO("[WRDifferentialDrive_node] about to call hw->init");
  	hw->init();
  	hw->controlLoop();
	
	return 0;
}