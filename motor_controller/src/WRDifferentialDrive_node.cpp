#include "ros/ros.h"

#include "motor_controller/WRDifferentialDrive.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "WRDifferentialDrive_node");
	ros::NodeHandle nh;

	ROS_INFO("[WRDifferentialDrive_node] starting spinner");

	ros::AsyncSpinner spinner(2);
	spinner.start();

	std::string n = "serial";
  	ros::NodeHandle ns(nh, n);

  	ROS_INFO("[WRDifferentialDrive_node] building shared ptr");
  	boost::shared_ptr<WRDifferentialDrive> hw;

	bool simulate = false; //#####

	ROS_INFO("[WRDifferentialDrive_node] About to call hw.reset");
  	if (simulate) {
    	//###hw.reset(new btr::SimHWInterface(nh));
  	} else {
    	hw.reset(new WRDifferentialDrive(nh));
  	}

  	ROS_INFO("[WRDifferentialDrive_node] about to call hw->init");
  	hw->init();

	// Start the control loop
  	//###ros_control_boilerplate::GenericHWControlLoop control_loop(nh, hw);

	ros::Rate rate(20);

	// Wait until shutdown signal recieved
  	ros::waitForShutdown();
	
	return 0;
}