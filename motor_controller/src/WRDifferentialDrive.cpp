#include "ros/ros.h"

#include "motor_controller/WRDifferentialDrive.h"

WRDifferentialDrive::WRDifferentialDrive(ros::NodeHandle &nh, urdf::Model *urdf_model) {
	if (urdf_model == NULL) {
		loadURDF(nh, "robot_description");
	} else {
		model_ = urdf_model;
	}

   // // connect and register the joint state interface
   // hardware_interface::JointStateHandle state_handle_a("A", &pos[0], &vel[0], &eff[0]);
   // jnt_state_interface.registerHandle(state_handle_a);

   // hardware_interface::JointStateHandle state_handle_b("B", &pos[1], &vel[1], &eff[1]);
   // jnt_state_interface.registerHandle(state_handle_b);

   // registerInterface(&jnt_state_interface);

   // // connect and register the joint position interface
   // hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("A"), &cmd[0]);
   // jnt_pos_interface.registerHandle(pos_handle_a);

   // hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("B"), &cmd[1]);
   // jnt_pos_interface.registerHandle(pos_handle_b);

   // registerInterface(&jnt_pos_interface);
}

WRDifferentialDrive::~WRDifferentialDrive() {

}


void WRDifferentialDrive::init() {
	ROS_INFO("[WRDifferentialDrive::init]");
}


void WRDifferentialDrive::loadURDF(ros::NodeHandle &nh, std::string param_name) {
	std::string urdf_string;
	model_ = new urdf::Model();

	// search and wait for robot_description on param server
	while (urdf_string.empty()/*### && ros::ok()###*/) {
		std::string search_param_name;
		if (nh.searchParam(param_name, search_param_name)) {
			ROS_INFO_STREAM("[WRDifferentialDrive::loadURDF] In namespace: "
								<< nh.getNamespace()
								<< ", waiting for model URDF"
		  	 				    " on the ROS param server at search_param_name: "
								<< search_param_name);
			nh.getParam(search_param_name, urdf_string);
			break;
		} else {
			ROS_INFO_STREAM("[WRDifferentialDrive::loadURDF] In namespace: "
								<< nh.getNamespace()
								<< ", waiting for model URDF"
								" on the ROS param server at param_name: "
								<< param_name);
			nh.getParam(param_name, urdf_string);
			break;
		}

		usleep(10000);
	}

			ROS_INFO_STREAM("[WRDifferentialDrive::loadURDF] fetched: " << urdf_string.length() << " bytes");
	ROS_INFO_STREAM("[WRDifferentialDrive::loadURDF] about to initialize model with string");
	if (!model_->initString(urdf_string)) {
		ROS_ERROR_STREAM("[WRDifferentialDrive::loadURDF] Unable to initialize model with loaded URDF model");
	} else {
		ROS_INFO_STREAM("[WRDifferentialDrive::loadURDF] URDF model successfully initialized");
	}
}
