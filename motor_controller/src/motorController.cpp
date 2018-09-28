#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include <stdint.h>
#include "motorSkidSteerDrive.h"
#include "motor_controller/motor_controllerConfig.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "motor_controller");
	MotorSkidSteerDrive *motorSkidSteerDrive = new MotorSkidSteerDrive();

	dynamic_reconfigure::Server<motor_controller::motor_controllerConfig> server;
	dynamic_reconfigure::Server<motor_controller::motor_controllerConfig>::CallbackType f;
	f = boost::bind(&MotorSkidSteerDrive::configCallback, motorSkidSteerDrive, _1, _2);
	server.setCallback(f);

	ROS_INFO("[motor_controller] Starting to spin...");

	ros::Rate r(1);
	float old_batt = 0.0;
	int32_t old_m1_enc = 0;
	int32_t old_m2_enc = 0;

	while (ros::ok())
	{
		try
		{
			if ((old_batt != motorSkidSteerDrive->getLogicBatteryLevel()) ||
				(old_m1_enc != motorSkidSteerDrive->getM1Encoder()) ||
				(old_m2_enc != motorSkidSteerDrive->getM2Encoder()))
			{
				// ROS_INFO("[motor_controller] m1 enc: %d, m2 enc: %d, batt: %f",
				// 		 motorSkidSteerDrive->getM1Encoder(),
				// 		 motorSkidSteerDrive->getM2Encoder(),
				// 		 motorSkidSteerDrive->getLogicBatteryLevel());
				old_batt = motorSkidSteerDrive->getLogicBatteryLevel();
				old_m1_enc = motorSkidSteerDrive->getM1Encoder();
				old_m2_enc = motorSkidSteerDrive->getM2Encoder();
			}

			ros::spinOnce();
			r.sleep();
		}
		catch (MotorSkidSteerDrive::TRoboClawException *e)
		{
			ROS_ERROR("[motor_controller] Exception: %s", e->what());
		}
		catch (...)
		{
			ROS_ERROR("[motor_controller] Unhandled exception");
		}
	}

	return 0;
}