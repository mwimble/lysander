#include "lysander/floorbot_l1.h"

Floorbot_L1::Floorbot_L1() {
	ROS_INFO("Floorbot_L1 constructor");
	lidar_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 10, boost::bind(&Floorbot_L1::handleLaserScan, this, _1));
	ROS_INFO("Floorbot_L1 scan subscriber set");
}


void Floorbot_L1::handleLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
	float angle = scan->angle_min;
	int angle_count = 0;
	int range_index = 0;
	//ROS_INFO("Floorbot_L1 ----- -----");
	while (angle <= scan->angle_max) {
		if ((angle >= (0 / 57.2958)) && (angle <= (2 / 57.2958))) {
			ROS_INFO("Floorbot_L1 angle: %7.3f, range: %5.3f", angle, scan->ranges[range_index]);
			break;
		}

		angle = angle + scan->angle_increment;
		angle_count++;
		range_index++;
	}

}