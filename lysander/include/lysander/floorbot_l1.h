#ifndef __FLOORBOT_L1
#define __FLOORBOT_L1

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>


class Floorbot_L1 {
private:
	// ROS node handle.
	ros::NodeHandle nh_;

	// Subscribers.
	ros::Subscriber lidar_sub_;

	// Handle a laser scan message.
	void handleLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan);

public:
	Floorbot_L1();

};

#endif