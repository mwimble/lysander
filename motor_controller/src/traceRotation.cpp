#include "ros/ros.h"
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include "std_msgs/String.h"
#include <tf/transform_datatypes.h>

using namespace std;

sensor_msgs::Imu imu;
nav_msgs::Odometry odom;
nav_msgs::Odometry mappedOdom;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	imu = *msg;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	odom = *msg;
}

void mappedOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	mappedOdom = *msg;
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "trace_rotation_node");
	ros::NodeHandle rosNode;
	ros::Subscriber imuSub = rosNode.subscribe("kaimi_imu/imu", 1, imuCallback);
	ros::Subscriber odomSub = rosNode.subscribe("odom", 1, odomCallback);
	ros::Subscriber mappedOdomSub = rosNode.subscribe("odometry/filtered_map", 1, mappedOdomCallback);

	ros::Rate r(10);

	tf::Quaternion q;
	tf::Matrix3x3 m;
	double imuRoll, imuPitch, imuYaw;
	double odomRoll, odomPitch, odomYaw;
	double mappedOdomRoll, mappedOdomPitch, mappedOdomYaw;
	double t = 0;
	while (ros::ok()) {
		q = tf::Quaternion(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
		m = tf::Matrix3x3(q);
		m.getRPY(imuRoll, imuPitch, imuYaw);

		q = tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
		m = tf::Matrix3x3(q);
		m.getRPY(odomRoll, odomPitch, odomYaw);

		q = tf::Quaternion(mappedOdom.pose.pose.orientation.x, mappedOdom.pose.pose.orientation.y, mappedOdom.pose.pose.orientation.z, mappedOdom.pose.pose.orientation.w);
		m = tf::Matrix3x3(q);
		m.getRPY(mappedOdomRoll, mappedOdomPitch, mappedOdomYaw);

		printf("[%3.1f] IMU %7.4f, ODOM: %7.4f, MAP: %7.4f\n", t, imuYaw, odomYaw, mappedOdomYaw);
		t += 0.1;
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}