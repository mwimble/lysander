#include <boost/bind.hpp>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Imu.h>
#include "std_msgs/String.h"
#include <tf/transform_datatypes.h>

ros::CallbackQueue queue;

using namespace std;

unsigned long imuCount;
unsigned long odomCount;

sensor_msgs::Imu imu;
nav_msgs::Odometry odom;
geometry_msgs::Twist cmdVel;

double x_dist;
double y_dist;
double x_pos;
double y_pos;
static string odom_topic;
static string imu_topic;

ros::Rate* r;
sensor_msgs::Imu lastImu;
nav_msgs::Odometry lastOdom;
ros::Publisher cmdvelPublisher;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
	imu = *msg;
	imuCount++;
	//ROS_INFO("[imuCallback] imuCount: %d", imuCount);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
	odom = *msg;
	odomCount++;
	//ROS_INFO("[odomCallback] odomCount: %d", odomCount);
}

double eulerFromImu(sensor_msgs::Imu imu) {
	tf::Quaternion q = tf::Quaternion(imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w);
	tf::Matrix3x3  m = tf::Matrix3x3(q);
	double imuRoll, imuPitch, imuYaw;
	m.getRPY(imuRoll, imuPitch, imuYaw);
	ROS_INFO("--- x: %7.4f, y: %7.4f, z: %7.4f, w: %7.4f, roll: %7.4f, pitch: %7.4f, yaw: %7.4f",
		imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w,
		imuRoll * 180 / M_PI, imuPitch * 180 / M_PI, imuYaw * 180 / M_PI);
	return imuYaw * 180 / M_PI;
}

void moveStraight(double dist) {
	ROS_INFO("[squareTest] About to move straight for %7.4f meters from x: %7.4f, y: %7.4f, z: %7.4f", 
			 x_dist,
			 lastOdom.pose.pose.position.x,
			 lastOdom.pose.pose.position.y,
			 eulerFromImu(lastImu));

	while (ros::ok()) {
		double dx = odom.pose.pose.position.x - lastOdom.pose.pose.position.x;
		double dy = odom.pose.pose.position.y - lastOdom.pose.pose.position.y;
		double distTraveled = sqrt(dx * dx + dy * dy);
		if (distTraveled < dist) {
			cmdVel.linear.x = 0.25;
			cmdVel.angular.z = 0;
			cmdvelPublisher.publish(cmdVel);
			ros::spinOnce();
			r->sleep();
		} else {
			break;
		}
	}

	cmdVel.linear.x = 0.0;
	cmdVel.angular.z = 0.0;
	cmdvelPublisher.publish(cmdVel);
	ros::spinOnce();
	r->sleep();
}

void turn(double degrees) {
	ROS_INFO("[squareTest] About to turn %7.4f degrees from x: %7.4f, y: %7.4f, z: %7.4f", 
			 degrees,
			 lastOdom.pose.pose.position.x,
			 lastOdom.pose.pose.position.y,
			 eulerFromImu(lastImu));
	double dGoal = eulerFromImu(lastImu) + degrees;
	if (dGoal < 0) { dGoal += 360; }
	
	while (ros::ok()) {
		double dCurr = eulerFromImu(imu);
		if (dCurr < 0) { dCurr += 360; }
		ROS_INFO("[###] dCurr: %7.4f, dGoal: %7.4f", dCurr, dGoal);
		if ((abs(dCurr - dGoal) > 1.0)) {
			cmdVel.linear.x = 0.0;
			cmdVel.angular.z = (degrees < 0) ? - M_PI / 10.0 : M_PI / 10.0;
			cmdvelPublisher.publish(cmdVel);
			ros::spinOnce();
			r->sleep();
		} else {
			break;
		}
	}

	cmdVel.linear.x = 0.0;
	cmdVel.angular.z = 0.0;
	cmdvelPublisher.publish(cmdVel);
	ros::spinOnce();
	r->sleep();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "square_tester");
	ros::NodeHandle rosNode;
	r = new ros::Rate(10);

	x_dist = 1.0;
	y_dist = 1.0;
	x_pos = 1.0;
	y_pos = 1.0;
	imuCount = 0;
	odomCount = 0;
	odom_topic = "/odom";
	imu_topic = "/kaimi_imu_node/imu";

	rosNode.param<double>("x_dist", x_dist, 1.0);
	rosNode.param<double>("y_dist", y_dist, 1.0);
	rosNode.param<string>("imu_topic", imu_topic, "/kaimi_imu/imu");
	rosNode.param<string>("odom_topic", odom_topic, "/odom");

	ROS_INFO("[squareTest] imu_topic: %s", imu_topic.c_str());
	ROS_INFO("[squareTest] odom_topic: %s", odom_topic.c_str());
	ROS_INFO("[squareTest] x_dist: %7.4f", x_dist);
	ROS_INFO("[squareTest] y_dist: %7.4f", y_dist);

	ros::Subscriber imuSub = rosNode.subscribe("kaimi_imu/imu", 1, imuCallback);
	ros::Subscriber odomSub = rosNode.subscribe("odom", 1, odomCallback);

	cmdvelPublisher = rosNode.advertise<geometry_msgs::Twist>("/cmd_vel", 50);

	while (ros::ok() && (imuCount < 5)) {
		ROS_INFO("[squareTest] waiting on Imu data, imuCount: %ld", imuCount);
		ros::spinOnce();
		r->sleep();
	}

	lastImu = imu;

	while (ros::ok() && (odomCount < 5)) {
		ROS_INFO("[squareTest] waiting on Odometry data, odomCount: %ld", odomCount);
		ros::spinOnce();
		r->sleep();
	}

	lastOdom = odom;

	// ---
	ROS_INFO("[squareTest] About to make first move");

	moveStraight(x_dist);

	ROS_INFO("[squareTest] End first move, x: %7.4f, y: %7.4f, z: %7.4f",
			 odom.pose.pose.position.x,
			 odom.pose.pose.position.y,
			 eulerFromImu(imu));
	ROS_INFO("[squareTest] About to make first turn. ");
	lastOdom = odom;
	lastImu = imu;

	turn(-90);

	ROS_INFO("[squareTest] End first turn, x: %7.4f, y: %7.4f, z: %7.4f",
			 odom.pose.pose.position.x,
			 odom.pose.pose.position.y,
			 eulerFromImu(imu));

	// ---
	ROS_INFO("[squareTest] About to make second move");

	moveStraight(x_dist);

	ROS_INFO("[squareTest] End second move, x: %7.4f, y: %7.4f, z: %7.4f",
			 odom.pose.pose.position.x,
			 odom.pose.pose.position.y,
			 eulerFromImu(imu));
	ROS_INFO("[squareTest] About to make second turn. ");
	lastOdom = odom;
	lastImu = imu;

	turn(-90);

	ROS_INFO("[squareTest] End second turn, x: %7.4f, y: %7.4f, z: %7.4f",
			 odom.pose.pose.position.x,
			 odom.pose.pose.position.y,
			 eulerFromImu(imu));

	// ---
	ROS_INFO("[squareTest] About to make third move");

	moveStraight(x_dist);

	ROS_INFO("[squareTest] End third move, x: %7.4f, y: %7.4f, z: %7.4f",
			 odom.pose.pose.position.x,
			 odom.pose.pose.position.y,
			 eulerFromImu(imu));
	ROS_INFO("[squareTest] About to make third turn. ");
	lastOdom = odom;
	lastImu = imu;

	turn(-90);

	ROS_INFO("[squareTest] End third turn, x: %7.4f, y: %7.4f, z: %7.4f",
			 odom.pose.pose.position.x,
			 odom.pose.pose.position.y,
			 eulerFromImu(imu));

	// ---
	ROS_INFO("[squareTest] About to make forth move");

	moveStraight(x_dist);

	ROS_INFO("[squareTest] End forth move, x: %7.4f, y: %7.4f, z: %7.4f",
			 odom.pose.pose.position.x,
			 odom.pose.pose.position.y,
			 eulerFromImu(imu));
	ROS_INFO("[squareTest] About to make forth turn. ");
	lastOdom = odom;
	lastImu = imu;

	turn(-90);

	ROS_INFO("[squareTest] End forth turn, x: %7.4f, y: %7.4f, z: %7.4f",
			 odom.pose.pose.position.x,
			 odom.pose.pose.position.y,
			 eulerFromImu(imu));


	return 0;
}
