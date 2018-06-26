#include <ros/ros.h>

#include <lysander/tablebot_strategy.h>
#include <geometry_msgs/Twist.h>

using namespace lysander;

TablebotStrategy::TablebotStrategy() :
	avgBackLeftMm_(0.0),
	avgBackRightMm_(0.0),
	avgFrontLeftMm_(0.0),
	avgFrontRightMm_(0.0),
	debug_(false),
	defaultForwardVelocity_(0.1),
	distanceToTableTopSlack_(10.0),
	backLeftMm_(0.0),
	backRightMm_(0.0),
	frontLeftMm_(0.0),
	frontRightMm_(0.0),
	tofFound_(false),
	tofReadCount(0) {

	ros::NodeHandle privnh("~");

	privnh.getParam("default_forward_velocity", defaultForwardVelocity_);

	ROS_INFO("TablebotStrategy defaultForwardVelocity_: %7.3f", defaultForwardVelocity_);

	arduinoSensorsSub_ = nh_.subscribe("arduino_sensors", 1, &TablebotStrategy::handleArduinoSensors, this);
	cmdVelPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 50);

	computeAvgTofDistances();
}

bool TablebotStrategy::anyTableEdgeFound() {
	bool result = frontLeftTableEdgeFound() ||
			      frontRightTableEdgeFound() ||
			      backLeftTableEdgeFound() ||
			      backRightTableEdgeFound();
	ROS_INFO_COND(debug_, "anyTableEdgeFound result: %d", result);
	return result;
}

bool TablebotStrategy::backLeftTableEdgeFound() {
	bool result =  tofFound_ && (backLeftMm_ > (avgBackLeftMm_ + distanceToTableTopSlack_));
	ROS_INFO_COND(debug_ || result, "backLeftTableEdgeFound: %d, backLeftMm_: %6.1f, avgBackLeftMm_: %6.1f", result, backLeftMm_, avgBackLeftMm_);
	return result;
}

bool TablebotStrategy::backRightTableEdgeFound() {
	bool result =  tofFound_ && (backRightMm_ > (avgBackRightMm_ + distanceToTableTopSlack_));
	ROS_INFO_COND(debug_ || result, "backRightTableEdgeFound: %d, backRightMm_: %6.1f, avgBackRightMm_: %6.1f", result, backRightMm_, avgBackRightMm_);
	return result;
}

void TablebotStrategy::computeAvgTofDistances() {
	const int averageIterationCount = 5;
	int readingCount = 0;
	ros::Rate r(10);

	avgFrontLeftMm_ = 0.0;
	avgFrontRightMm_ = 0.0;
	avgBackLeftMm_ = 0.0;
	avgBackRightMm_ = 0.0;

	while (ros::ok() && (readingCount < averageIterationCount)) {
		if (tofFound_) {
			avgFrontLeftMm_ += frontLeftMm_;
			avgFrontRightMm_ += frontRightMm_;
			avgBackLeftMm_ += backLeftMm_;
			avgBackRightMm_ += backRightMm_;

			tofFound_ = false;
			readingCount++;
		}

		ros::spinOnce();
		r.sleep();
	}

	if (readingCount > 0) {
		avgFrontLeftMm_ = avgFrontLeftMm_ / readingCount;
		avgFrontRightMm_ = avgFrontRightMm_ / readingCount;
		avgBackLeftMm_ = avgBackLeftMm_ / readingCount;
		avgBackRightMm_ = avgBackRightMm_ / readingCount;
	}

	ROS_INFO_COND(debug_, "computeAvgTofDistances avgFrontLeftMm_: %6.1f, avgFrontRightMm_: %6.1f, avgBackLeftMm_: %6.1f, avgBackRightMm_: %6.1f",
			 avgFrontLeftMm_,
			 avgFrontRightMm_,
			 avgBackLeftMm_,
			 avgBackRightMm_
			 );
}

bool TablebotStrategy::driveUntilEdgeIsComplete() {
	if (anyTableEdgeFound()) {
		ROS_INFO_COND(debug_, "driveUntilEdgeIsComplete anyTableEdgeFound");
		return true;
	} else {
		geometry_msgs::Twist cmdVel;

		cmdVel.linear.x = defaultForwardVelocity_;
		cmdVel.angular.z = 0.0;
		cmdVelPub_.publish(cmdVel);
		ROS_INFO_COND(debug_, "driveUntilEdgeIsComplete move");
		return false;
	}
}

bool TablebotStrategy::frontLeftTableEdgeFound() {
	bool result = tofFound_ && (frontLeftMm_ > (avgFrontLeftMm_ + distanceToTableTopSlack_));
	ROS_INFO_COND(debug_ || result, "frontLeftTableEdgeFound: %d, frontLeftMm_: %6.1f, avgFrontLeftMm_: %6.1f", result, frontLeftMm_, avgFrontLeftMm_);
	return result;
}

bool TablebotStrategy::frontRightTableEdgeFound() {
	bool result =  tofFound_ && (frontRightMm_ > (avgFrontRightMm_ + distanceToTableTopSlack_));
	ROS_INFO_COND(debug_ || result, "frontRightTableEdgeFound: %d, frontRightMm_: %6.1f, avgFrontRightMm_: %6.1f", result, frontRightMm_, avgFrontRightMm_);
	return result;
}

void TablebotStrategy::handleArduinoSensors(const lysander::ArduinoSensors::ConstPtr& arduino_sensors) {
	tofFound_ = true;
	tofReadCount++;
	frontLeftMm_ = arduino_sensors->frontLeftMm;
	frontRightMm_ = arduino_sensors->frontRightMm;
	backLeftMm_ = arduino_sensors->backLeftMm;
	backRightMm_ = arduino_sensors->backRightMm;
	ROS_INFO_COND(debug_, "frontLeftMm: %6.1f, frontRightMm: %6.1f, backLeftMm: %6.1f, backRightMm: %6.1f, tofReadCount: %d",
			 arduino_sensors->frontLeftMm,
			 arduino_sensors->frontRightMm,
			 arduino_sensors->backLeftMm,
			 arduino_sensors->backRightMm,
			 tofReadCount);
}

void TablebotStrategy::solveChallenge1() {
	ros::Rate r(10);
	while (ros::ok()) {
		if (driveUntilEdgeIsComplete()) {
			ROS_INFO_COND(debug_, "driveUntilEdgeIsComplete is false, continue");
			break;
		}

		ros::spinOnce();
		r.sleep();
	}
}

void TablebotStrategy::stop() {
	geometry_msgs::Twist cmdVel;
	cmdVel.linear.x = 0.0;
	cmdVel.angular.z = 0.0;
	cmdVelPub_.publish(cmdVel);
}


