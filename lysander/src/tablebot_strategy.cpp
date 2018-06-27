#include <ros/ros.h>
#include <cstdlib>

#include <lysander/tablebot_strategy.h>
#include <geometry_msgs/Twist.h>

using namespace lysander;

TablebotStrategy::TablebotStrategy() :
	avgBackLeftMm_(0.0),
	avgBackRightMm_(0.0),
	avgFrontLeftMm_(0.0),
	avgFrontRightMm_(0.0),
	backLeftMm_(0.0),
	backRightMm_(0.0),
	lastDirection_(0),
	nextGoalIndex_(0),
	debug_(false),
	defaultForwardVelocity_(0.1),
	distanceToTableTopSlack_(10.0),
	frontLeftMm_(0.0),
	frontRightMm_(0.0),
	odomFound_(false),
	odomReadCount_(0),
	tofFound_(false),
	tofReadCount(0) {

	pushGoal(kNONE);
	ros::NodeHandle privnh("~");

	privnh.getParam("debug_", debug_);
	privnh.getParam("default_forward_velocity", defaultForwardVelocity_);
	privnh.getParam("distanceToTableTopSlack_", distanceToTableTopSlack_);

	ROS_INFO("[TablebotStrategy::TablebotStrategy] debug_: %s", debug_ ? "TRUE" : "FALSE");
	ROS_INFO("[TablebotStrategy::TablebotStrategy] defaultForwardVelocity_: %7.3f", defaultForwardVelocity_);
	ROS_INFO("[TablebotStrategy::TablebotStrategy] distanceToTableTopSlack_: %7.3f", distanceToTableTopSlack_);

	arduinoSensorsSub_ = nh_.subscribe("arduino_sensors", 1, &TablebotStrategy::handleArduinoSensors, this);
	odomSUb_ = nh_.subscribe("motor_odom", 1, &TablebotStrategy::handleOdom, this);
	cmdVelPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 50);

	computeAvgTofDistances();
}

bool TablebotStrategy::anyTableEdgeFound() {
	bool result = frontLeftTableEdgeFound() ||
			      frontRightTableEdgeFound() ||
			      backLeftTableEdgeFound() ||
			      backRightTableEdgeFound();
	ROS_INFO_COND(debug_, "[TablebotStrategy::anyTableEdgeFound] result: %d", result);
	return result;
}

bool TablebotStrategy::backLeftTableEdgeFound() {
	bool result =  tofFound_ && (backLeftMm_ > (avgBackLeftMm_ + distanceToTableTopSlack_));
	ROS_INFO_COND(debug_ || result, "[TablebotStrategy::backLeftTableEdgeFound] result: %d, backLeftMm_: %6.1f, avgBackLeftMm_: %6.1f", result, backLeftMm_, avgBackLeftMm_);
	return result;
}

bool TablebotStrategy::backRightTableEdgeFound() {
	bool result =  tofFound_ && (backRightMm_ > (avgBackRightMm_ + distanceToTableTopSlack_));
	ROS_INFO_COND(debug_ || result, "[TablebotStrategy::backRightTableEdgeFound] result: %d, backRightMm_: %6.1f, avgBackRightMm_: %6.1f", result, backRightMm_, avgBackRightMm_);
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

	ROS_INFO_COND(debug_, "[TablebotStrategy::computeAvgTofDistances] avgFrontLeftMm_: %6.1f, avgFrontRightMm_: %6.1f, avgBackLeftMm_: %6.1f, avgBackRightMm_: %6.1f",
			 avgFrontLeftMm_,
			 avgFrontRightMm_,
			 avgBackLeftMm_,
			 avgBackRightMm_
			 );
}

TablebotStrategy::GOAL TablebotStrategy::currentGoal() {
	return currentGoal_[nextGoalIndex_ - 1];
}


bool TablebotStrategy::driveUntilEdgeIsComplete() {
	if (anyTableEdgeFound()) {
		stop();
		ROS_INFO_COND(debug_, "[TablebotStrategy::driveUntilEdgeIsComplete] anyTableEdgeFound");
		return true;
	} else {
		geometry_msgs::Twist cmdVel;

		cmdVel.linear.x = defaultForwardVelocity_;
		cmdVel.angular.z = 0.0;
		lastDirection_ = 1;
		cmdVelPub_.publish(cmdVel);
		ROS_INFO_COND(debug_, "[TablebotStrategy::driveUntilEdgeIsComplete] move");
		return false;
	}
}

bool TablebotStrategy::frontLeftTableEdgeFound() {
	bool result = tofFound_ && (frontLeftMm_ > (avgFrontLeftMm_ + distanceToTableTopSlack_));
	ROS_INFO_COND(debug_ || result, "[TablebotStrategy::frontLeftTableEdgeFound] result: %d, frontLeftMm_: %6.1f, avgFrontLeftMm_: %6.1f", result, frontLeftMm_, avgFrontLeftMm_);
	return result;
}

bool TablebotStrategy::frontRightTableEdgeFound() {
	bool result =  tofFound_ && (frontRightMm_ > (avgFrontRightMm_ + distanceToTableTopSlack_));
	ROS_INFO_COND(debug_ || result, "[TablebotStrategy::frontRightTableEdgeFound] result: %d, frontRightMm_: %6.1f, avgFrontRightMm_: %6.1f", result, frontRightMm_, avgFrontRightMm_);
	return result;
}

void TablebotStrategy::handleArduinoSensors(const lysander::ArduinoSensors::ConstPtr& arduino_sensors) {
	tofFound_ = true;
	tofReadCount++;
	frontLeftMm_ = arduino_sensors->frontLeftMm;
	frontRightMm_ = arduino_sensors->frontRightMm;
	backLeftMm_ = arduino_sensors->backLeftMm;
	backRightMm_ = arduino_sensors->backRightMm;
	ROS_INFO_COND(debug_, "[TablebotStrategy::handleArduinoSensors] frontLeftMm: %6.1f, frontRightMm: %6.1f, backLeftMm: %6.1f, backRightMm: %6.1f, tofReadCount: %d",
			 arduino_sensors->frontLeftMm,
			 arduino_sensors->frontRightMm,
			 arduino_sensors->backLeftMm,
			 arduino_sensors->backRightMm,
			 tofReadCount);
}

void TablebotStrategy::handleOdom(const nav_msgs::Odometry::ConstPtr& odom) {
	lastOdom_ = *odom;
	odomFound_ = true;
	odomReadCount_++;
	ROS_INFO_COND(debug_, "[TablebotStrategy::handleOdom] odomReadCount_: %d, x: %7.3f, y: %7.3f",
				  odomReadCount_,
				  lastOdom_.pose.pose.position.x,
				  lastOdom_.pose.pose.position.y);
}

TablebotStrategy::GOAL TablebotStrategy::popGoal() {
	return currentGoal_[--nextGoalIndex_];
}


void TablebotStrategy::pushGoal(GOAL goal) {
	currentGoal_[nextGoalIndex_++] = goal;
}


void TablebotStrategy::solveChallenge1() {
	static const float kBACKUP_DISTANCE = .0254 * 3;

	ros::Rate r(10);
	nav_msgs::Odometry markedOdom;
	geometry_msgs::Twist cmdVel;

	while (ros::ok()) {
		switch (currentGoal()) {
		case kBACKUP:
			if (lastOdom_.pose.pose.position.x > (markedOdom.pose.pose.position.x - kBACKUP_DISTANCE)) {
				// Still need to backup.
				cmdVel.linear.x = - defaultForwardVelocity_;
				cmdVel.angular.z = 0.0;
				lastDirection_ = -1;
				cmdVelPub_.publish(cmdVel);
				ROS_INFO_COND(debug_ || true, 
							  "[TablebotStrategy::solveChallenge1] backing up marked x: %6.3f, current x: %6.3f, goal x: %6.3f",
							  markedOdom.pose.pose.position.x,
							  lastOdom_.pose.pose.position.x,
							  markedOdom.pose.pose.position.x - kBACKUP_DISTANCE
							  );
			} else {
				stop();
				popGoal();
				pushGoal(kSUCCESS);
			}

			break;

		case kFIND_TABLE_EDGE:
			if (! driveUntilEdgeIsComplete()) {
				// Not yet at table edge.
				ROS_INFO_COND(debug_ || true, "[TablebotStrategy::solveChallenge1] driveUntilEdgeIsComplete is false, continue");
			} else {
				// Found table edge.
				ROS_INFO_COND(debug_ || true, "[TablebotStrategy::solveChallenge1] Found table edge, current x: %6.3f", lastOdom_.pose.pose.position.x);
				popGoal();
				pushGoal(kBACKUP);
				markedOdom = lastOdom_;
			}

			break;

		case kNONE:
			if (odomFound_) {
				ROS_INFO_COND(debug_ || true, "[TablebotStrategy::solveChallenge1] start, current x: %6.3f", lastOdom_.pose.pose.position.x);
				pushGoal(kFIND_TABLE_EDGE);
			}

			break;

		case kSUCCESS:
			ROS_INFO("[TablebotStrategy::solveChallenge1] PROBLEM SOLVED");
			ROS_INFO_COND(debug_ || true, "[TablebotStrategy::solveChallenge1] end, final x: %6.3f", lastOdom_.pose.pose.position.x);
			return;
			break;

		default:
			ROS_ERROR("[TablebotStrategy::solveChallenge1] invalid goal: %d", currentGoal());
			exit(-1);
		}

		

		ros::spinOnce();
		r.sleep();
	}
}

void TablebotStrategy::stop() {
	geometry_msgs::Twist cmdVel;
	cmdVel.linear.x = lastDirection_ == 1 ? -1.0 : 1.0;
	cmdVel.angular.z = 0.0;
	cmdVelPub_.publish(cmdVel);

	cmdVel.linear.x = 0.0;
	cmdVel.angular.z = 0.0;
	cmdVelPub_.publish(cmdVel);

	lastDirection_ = 0;
}


