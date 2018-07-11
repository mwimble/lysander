// TODO:
// smallestEulerAngleBetween wrong? 321, -147 => -108
// Fix oscillation at table edge.
// When traveling in a straight line (fwd/rev), use euler to correct angle drift.
// Calibrate motors.
// Faster motors.
// Clean up motor driver.
// Motor driver to pull all constants from yaml.
// Better sense of rotation.
// Handle bot along close angle to table -- may require repositioning sensors.
// Add sonar.
// Add camera.
#include <ros/ros.h>
#include <tf/tf.h>
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
	debug_(true),
	defaultForwardVelocity_(0.1),
	defaultRotationVelocity_(0.6),
	distanceToTableTopSlack_(16.0),
	frontLeftMm_(0.0),
	frontRightMm_(0.0),
	nextEdgePoseIndex_(0),
	nextGoalIndex_(0),
	odomFound_(false),
	odomReadCount_(0),
	tofFound_(false),
	tofReadCount(0) {

	pushGoal(kNONE);
	ros::NodeHandle privnh("~");

	privnh.getParam("debug", debug_);
	privnh.getParam("default_forward_velocity", defaultForwardVelocity_);
	privnh.getParam("default_rotation_velocity", defaultRotationVelocity_);
	privnh.getParam("distanceToTableTopSlack_", distanceToTableTopSlack_);

	ROS_INFO("[TablebotStrategy::TablebotStrategy] debug: %s", debug_ ? "TRUE" : "FALSE");
	ROS_INFO("[TablebotStrategy::TablebotStrategy] default_forward_velocity: %7.3f", defaultForwardVelocity_);
	ROS_INFO("[TablebotStrategy::TablebotStrategy] default_rotation_velocity: %7.3f", defaultRotationVelocity_);
	ROS_INFO("[TablebotStrategy::TablebotStrategy] distanceToTableTopSlack_: %7.3f", distanceToTableTopSlack_);

	arduinoSensorsSub_ = nh_.subscribe("arduino_sensors", 1, &TablebotStrategy::handleArduinoSensors, this);
	odomSUb_ = nh_.subscribe("motor_odom", 1, &TablebotStrategy::handleOdom, this);
	cmdVelPub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 50);

	computeAvgTofDistances();
}

bool TablebotStrategy::anyEdgeFound() {
	bool result = frontLeftEdgeFound() ||
			      frontRightEdgeFound() ||
			      backLeftEdgeFound() ||
			      backRightEdgeFound();
	ROS_INFO_COND(debug_, "[TablebotStrategy::anyEdgeFound] result: %d", result);
	return result;
}

bool TablebotStrategy::bothFrontSensorsFoundEdge() {
	bool result = frontLeftEdgeFound() &&
			      frontRightEdgeFound();
	ROS_INFO_COND(debug_, "[TablebotStrategy::bothFrontSensorsFoundEdge] result: %d", result);
	return result;
}


bool TablebotStrategy::backLeftEdgeFound() {
	bool result =  tofFound_ && (backLeftMm_ > (avgBackLeftMm_ + distanceToTableTopSlack_));
	ROS_INFO_COND(debug_ || result, "[TablebotStrategy::backLeftEdgeFound] result: %d, backLeftMm_: %6.1f, avgBackLeftMm_: %6.1f", result, backLeftMm_, avgBackLeftMm_);
	return result;
}

bool TablebotStrategy::backRightEdgeFound() {
	bool result =  tofFound_ && (backRightMm_ > (avgBackRightMm_ + distanceToTableTopSlack_));
	ROS_INFO_COND(debug_ || result, "[TablebotStrategy::backRightEdgeFound] result: %d, backRightMm_: %6.1f, avgBackRightMm_: %6.1f", result, backRightMm_, avgBackRightMm_);
	return result;
}

bool TablebotStrategy::closeToDesiredPose(SimplePose desiredPose) {
	static const float closeDeltaX = 0.0254 / 2.0;
	static const float closeDeltaY = 0.0254 / 2.0;
	float deltaX = fabs(desiredPose.x_ - lastOdom_.pose.pose.position.x);
	float deltaY = fabs(desiredPose.y_ - lastOdom_.pose.pose.position.y);
	return (deltaX < closeDeltaX) && (deltaY < closeDeltaY);
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

float TablebotStrategy::constrainEulerAngle(float x) {
    x = fmod(x,360);
    if (x < 0) x += 360;

    return x;
}

TablebotStrategy::GOAL TablebotStrategy::currentGoal() {
	return currentGoal_[nextGoalIndex_ - 1];
}

SimplePose TablebotStrategy::currentMarkedPose() {
	return markedPose_[nextGoalIndex_ -1];
}


SimplePose TablebotStrategy::currentPose() {
	// tf::Quaternion q(lastOdom_.pose.pose.orientation.x,
	// 				 lastOdom_.pose.pose.orientation.y,
	// 				 lastOdom_.pose.pose.orientation.z,
	// 				 lastOdom_.pose.pose.orientation.w);
	// tf::Matrix3x3 m(q);
	// double roll, pitch, yaw;
	// m.getRPY(roll, pitch, yaw);
	static const float kRADIANS_TO_DEGREES = 57.2958;
	float yaw = lastEulers_.x_ / kRADIANS_TO_DEGREES;

	ROS_INFO_COND(debug_, 
				  "[TablebotStrategy::currentPose] x: %6.3f, y: %6.3f, eurler: %6.3f"
				  ", q: %6.3f, %6.3f, %6.3f, %6.3f",
			      lastOdom_.pose.pose.position.x,
			      lastOdom_.pose.pose.position.y,
			      yaw * 57.2958,
				  lastOdom_.pose.pose.orientation.x,
				  lastOdom_.pose.pose.orientation.y,
				  lastOdom_.pose.pose.orientation.z,
				  lastOdom_.pose.pose.orientation.w);
	return SimplePose(lastOdom_.pose.pose.position.x,
					  lastOdom_.pose.pose.position.y,
					  yaw * 57.2958,
					  tf::Quaternion(lastOdom_.pose.pose.orientation.x,
					  				 lastOdom_.pose.pose.orientation.y,
					  				 lastOdom_.pose.pose.orientation.z,
					  				 lastOdom_.pose.pose.orientation.w));
}


bool TablebotStrategy::frontLeftEdgeFound() {
	bool result = tofFound_ && (frontLeftMm_ > (avgFrontLeftMm_ + distanceToTableTopSlack_));
	ROS_INFO_COND(debug_ || result, "[TablebotStrategy::frontLeftEdgeFound] result: %d, frontLeftMm_: %6.1f, avgFrontLeftMm_: %6.1f", result, frontLeftMm_, avgFrontLeftMm_);
	return result;
}

bool TablebotStrategy::frontRightEdgeFound() {
	bool result =  tofFound_ && (frontRightMm_ > (avgFrontRightMm_ + distanceToTableTopSlack_));
	ROS_INFO_COND(debug_ || result, "[TablebotStrategy::frontRightEdgeFound] result: %d, frontRightMm_: %6.1f, avgFrontRightMm_: %6.1f", result, frontRightMm_, avgFrontRightMm_);
	return result;
}

const char* TablebotStrategy::goalName(GOAL goal) {
	switch (goal) {
	case kBACKUP_AND_ROTATE_180: return "kBACKUP_AND_ROTATE_180";
	case kFIND_FAR_EDGE: return "kFIND_FAR_EDGE";
	case kFIND_NEAR_EDGE: return "kFIND_NEAR_EDGE";
	case kNONE: return "kNONE";
	case kROTATE_180: return "kROTATE_180";
	case kROTATE_CLOCKWISE_A_BIT: return "kROTATE_CLOCKWISE_A_BIT";
	case kROTATE_COUNTERCLOCKWISE_A_BIT: return "kROTATE_COUNTERCLOCKWISE_A_BIT";
	case kSUCCESS: return "kSUCCESS";
	case kVICTORY_DANCE: return "kVICTORY_DANCE";
	case kVICTORY_DANCE_LEFT: return "kVICTORY_DANCE_LEFT";
	case kVICTORY_DANCE_RIGHT: return "kVICTORY_DANCE_RIGHT";
	case kWIGGLE_LEFT_A_BIT: return "kWIGGLE_LEFT_A_BIT";
	case kWIGGLE_RIGHT_A_BIT: return "kWIGGLE_RIGHT_A_BIT";
	default: return "<<UNKNOWN GOAL>>";
	}
}

SimplePose TablebotStrategy::goalPose(SimplePose originalPose, float changeEuler, float distance) {
	tf::Quaternion q = originalPose.q_;
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	static const float degreesToRadians = 0.017453292519943;
	
	float changeX = cos(yaw + (changeEuler * degreesToRadians)) * distance;
	float changeY = sin(yaw + (changeEuler * degreesToRadians)) * distance;
	ROS_INFO_COND(debug_,
				  "[TablebotStrategy::goalPose] original x: %6.3f, y: %6.3f, euler: %6.3f"
				  ", rpy: %6.3f, %6.3f, %6.3f"
				  ", changeEuler: %6.3f, distance: %6.3f"
				  ", changeX: %6.3f, changeY: %6.3f",
				  originalPose.x_,
				  originalPose.y_,
				  originalPose.euler_,
				  roll,
				  pitch,
				  yaw,
				  changeEuler,
				  distance,
				  changeX,
				  changeY);

	return SimplePose(originalPose.x_ + changeX,
					  originalPose.y_ + changeY,
					  constrainEulerAngle(originalPose.euler_ + changeEuler),
					  q);
}


void TablebotStrategy::handleArduinoSensors(const lysander::ArduinoSensors::ConstPtr& arduino_sensors) {
	tf::Quaternion q = tf::Quaternion(lastOdom_.pose.pose.orientation.x,
					 lastOdom_.pose.pose.orientation.y,
					 lastOdom_.pose.pose.orientation.z,
					 lastOdom_.pose.pose.orientation.w);
	tofFound_ = true;
	tofReadCount++;
	frontLeftMm_ = arduino_sensors->frontLeftMm;
	frontRightMm_ = arduino_sensors->frontRightMm;
	backLeftMm_ = arduino_sensors->backLeftMm;
	backRightMm_ = arduino_sensors->backRightMm;
	lastEulers_ = Eulers(arduino_sensors->euler_x,
						 arduino_sensors->euler_y,
						 arduino_sensors->euler_z,
						 q);
	ROS_INFO_COND(debug_, "[TablebotStrategy::handleArduinoSensors] frontLeftMm: %6.1f, frontRightMm: %6.1f, backLeftMm: %6.1f, backRightMm: %6.1f, tofReadCount: %d, x: %6.3f, y: %6.3f, z: %6.3f",
			 arduino_sensors->frontLeftMm,
			 arduino_sensors->frontRightMm,
			 arduino_sensors->backLeftMm,
			 arduino_sensors->backRightMm,
			 tofReadCount,
			 arduino_sensors->euler_x,
			 arduino_sensors->euler_y,
			 arduino_sensors->euler_z);
}


void TablebotStrategy::handleOdom(const nav_msgs::Odometry::ConstPtr& odom) {
	lastOdom_ = *odom;
	odomFound_ = true;
	odomReadCount_++;

	tf::Quaternion q(lastOdom_.pose.pose.orientation.x,
					 lastOdom_.pose.pose.orientation.y,
					 lastOdom_.pose.pose.orientation.z,
					 lastOdom_.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
 	ROS_INFO_COND(debug_, 
				  "[TablebotStrategy::handleOdom] odomReadCount_: %d, x: %7.3f, y: %7.3f"
				  ", q: %6.3f, %6.3f, %6.3f, %6.3f"
				  ", rpy: %6.3f, %6.3f, %6.3f",
				  odomReadCount_,
				  lastOdom_.pose.pose.position.x,
				  lastOdom_.pose.pose.position.y,
				  lastOdom_.pose.pose.orientation.x,
				  lastOdom_.pose.pose.orientation.y,
				  lastOdom_.pose.pose.orientation.z,
				  lastOdom_.pose.pose.orientation.w,
				  roll, pitch, yaw);
}


TablebotStrategy::GOAL TablebotStrategy::popGoal() {
	return currentGoal_[--nextGoalIndex_];
}


void TablebotStrategy::pushEdgePose(SimplePose pose) {
	edgePose_[nextEdgePoseIndex_++] = pose;
}


void TablebotStrategy::pushGoal(GOAL goal) {
	markedPose_[nextGoalIndex_] = currentPose();
	currentGoal_[nextGoalIndex_++] = goal;
}


float TablebotStrategy::smallestEulerAngleBetween(float a1, float a2) {
	//float result = 180.0 - abs(abs(a1 - a2) - 180.0);
	float diff = a1 -a2;
	if (diff > 180) diff = diff - 360.0;
	if (diff < -180) diff = diff + 360.0;

	return fabs(diff);
}

void TablebotStrategy::solveChallenge1() {
	static const float kBACKUP_DISTANCE = .0254 * 7;

	SimplePose desiredPose;
	ros::Rate r(10);
	nav_msgs::Odometry markedOdom;
	geometry_msgs::Twist cmdVel;

	// pushGoal(kSUCCESS);
	// pushGoal(kROTATE_180);

	while (ros::ok()) {
		ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] ---- ---- Goal: %s", goalName(currentGoal()));
		switch (currentGoal()) {
		case kBACKUP_AND_ROTATE_180:
			desiredPose = goalPose(currentMarkedPose(), 180, 0.0254 * 6);
			ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] kBACKUP_AND_ROTATE_180 desired x: %6.3f, desired y: %6.3f, desired euler: %6.3f",
						  desiredPose.x_, desiredPose.y_, desiredPose.euler_);
			if (! closeToDesiredPose(desiredPose)) {
				// Still need to backup.
				cmdVel.linear.x = - defaultForwardVelocity_;
				cmdVel.angular.z = 0.0;
				cmdVelPub_.publish(cmdVel);
				ROS_INFO_COND(debug_, 
							  "[TablebotStrategy::solveChallenge1] backing up, marked x: %6.3f, current x: %6.3f, goal x: %6.3f",
							  currentMarkedPose().x_,
							  lastOdom_.pose.pose.position.x,
							  desiredPose.x_
							  );
			} else {
				ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] kBACKUP_AND_ROTATE_180 backup portion success");
				stop();
				popGoal();
				pushGoal(kROTATE_180); // Use marked Pose for rotation
			}

			break;

		case kFIND_FAR_EDGE:
			if (!findEdgeShouldContinue()) {
				ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] kFIND_FAR_EDGE successful");
				pushGoal(kFIND_NEAR_EDGE);
				pushGoal(kBACKUP_AND_ROTATE_180);
			}

			break;

		case kFIND_NEAR_EDGE:
			if (!findEdgeShouldContinue()) {
				ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] kFIND_NEAR_EDGE successful");
				pushGoal(kVICTORY_DANCE);
				pushGoal(kBACKUP_AND_ROTATE_180);
			}


			break;
		case kNONE:
			if (odomFound_) {
				ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] start, current x: %6.3f", lastOdom_.pose.pose.position.x);
				pushGoal(kFIND_FAR_EDGE);
			}

			break;

		case kROTATE_180:
			if (rotationComplete(&TablebotStrategy::stillNeedToRotate180, kCOUNTERCLOCKWISE)) {
				ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] kROTATE_180 completed rotation");
				stop();
				popGoal();
			}

			break;

		case kROTATE_CLOCKWISE_A_BIT:
			// Rotate until front-left sensor now longer sees the edge, or both sensors see the edge.
			// Assumes a marked pose was pushed.
			if (rotationComplete(&TablebotStrategy::stillNeedToRotateAwayFromFrontRightSensor, kCLOCKWISE)) {
				ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] kROTATE_CLOCKWISE_A_BIT completed rotation");
				stop();
				popGoal();
			}

			break;

		case kROTATE_COUNTERCLOCKWISE_A_BIT:
			// Rotate until front-right sensor now longer sees the edge, or both sensors see the edge.
			// Assumes a marked pose was pushed.
			if (rotationComplete(&TablebotStrategy::stillNeedToRotateAwayFromFrontLeftSensor, kCOUNTERCLOCKWISE)) {
				ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] kROTATE_COUNTERCLOCKWISE_A_BIT completed rotation");
				stop();
				popGoal();
			}

			break;

		case kSUCCESS:
			ROS_INFO("[TablebotStrategy::solveChallenge1] PROBLEM SOLVED");
			ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] end, final x: %6.3f", lastOdom_.pose.pose.position.x);
			return;
			break;

		case kVICTORY_DANCE:
			popGoal();
			pushGoal(kSUCCESS);
			pushGoal(kVICTORY_DANCE_RIGHT);
			pushGoal(kVICTORY_DANCE_LEFT);
			pushGoal(kVICTORY_DANCE_RIGHT);
			pushGoal(kVICTORY_DANCE_LEFT);
			pushGoal(kVICTORY_DANCE_RIGHT);
			pushGoal(kVICTORY_DANCE_LEFT);
			pushGoal(kVICTORY_DANCE_RIGHT);
			pushGoal(kVICTORY_DANCE_LEFT);
			break;

		case kVICTORY_DANCE_LEFT:
			if (rotationComplete(&TablebotStrategy::stillNeedToDamceLeft, kCOUNTERCLOCKWISE)) {
				ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] kVICTORY_DANCE_LEFT completed rotation");
				stop();
				popGoal();
			}

			break;

		case kVICTORY_DANCE_RIGHT:
			if (rotationComplete(&TablebotStrategy::stillNeedToDamceRight, kCLOCKWISE)) {
				ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] kVICTORY_DANCE_RIGHT completed rotation");
				stop();
				popGoal();
			}

			break;

		default:
			ROS_ERROR("[TablebotStrategy::solveChallenge1] invalid goal: %d", currentGoal());
			exit(-1);
		}

		

		ros::spinOnce();
		r.sleep();
	}
}


bool TablebotStrategy::findEdgeShouldContinue() {
		geometry_msgs::Twist cmdVel;

		// Looking for table edge.
		if (anyEdgeFound()) {
			stop();	// Brake so robot won't keep moving before further actions take place.
			ROS_INFO_COND(debug_, "[TablebotStrategy::findEdgeShouldContinue] anyEdgeFound");
			if (bothFrontSensorsFoundEdge()) {
				// Success.
				stop();
				popGoal();
				pushEdgePose(currentPose()); // Keep list of poses for edges of table.
				return false;
			} else {
				if (frontLeftEdgeFound()) {
					// Need to rotate counterclockwise a bit so sensor isn't hanging over the edge.
					ROS_INFO_COND(debug_, "[TablebotStrategy::findEdgeShouldContinue] front left edge found, need to rotate counterclockwise a bit");
					pushGoal(kROTATE_COUNTERCLOCKWISE_A_BIT);
				} else if (frontRightEdgeFound()) {
					// Need to rotate clockwise a bit so sensor isn't hanging over the edge.
					ROS_INFO_COND(debug_, "[TablebotStrategy::findEdgeShouldContinue] front right edge found, need to rotate clockwise a bit");
					pushGoal(kROTATE_CLOCKWISE_A_BIT);
				} else {
					// UNIMPLEMENTED.
					ROS_ERROR("[TablebotStrategy::findEdgeShouldContinue] invalid combination of edge sensor readings");
					//exit(-1);
					cmdVel.linear.x = defaultForwardVelocity_;
					cmdVel.angular.z = 0.0;
					cmdVelPub_.publish(cmdVel);
				}

				return true;
			}
		} else {
			// No edge found, continue to far edge.
			geometry_msgs::Twist cmdVel;

			cmdVel.linear.x = defaultForwardVelocity_;
			cmdVel.angular.z = 0.0;
			cmdVelPub_.publish(cmdVel);
			ROS_INFO_COND(debug_, "[TablebotStrategy::findEdgeShouldContinue] continue towards edge");
			return true;
		}

}


bool TablebotStrategy::rotationComplete(ContinueTestFn continueTestFn, ROTATE_DIRECTION rotateDirection) {
	if ((this->*continueTestFn)()) {
		geometry_msgs::Twist cmdVel;
	
		cmdVel.linear.x = 0;
		cmdVel.angular.z = rotateDirection == kCLOCKWISE ? -defaultRotationVelocity_ : defaultRotationVelocity_;
		cmdVelPub_.publish(cmdVel);
		ROS_INFO_COND(debug_, "[TablebotStrategy::rotationComplete] rotating");
		return false;
	} else {
		ROS_INFO_COND(debug_, "[TablebotStrategy::rotationComplete] complete");
		return true;
	}
}

static const float danceYawDegrees = 5;

bool TablebotStrategy::stillNeedToDamceLeft() {
	float eulerGoal = currentMarkedPose().euler_ - danceYawDegrees;
	float eulerDiff = smallestEulerAngleBetween(lastEulers_.x_, eulerGoal);
	bool done = abs(eulerDiff) < 5.0;
	ROS_INFO_COND(debug_,
				  "[TablebotStrategy::stillNeedToDamceLeft]"
				  " currentMarkedPose().euler_: %6.3f"
				  ", eulerGoal: %6.3f"
				  ", eulerDiff: %6.3f"
				  ", done: %d",
				  currentMarkedPose().euler_,
				  eulerGoal,
				  eulerDiff,
				  done);
	return !done;	
}


bool TablebotStrategy::stillNeedToDamceRight() {
	float eulerGoal = currentMarkedPose().euler_ + danceYawDegrees;
	float eulerDiff = smallestEulerAngleBetween(lastEulers_.x_, eulerGoal);
	bool done = abs(eulerDiff) < 5.0;
	ROS_INFO_COND(debug_,
				  "[TablebotStrategy::stillNeedToDamceRight]"
				  " currentMarkedPose().euler_: %6.3f"
				  ", eulerGoal: %6.3f"
				  ", eulerDiff: %6.3f"
				  ", done: %d",
				  currentMarkedPose().euler_,
				  eulerGoal,
				  eulerDiff,
				  done);
	return !done;	
}


bool TablebotStrategy::stillNeedToRotateAwayFromFrontLeftSensor() {
	return frontLeftEdgeFound() && !frontRightEdgeFound();
}


bool TablebotStrategy::stillNeedToRotateAwayFromFrontRightSensor() {
	return frontRightEdgeFound() && !frontLeftEdgeFound();
}


bool TablebotStrategy::stillNeedToRotate180() {
	float eulerGoal = fabs(180 + currentMarkedPose().euler_);
	float eulerDiff = smallestEulerAngleBetween(lastEulers_.x_, eulerGoal);
	bool done = abs(eulerDiff) < 5.0;
	ROS_INFO_COND(debug_,
				  "[TablebotStrategy::stillNeedToRotate180]"
				  " currentMarkedPose().euler_: %6.3f"
				  ", eulerGoal: %6.3f"
				  ", eulerDiff: %6.3f"
				  ", done: %d",
				  currentMarkedPose().euler_,
				  eulerGoal,
				  eulerDiff,
				  done);
	return !done;	
}


void TablebotStrategy::stop() {
	geometry_msgs::Twist cmdVel;
	cmdVel.linear.x = -1;
	cmdVel.angular.z = 0.0;
	cmdVelPub_.publish(cmdVel);
	cmdVelPub_.publish(cmdVel);

	cmdVel.linear.x = 0.0;
	cmdVel.angular.z = 0.0;
	cmdVelPub_.publish(cmdVel);
}


