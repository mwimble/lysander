// TODO:
// Calibrate motors.
// Faster motors.
// Clean up motor driver.
// Motor driver to pull all constants from yaml.
// Better sense of rotation.
// Push information for goal, like marked orientation, etc. so when goal popped, don't have to synchronize other stacks.
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
	defaultRotationVelocity_(0.4),
	distanceToTableTopSlack_(16.0),
	frontLeftMm_(0.0),
	frontRightMm_(0.0),
	lastDirection_(0),
	nextEdgePoseIndex_(0),
	nextGoalIndex_(0),
	nextMarkedPoseIndex_(0),
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

float TablebotStrategy::constrainEulerAngle(float x) {
    x = fmod(x,360);
    if (x < 0) x += 360;

    return x;
}

TablebotStrategy::GOAL TablebotStrategy::currentGoal() {
	return currentGoal_[nextGoalIndex_ - 1];
}

SimplePose TablebotStrategy::currentMarkedPose() {
	return markedPose_[nextMarkedPoseIndex_ -1];
}


SimplePose TablebotStrategy::currentPose() {
	tf::Quaternion q(lastOdom_.pose.pose.orientation.x,
					 lastOdom_.pose.pose.orientation.y,
					 lastOdom_.pose.pose.orientation.z,
					 lastOdom_.pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	ROS_INFO_COND(debug_, "[TablebotStrategy::currentPose] x: %6.3f, y: %6.3f, eurler: %6.3f",
			      lastOdom_.pose.pose.position.x,
			      lastOdom_.pose.pose.position.y,
			      yaw * 57.2958);
	return SimplePose(lastOdom_.pose.pose.position.x,
					  lastOdom_.pose.pose.position.y,
					  yaw * 57.2958);
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
	case kWIGGLE_LEFT_A_BIT: return "kWIGGLE_LEFT_A_BIT";
	case kWIGGLE_RIGHT_A_BIT: return "kWIGGLE_RIGHT_A_BIT";
	default: return "<<UNKNOWN GOAL>>";
	}
}

SimplePose TablebotStrategy::goalPose(SimplePose originalPose, float changeEuler, float distance) {
	float changeX = cos((originalPose.euler_ + changeEuler) * 0.0174533) * distance;
	float changeY = sin((originalPose.euler_ + changeEuler) * 0.0174533) * distance;
	// float partA = sin(originalPose.euler_ + changeEuler);
	// float partB = partA * distance;
	// float partC = originalPose.y_ + partB;
	// ROS_INFO_COND(debug_, "partA: %6.3f, partB: %6.3f, partC: %6.3f", partA, partB, partC);
	ROS_INFO_COND(debug_,
				  "[TablebotStrategy::goalPose] original x: %6.3f, y: %6.3f, euler: %6.3f"
				  ", changeEuler: %6.3f, distance: %6.3f"
				  ", changeX: %6.3f, changeY: %6.3f",
				  originalPose.x_,
				  originalPose.y_,
				  originalPose.euler_,
				  changeEuler,
				  distance,
				  changeX,
				  changeY);

	return SimplePose(originalPose.x_ + changeX,
					  originalPose.y_ + changeY,
					  constrainEulerAngle(originalPose.euler_ + changeEuler));
}


void TablebotStrategy::handleArduinoSensors(const lysander::ArduinoSensors::ConstPtr& arduino_sensors) {
	tofFound_ = true;
	tofReadCount++;
	frontLeftMm_ = arduino_sensors->frontLeftMm;
	frontRightMm_ = arduino_sensors->frontRightMm;
	backLeftMm_ = arduino_sensors->backLeftMm;
	backRightMm_ = arduino_sensors->backRightMm;
	lastEulers_ = Eulers(currentPose().euler_ /* arduino_sensors->euler_x */,
						 arduino_sensors->euler_y,
						 arduino_sensors->euler_z);
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
	ROS_INFO_COND(debug_, "[TablebotStrategy::handleOdom] odomReadCount_: %d, x: %7.3f, y: %7.3f",
				  odomReadCount_,
				  lastOdom_.pose.pose.position.x,
				  lastOdom_.pose.pose.position.y);
}

TablebotStrategy::GOAL TablebotStrategy::popGoal() {
	return currentGoal_[--nextGoalIndex_];
}

SimplePose TablebotStrategy::popMarkedPose() {
	return markedPose_[--nextMarkedPoseIndex_];
}


void TablebotStrategy::pushEdgePose(SimplePose pose) {
	edgePose_[nextEdgePoseIndex_++] = pose;
}


void TablebotStrategy::pushGoal(GOAL goal) {
	currentGoal_[nextGoalIndex_++] = goal;
}


void TablebotStrategy::pushMarkedPose(SimplePose pose) {
	markedPose_[nextMarkedPoseIndex_++] = pose;
}


float TablebotStrategy::smallestEulerAngleBetween(float a1, float a2) {
	float result = 180.0 - abs(abs(a1 - a2) - 180.0);
	return result;
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
			desiredPose = goalPose(currentMarkedPose(), 180, 0.0254 * 5);
			ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] kBACKUP_AND_ROTATE_180 desired x: %6.3f, desired y: %6.3f, desired euler: %6.3f",
						  desiredPose.x_, desiredPose.y_, desiredPose.euler_);
			if (! closeToDesiredPose(desiredPose)) {
				// Still need to backup.
				cmdVel.linear.x = - defaultForwardVelocity_;
				cmdVel.angular.z = 0.0;
				lastDirection_ = -1;
				cmdVelPub_.publish(cmdVel);
				ROS_INFO_COND(debug_, 
							  "[TablebotStrategy::solveChallenge1] backing up, marked x: %6.3f, current x: %6.3f, goal x: %6.3f",
							  currentMarkedPose().x_,
							  lastOdom_.pose.pose.position.x,
							  desiredPose.x_
							  );
			} else {
				stop();
				popGoal();
				pushGoal(kROTATE_180); // Use marked Pose for rotation
			}

			break;

		case kFIND_FAR_EDGE:
			// Looking for far table edge.
			if (anyEdgeFound()) {
				stop();
				ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] kFIND_FAR_EDGE anyEdgeFound");
				if (bothFrontSensorsFoundEdge()) {
					// Success.
					stop();
					popGoal();
					pushGoal(kFIND_NEAR_EDGE);
					pushEdgePose(currentPose());
					pushMarkedPose(currentPose());
					pushGoal(kBACKUP_AND_ROTATE_180);
				} else {
					if (frontLeftEdgeFound()) {
						// Need to rotate clockwise a bit so sensor isn't hanging over the edge.
						pushMarkedPose(currentPose());
						pushGoal(kROTATE_COUNTERCLOCKWISE_A_BIT);
					} else if (frontRightEdgeFound()) {
						// Need to rotate counterclockwise a bit so sensor isn't hanging over the edge.
						pushMarkedPose(currentPose());
						pushGoal(kROTATE_CLOCKWISE_A_BIT);
					} else {
						// UNIMPLEMENTED.
						ROS_ERROR("[TablebotStrategy::solveChallenge1] kFIND_FAR_EDGE invalid combination of edge sensor readings");
						//exit(-1);
						cmdVel.linear.x = defaultForwardVelocity_;
						cmdVel.angular.z = 0.0;
						lastDirection_ = 1;
						cmdVelPub_.publish(cmdVel);
					}
				}
			} else {
				// No edge found, continue to far edge.
				geometry_msgs::Twist cmdVel;

				cmdVel.linear.x = defaultForwardVelocity_;
				cmdVel.angular.z = 0.0;
				lastDirection_ = 1;
				cmdVelPub_.publish(cmdVel);
				ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] continue towards far edge");
			}

			break;

		case kFIND_NEAR_EDGE:
			// Looking for near table edge.
			if (anyEdgeFound()) {
				stop();
				ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] kFIND_NEAR_EDGE anyEdgeFound");
				if (bothFrontSensorsFoundEdge()) {
					// Success.
					stop();
					popGoal();
					pushGoal(kVICTORY_DANCE);
					pushEdgePose(currentPose());
					pushMarkedPose(currentPose());
					pushGoal(kBACKUP_AND_ROTATE_180);
				} else {
					if (frontLeftEdgeFound()) {
						// Need to rotate clockwise a bit so sensor isn't hanging over the edge.
						pushMarkedPose(currentPose());
						pushGoal(kROTATE_COUNTERCLOCKWISE_A_BIT);
					} else if (frontRightEdgeFound()) {
						// Need to rotate counterclockwise a bit so sensor isn't hanging over the edge.
						pushMarkedPose(currentPose());
						pushGoal(kROTATE_CLOCKWISE_A_BIT);
					} else {
						// UNIMPLEMENTED.
						ROS_ERROR("[TablebotStrategy::solveChallenge1] kFIND_NEAR_EDGE invalid combination of edge sensor readings");
						//exit(-1);
						cmdVel.linear.x = defaultForwardVelocity_;
						cmdVel.angular.z = 0.0;
						lastDirection_ = 1;
						cmdVelPub_.publish(cmdVel);
					}
				}
			} else {
				// No edge found, continue to near edge.
				cmdVel.linear.x = defaultForwardVelocity_;
				cmdVel.angular.z = 0.0;
				lastDirection_ = 1;
				cmdVelPub_.publish(cmdVel);
				ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] continue towards near edge");
			}

			break;

		case kNONE:
			if (odomFound_) {
				ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] start, current x: %6.3f", lastOdom_.pose.pose.position.x);
				pushGoal(kFIND_FAR_EDGE);
			}

			break;

		case kROTATE_180:
			{
				float eulerGoal = 180 - abs(currentMarkedPose().euler_);
				float eulerDiff = smallestEulerAngleBetween(lastEulers_.x_, eulerGoal);
				bool done = abs(eulerDiff) < 5.0;
				ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] kROTATE_180] current euler: %6.3f, marked: %6.3f, , goal: %6.3f, diff: %6.3f, done: %d",
							  lastEulers_.x_,
							  currentMarkedPose().x_,
							  eulerGoal,
							  eulerDiff,
							  done);
				if (!done) {
					// Still need to rotate.
					cmdVel.linear.x = 0;
					cmdVel.angular.z = defaultRotationVelocity_;
					lastDirection_ = 0;
					cmdVelPub_.publish(cmdVel);
					ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] kROTATE_180 rotating");
				} else {
					ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] kROTATE_180 completed rotation");
					stop();
					popGoal();
					popMarkedPose();
				}
			}

			break;

		case kROTATE_CLOCKWISE_A_BIT:
			// Rotate until front-left sensor now longer sees the edge, or both sensors see the edge.
			// Assumes a marked pose was pushed.
			if (frontRightEdgeFound() && !frontLeftEdgeFound()) {
				// Still need to rotate.
				cmdVel.linear.x = 0;
				cmdVel.angular.z = -defaultRotationVelocity_;
				lastDirection_ = 0;
				cmdVelPub_.publish(cmdVel);
				ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] rotate clockwise a bit");
			} else {
				popMarkedPose();
				stop();
				popGoal();
			}

			break;

		case kROTATE_COUNTERCLOCKWISE_A_BIT:
			// Rotate until front-right sensor now longer sees the edge, or both sensors see the edge.
			// Assumes a marked pose was pushed.
			if (frontLeftEdgeFound() && !frontRightEdgeFound()) {
				// Still need to rotate.
				cmdVel.linear.x = 0;
				cmdVel.angular.z = defaultRotationVelocity_;
				lastDirection_ = 0;
				cmdVelPub_.publish(cmdVel);
				ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] rotate counterclockwise a bit");
			} else {
				popMarkedPose();
				stop();
				popGoal();
			}

			break;

		case kSUCCESS:
			ROS_INFO("[TablebotStrategy::solveChallenge1] PROBLEM SOLVED");
			ROS_INFO_COND(debug_, "[TablebotStrategy::solveChallenge1] end, final x: %6.3f", lastOdom_.pose.pose.position.x);
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


