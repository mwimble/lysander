#include <ros/ros.h>

#include <lysander/ArduinoSensors.h>
#include <nav_msgs/Odometry.h>

namespace lysander {

	// Arduino sensors data.
	class Eulers {
	public:
		float x_;
		float y_;
		float z_;
		Eulers() {
			x_ = 0;
			y_ = 0;
			z_ = 0;
		}

		Eulers(float x, float y, float z) {
			x_ = x;
			y_ = y;
			z_ = z;
		}

		Eulers(const Eulers& other) {
			x_ = other.x_;
			y_ = other.y_;
			z_ = other.z_;
		}
	};

 	class SimplePose {
	public:
		float x_;
		float y_;
		float euler_;
		SimplePose() {
			x_ = 0;
			y_ = 0;
			euler_ = 0;
		}

		SimplePose(float x, float y, float euler) {
			x_ = x;
			y_ = y;
			euler_ = euler;
		}

		SimplePose(const SimplePose& other) {
			x_ = other.x_;
			y_ = other.y_;
			euler_ = other.euler_;
		}
	};

 	class TablebotStrategy {
	public:

		TablebotStrategy();

		void solveChallenge1();

	private:
		// Templete functions.
		typedef bool (TablebotStrategy::*ContinueTestFn)();
		typedef enum ROTATE_DIRECTION {
			kCLOCKWISE,
			kCOUNTERCLOCKWISE
		} ROTATE_DIRECTION;

		bool stillNeedToDamceLeft();
		bool stillNeedToDamceRight();
		bool stillNeedToRotate180();
		bool stillNeedToRotateAwayFromFrontLeftSensor();
		bool stillNeedToRotateAwayFromFrontRightSensor();

		// Move towards edge, if not there.
		// RETURN true => edge found, no further movement required, false => edge not found,
		//				  further calls needed to make progress.
		bool findEdgeShouldContinue();

		// Rotate if needed.
		// RETURN true => rotation is complete, no further movement required,
		//		  false => further calls needed to make progress.
		bool rotationComplete(ContinueTestFn continueTestFn, ROTATE_DIRECTION rotateDirection);

		Eulers lastEulers_;


		// List of poses detcted for edges of the table.
		static const int MAX_EDGE_POSE_DEPTH = 32; // Maximum depth of edgePose_ list.
		SimplePose edgePose_[MAX_EDGE_POSE_DEPTH];
		int nextEdgePoseIndex_;

		// Goals.
		typedef enum GOAL {
			kBACKUP_AND_ROTATE_180,
			kFIND_FAR_EDGE,
			kFIND_NEAR_EDGE,
			kNONE,
			kROTATE_180,
			kROTATE_CLOCKWISE_A_BIT,
			kROTATE_COUNTERCLOCKWISE_A_BIT,
			kSUCCESS,
			kVICTORY_DANCE,
			kVICTORY_DANCE_LEFT,
			kVICTORY_DANCE_RIGHT,
			kWIGGLE_LEFT_A_BIT,
			kWIGGLE_RIGHT_A_BIT
		} GOAL;

		// Goal stack and markedPose stack are in sync.
		static const int MAX_GOAL_DEPTH = 20;	// Maximum depth of currentGoal_ stack.
		GOAL currentGoal_[MAX_GOAL_DEPTH];		// Goal stack;
		int nextGoalIndex_;						// Index to place next goal in stack.
		SimplePose markedPose_[MAX_GOAL_DEPTH];

		// Parameters.
		bool debug_; // Generate vebose debug output.
		float distanceToTableTopSlack_; // Distance to back up from edge before rotating.
		float defaultForwardVelocity_;	// Default velocity for moving forward.
		float defaultRotationVelocity_; // Default velocity for rotating.

		// ROS variables.
		ros::Subscriber arduinoSensorsSub_;
		ros::Publisher cmdVelPub_;
		ros::NodeHandle nh_;
		ros::Subscriber odomSUb_;

		// Time of Flight sensor data.
		int tofReadCount;  // Number of times TOF sensors read.
		bool tofFound_;
		float frontLeftMm_;
		float frontRightMm_;
		float backLeftMm_;
		float backRightMm_;

		float avgFrontLeftMm_;
		float avgFrontRightMm_;
		float avgBackLeftMm_;
		float avgBackRightMm_;

		// Odometry.
		bool odomFound_;
		int odomReadCount_;
		nav_msgs::Odometry lastOdom_;

		// Private methods.

		// Current Pose;
		SimplePose currentPose();

		void handleArduinoSensors(const lysander::ArduinoSensors::ConstPtr& arduino_sensors);
		void handleOdom(const nav_msgs::Odometry::ConstPtr& odom);

		// Have any of the edge sensors found an edge?
		bool anyEdgeFound();

		// Have both of the front edge sensors found an edge?
		bool bothFrontSensorsFoundEdge();

		// Has back-left edge sensor found an edge?
		bool backLeftEdgeFound();

		// Has back-right edge sensor found an edge?
		bool backRightEdgeFound();

		// Is the current Pose close to the desired Pose?
		bool closeToDesiredPose(SimplePose desiredPose);

		// Constrain angle to be in [0..360].
		float constrainEulerAngle(float x);

		void computeAvgTofDistances();

		// Get current goal.
		GOAL currentGoal();

		// Top marked Pose from stack;
		SimplePose currentMarkedPose();

		// Has front-left edge sensor found an edge?
		bool frontLeftEdgeFound();

		// Has front-right edge sensor found an edge?
		bool frontRightEdgeFound();
		
		// String equivalent of GOAL.
		const char* goalName(GOAL goal);

		// Compute goal Pose from a given Pose for a given angle and distance.
		SimplePose goalPose(SimplePose originalPose, float changeEuler, float distance);

		// Remove top goal from stack.
		GOAL popGoal();

		// Push a Pose reading of one edge of the table.
		void pushEdgePose(SimplePose pose);

		// Install new goal on stack.
		void pushGoal(GOAL goal);

		// Computer smallest Euler angle between two angles.
		float smallestEulerAngleBetween(float a1, float a2);

		// Stop motors.
		void stop();
	};

}