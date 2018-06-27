#include <ros/ros.h>

#include <lysander/ArduinoSensors.h>
#include <nav_msgs/Odometry.h>

namespace lysander {

	class TablebotStrategy {
	public:

		TablebotStrategy();

		void solveChallenge1();

	private:
		// Goals.
		typedef enum GOAL {
			kBACKUP,
			kFIND_TABLE_EDGE,
			kNONE,
			kSUCCESS,
		} GOAL;

		static const int MAX_GOAL_DEPTH = 20;	// Maximum depth of goal stack.
		GOAL currentGoal_[MAX_GOAL_DEPTH];		// Goal stack;
		int nextGoalIndex_;						// Index to place next goal in stack.

		// Parameters.
		bool debug_;
		float distanceToTableTopSlack_;
		float defaultForwardVelocity_;

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
		int lastDirection_;
		bool odomFound_;
		int odomReadCount_;
		nav_msgs::Odometry lastOdom_;

		// Private methods.

		void handleArduinoSensors(const lysander::ArduinoSensors::ConstPtr& arduino_sensors);
		void handleOdom(const nav_msgs::Odometry::ConstPtr& odom);

		bool anyTableEdgeFound();
		bool backLeftTableEdgeFound();
		bool backRightTableEdgeFound();
		void computeAvgTofDistances();

		// Get current goal.
		GOAL currentGoal();

		bool driveUntilEdgeIsComplete();
		bool frontLeftTableEdgeFound();
		bool frontRightTableEdgeFound();
		
		// Remove top goal from stack.
		GOAL popGoal();

		// Install new goal on stack.
		void pushGoal(GOAL goal);

		// Stop motors.
		void stop();
	};

}