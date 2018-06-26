#include <ros/ros.h>

#include <lysander/ArduinoSensors.h>

namespace lysander {

	class TablebotStrategy {
	public:

		TablebotStrategy();

		void solveChallenge1();

	private:
		// Parameters.
		bool debug_;
		float distanceToTableTopSlack_;
		float defaultForwardVelocity_;

		// ROS variables.
		ros::NodeHandle nh_;
		ros::Subscriber arduinoSensorsSub_;
		ros::Publisher cmdVelPub_;

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


		// Private methods.

		void handleArduinoSensors(const lysander::ArduinoSensors::ConstPtr& arduino_sensors);

		bool anyTableEdgeFound();
		bool backLeftTableEdgeFound();
		bool backRightTableEdgeFound();
		void computeAvgTofDistances();
		bool driveUntilEdgeIsComplete();
		bool frontLeftTableEdgeFound();
		bool frontRightTableEdgeFound();
		void stop();
	};

}