#ifndef __LYSANDER_MOTOR
#define __LYSANDER_MOTOR

#include "ros/ros.h"
#include <time.h>
#include "WRDifferentialDrive.h"

class LysanderMotor : public WRDifferentialDrive {
public:
	LysanderMotor(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);

	~LysanderMotor();

	void controlLoop();

	virtual void read(const ros::Time& time, const ros::Duration& period);

	virtual void update();

	virtual void write(const ros::Time& time, const ros::Duration& period);

private:
	static const double kBILLION;

	ros::NodeHandle nh_;

	urdf::Model *urdf_model_;

	// Time.
	ros::Duration elapsedTime_;
	ros::Duration expectedControlLoopDuration_;
	struct timespec lastTime_;
	double controlLoopMaxAllowedDurationDeviation_;
	struct timespec now_;


	/** \brief ROS Controller Manager and Runner
	 *
	 * This class advertises a ROS interface for loading, unloading, starting, and
	 * stopping ros_control-based controllers. It also serializes execution of all
	 * running controllers in \ref update.
	 */
	boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;

	double controlLoopHz_;
};

#endif
