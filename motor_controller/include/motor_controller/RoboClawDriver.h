#ifndef __ROBOCLAW_DRIVER_H
#define __ROBOCLAW_DRIVER_H

#include <fcntl.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <LockFreeQueue.h>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#include "motor_controller/motor_controllerConfig.h"
#include "motor_controller/RoboClawStatus.h"

#include <boost/asio.hpp>
#include <boost/thread.hpp>

using namespace std;

class RoboClawDriver {
public:
	struct RoboClawDriverException : public std::exception {
		std::string s;
		RoboClawDriverException(std::string ss) : s(ss) {}
		~RoboClawDriverException() throw() {}
		const char* what() const throw() { return s.c_str(); }
	};


	RoboClawDriver();
	~RoboClawDriver();

	void configCallback(motor_controller::motor_controllerConfig &config, uint32_t level);

	// Get the logic battery level from the RoboClaw.
	float getLogicBatteryLevel();

	// Get the main battery level from the RoboClaw.
	float getMainBatteryLevel();

	typedef struct {
		int32_t		value;
		uint8_t		status;
	} EncodeResult;

	int32_t getM1Encoder();

	int32_t getM2Encoder();

	typedef struct {
		float m1Current;
		float m2Current;
	} TMotorCurrents;

	// Get the motor currents from the RoboClaw.
	TMotorCurrents getMotorCurrents();

	typedef struct {
		int32_t p;
		int32_t i;
		int32_t d;
		int32_t q;
	} TPIDQ;

	// Get the PID and qpps values from the RoboClaw for motor 1.
	TPIDQ getM1PIDQ();
	
	// Get the current Velocity for motor 1 from the RoboClaw.
	int32_t getM1Velocity();

	// Get the PID and qpps values from the RoboClaw for motor 1.
	TPIDQ getM2PIDQ();

	// Get the current Velocity for motor 2 from the RoboClaw.
	int32_t getM2Velocity();

	// Get the version string from the RoboClaw.
	string getVersion();

	// Stop the motors (free run).
	void stop();

private:
	bool	debug_;	// Enable debug traces.

	ros::NodeHandle nh_;	// Ros node handle.

	// Threads, command queue and mutex
	static boost::mutex roboClawLock_; 					// Mutex for access to roboclaw via USB.
	boost::thread roboClawStatusReaderThread_; 			// Thead to publish RoboClaw status.

	// For cmd_vel message handling.
	ros::CallbackQueue cmdVelMessageQueue_; 			// Will hold queued cmd_vel messages
	boost::thread cmdVelQueueHandlerThread_;			// Thread to handle queued cmd_vel messages.
	string cmdVelTopicName_;							// Topic name for cmd_vel message.
	boost::thread handleMotionCommandsThead_; 			// Thread to cmd_vel commands.
	LockFreeQueue<geometry_msgs::Twist> twistQueue_;	// Queue of cmd_vel messages.
	ros::Subscriber cmdVelSubscriber_;					// Subscriber to cmd_vel messages.
	
	// Maximum wheel velocity in meters/second.
	double maximumWheelVelocity_;

	// Minimum wheel velocity in meters/second.
	double minimumWheelVelocity_;

	typedef struct {
		unsigned long p1;
		unsigned long p2;
	} ULongPair;

	enum {M1FORWARD = 0,
	    M1BACKWARD = 1,
	    SETMINMB = 2,
	    SETMAXMB = 3,
	    M2FORWARD = 4,
	    M2BACKWARD = 5,
	    M17BIT = 6,
	    M27BIT = 7,
	    MIXEDFORWARD = 8,
	    MIXEDBACKWARD = 9,
	    MIXEDRIGHT = 10,
	    MIXEDLEFT = 11,
	    MIXEDFB = 12,
	    MIXEDLR = 13,
	    GETM1ENC = 16,
	    GETM2ENC = 17,
	    GETM1SPEED = 18,
	    GETM2SPEED = 19,
	    RESETENC = 20,
	    GETVERSION = 21,
	    GETMBATT = 24,
	    GETLBATT = 25,
	    SETMINLB = 26,
	    SETMAXLB = 27,
	    SETM1PID = 28,
	    SETM2PID = 29,
	    GETM1ISPEED = 30,
	    GETM2ISPEED = 31,
	    M1DUTY = 32,
	    M2DUTY = 33,
	    MIXEDDUTY = 34,
	    M1SPEED = 35,
	    M2SPEED = 36,
	    MIXEDSPEED = 37,
	    M1SPEEDACCEL = 38,
	    M2SPEEDACCEL = 39,
	    MIXEDSPEEDACCEL = 40,
	    M1SPEEDDIST = 41,
	    M2SPEEDDIST = 42,
	    MIXEDSPEEDDIST = 43,
	    M1SPEEDACCELDIST = 44,
	    M2SPEEDACCELDIST = 45,
	    MIXEDSPEEDACCELDIST = 46,
	    GETBUFFERS = 47,
	    SETPWM = 48,
	    GETCURRENTS = 49,
	    MIXEDSPEED2ACCEL = 50,
	    MIXEDSPEED2ACCELDIST = 51,
	    M1DUTYACCEL = 52,
	    M2DUTYACCEL = 53,
	    MIXEDDUTYACCEL = 54,
	    GETM1PID = 55,
	    GETM2PID = 56,
	    GETERROR = 90,
	    WRITENVM = 94,
		GETM1MAXCURRENT = 135};

	// For communicating with the RoboClaw device.
	int clawPort_;				// Device ID.
	char portAddress;
	int MAX_COMMAND_RETRIES;

	motor_controller::RoboClawStatus roboClawStatus;

	// PID and max quadrature pulses per second values.
	float m1P_;
	float m2P_;
	float m1I_;
	float m2I_;
	unsigned long m1Qpps_;
	unsigned long m2Qpps_;
	float axleWidth_;
	
	int32_t expectedM1Velocity_;	// Current motor 1 velocity.
	int32_t expectedM2Velocity_;	// Current motor 2 velocity.
	uint32_t maxM1Distance_;
	uint32_t maxM2Distance_;
	bool m1MovingForward_;		// Is motor 1 moving in a forward direction.
	bool m2MovingForward_;		// Is motor 2 moving in a forward direction.

	// Maximum time robot can move without a command.
	static const float kMAX_SECONDS_TRAVEL;

	string roboClawDeviceName_;		// Name of device port for RoboClaw.

	void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
	
	// Manage the cmd_vel message queue.
	void cmdVelQueueHandler();

	// Command robot to move at a given vector (velocity and angle).
	void drive(float velocity, float angle);

	// Flush the RoboClaw device.
	void flush();

	// Get short int command result from RoboClaw.
	unsigned short get2ByteCommandResult(uint8_t command);

	// The the results of an encoder command frmo the RoboClaw.
	EncodeResult getEncoderCommandResult(uint8_t command);

	// Get the error status bytes from the RoboClaw
	uint16_t getErrorStatus();

	// Get unaifnws long value and allow for continuation of read from RoboClaw.
	uint32_t getULongCont(uint16_t& crc);
	
	// Get the velocity from the RoboClaw using the command.
	int32_t getVelocityResult(uint8_t command);

	// Get an unsigned long result from a command issued to the RoboClaw.
	unsigned long getUlongCommandResult(uint8_t command);

	// Get a pair of unsigned long results from a command issued to the RoboClaw.
	ULongPair getULongPairCommandResult(uint8_t command);

	// Open the RoboClaw device.
	void  openPort();

	// Read a byte from the RoboClaw with timeout.
	uint8_t readByteWithTimeout();
	
	// Attempt to restart the RoboClaw connection.
	void restartRoboClaw();

	// Process cmd_vel commands from the queue.
    void handleMotionCommands();
    
	// Read status from the RoboClaw.
	void roboClawStatusReader();

	// Send PID and max quadrature pulses to RoboClaw for motor 1.
	void setM1PID(float p, float i, float d, uint32_t qpps);

	// Send PID and max quadrature pulses to RoboClaw for motor 2.
	void setM2PID(float p, float i, float d, uint32_t qpps);

 	// Compute the running CRC for a series of bytes.
	void updateCrc(uint16_t& crc, uint8_t data);
    
    // Convert an vector (angle and velocity) to left and right motor velocities.
	void vwToWheelVelocity(float velocity, float angle, float *leftMotorVelocity, float *rightMotorVelocity);

	// Write a single byte to the RoboClaw.
	void writeByte(uint8_t byte);

	// Write a list of bytes to the RoboClaw.
	void writeN(bool sendCRC, uint8_t cnt, ...);

};

#endif
