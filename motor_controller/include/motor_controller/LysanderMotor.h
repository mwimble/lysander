#ifndef __LYSANDER_MOTOR
#define __LYSANDER_MOTOR

#include "ros/ros.h"
#include <boost/thread/mutex.hpp>
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/joint_state_interface.h"
#include "WRDifferentialDrive.h"

class LysanderMotor : public WRDifferentialDrive {
public:
	struct TRoboClawException : public std::exception {
		std::string s;
		TRoboClawException(std::string ss) : s(ss) {}
		~TRoboClawException() throw() {}
		const char* what() const throw() { return s.c_str(); }
	};

	LysanderMotor(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL);

	~LysanderMotor();

	void controlLoop();

	virtual void read(const ros::Time& time, const ros::Duration& period);

	virtual void update();

	virtual void write(const ros::Time& time, const ros::Duration& period);

	// Get RoboClaw error status bits.
	uint16_t getErrorStatus();

	// Get RoboClaw error status as a string.
	std::string getErrorString();

	float getLogicBatteryLevel();

	float getMainBatteryLevel();

	typedef struct {
		int32_t		value;
		uint8_t		status;
	} EncodeResult;

	// Get the encoder value for motor 1.
	int32_t getM1Encoder();

	// Get the encoder value for motor 2.
	int32_t getM2Encoder();

	typedef struct {
		float m1Current;
		float m2Current;
	} TMotorCurrents;

	TMotorCurrents getMotorCurrents();

	typedef struct {
		int32_t p;
		int32_t i;
		int32_t d;
		int32_t q;
	} TPIDQ;

	TPIDQ getM1PIDQ();
	
	int32_t getM1Speed();

	TPIDQ getM2PIDQ();

	int32_t getM2Speed();

	// Get RoboClaw software versions.
	std::string getVersion();

	// Stop motion.
	void stop();

private:
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
	    kGETM1ENC = 16,
	    kGETM2ENC = 17,
	    GETM1SPEED = 18,
	    GETM2SPEED = 19,
	    RESETENC = 20,
	    kGETVERSION = 21,
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
	    kMIXEDSPEED = 37,
	    M1SPEEDACCEL = 38,
	    M2SPEEDACCEL = 39,
	    MIXEDSPEEDACCEL = 40,
	    M1SPEEDDIST = 41,
	    M2SPEEDDIST = 42,
	    kMIXEDSPEEDDIST = 43,
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
	    kGETERROR = 90,
	    WRITENVM = 94,
		GETM1MAXCURRENT = 135};

	int clawPort_;							// Unix file descriptor for RoboClaw connection.
	double controlLoopHz_;					// Loop rate for control loop.
	int maxCommandRetries_;					// Maximum number of times to retry a RoboClaw command.
	double maxSecondsUncommandedTravel_;	// Abort travel after this number of seconds if a new motion command has not arrived.
	std::string motorUSBPort_;				// Device name of RoboClaw device.
	int portAddress_;						// Port number of RoboClaw device under control
	double quadPulsesPerMeter_;				// Number of quadrature pulses that will be received after 1 meter of travel.
	boost::mutex roboClawLock_;				// To control multithread access.
	double wheelRadius_;					// Wheel radius.

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

	// Get the encoder result given a command which indicates which motor to interrogate.
	EncodeResult getEncoderCommandResult(uint8_t command);

	// Open the RoboClaw USB port.
	void  openPort();

	// Read one byte from device with timeout.
	uint8_t readByteWithTimeout();

	// Perform error recovery to re-open a failed device port.
	void restartPort();

	// Update the running CRC result.
	void updateCrc(uint16_t& crc, uint8_t data);

	// Write one byte to the device.
	void writeByte(uint8_t byte);

	// Write a stream of bytes to the device.
	void writeN(bool sendCRC, uint8_t cnt, ...);
};

#endif
