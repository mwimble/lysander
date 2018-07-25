
// roslaunch motor_controller motor.launch
// rosrun teleop_twist_keyboard teleop_twist_keyboard.py
// rostopic echo /RoboClawStatus
// rostopic echo /cmd_vel

// On exception, clear queue by waiting 10ms.
// Capture speed, update last speed.

#include <fcntl.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <poll.h>
#include <pthread.h>
#include <sstream>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <tf/transform_broadcaster.h>
#include <unistd.h>
#include <vector>

#include <linux/usbdevice_fs.h>

#include "motorSkidSteerDrive.h"

#define SetDWORDval(arg) (uint8_t)(arg>>24),(uint8_t)(arg>>16),(uint8_t)(arg>>8),(uint8_t)arg

// 7.5 inches in meters.
//#define axle_width_ 0.1905
#define axle_width_ 0.285 //0.255 //0.21166666666667
// 0.955173394218208, prev 1.22, new = 1.071704548312829
#define calibration_correction_factor_ 1.087780116537521 //1.071704548312829
#define quad_pulse_per_meter_ 6353.4523937932416

// Max no-load speed = 160 RPM
//#define meterdistTraveledper_revolution 0.392699081698724 // 0.1888736903376
//#define pulsedistTraveledper_inch 161.37769080234834

boost::mutex roboClawLock;

MotorSkidSteerDrive::MotorSkidSteerDrive() :
    expectedM1Speed(0),
    expectedM2Speed(0),
    m1MovingForward(true),
    m2MovingForward(true),
    maxM1Distance(0),
    maxM2Distance(0),
    max_wheel_vel_(0.4),
    min_wheel_vel_(0.0),
    robot_pose_px_(0.0),
    robot_pose_py_(0.0),
    theta_(0.0)
    {
	DEBUG = true;
    ROS_INFO_COND(DEBUG, "[MotorSkidSteerDrive::MotorSkidSteerDrive] constructor");
	if (!ros::isInitialized()) {
		ROS_FATAL_STREAM("[MotorSkidSteerDrive::MotorSkidSteerDrive] A ROS Node for MotorSkidSteerDrive has not been initialized.");
		throw new TRoboClawException("A ROS Node for MotorSkidSteerDrive has not been initialized.");
	}

	// if (pthread_mutex_init(&roboClawLock, NULL) != 0) {
	// 	ROS_FATAL_STREAM("[MotorSkidSteerDrive::MotorSkidSteerDrive] pthread_mutex_init failed");
	// 	throw new TRoboClawException("[MotorSkidSteerDrive::MotorSkidSteerDrive] pthread_mutex_init failed");
	// }

	alive = true;
	updateRate_ = 100.0;
	updatePeriod_ = 1.0 / updateRate_;
	M1_MAX_METERS_PER_SEC = 0.333;
	M2_MAX_METERS_PER_SEC = 0.333;
	MAX_SECONDS_TRAVEL = 0.5;
	portAddress = 0x80;
	MAX_COMMAND_RETRIES = 5;

	M1_P =  8762.98571;
	M2_P = 9542.41265;
	M1_I = 1535.49646;
	M2_I = 1773.65086;
	M1_QPPS = 3562;
	M2_QPPS = 3340;
	AXLE_WIDTH = 0.125;

	rosNode = new ros::NodeHandle(); //### namespace

	lastTime = ros::Time::now();

	assert(ros::param::get("~cmd_vel_topic", cmdVelTopic));
	assert(ros::param::get("~motor_usb_port", motorUSBPort));
	assert(ros::param::get("~do_odom_tf", do_odom_tf_));

	ROS_INFO("[MotorSkidSteerDrive] PARAM cmdVelTopic: %s", cmdVelTopic.c_str());
	ROS_INFO("[MotorSkidSteerDrive] PARAM motorUSBPort: %s", motorUSBPort.c_str());
	ROS_INFO("[MotorSkidSteerDrive] PARAM do_odom_tf: %s", do_odom_tf_ ? "TRUE" : "FALSE");

	openPort();

	stop();

	setM1PID(M1_P, M1_I, 0, M1_QPPS);
	setM2PID(M2_P, M2_I, 0, M2_QPPS);
	ROS_INFO_COND(DEBUG, "[MotorSkidSteerDrive::MotorSkidSteerDrive] Starting");
	ROS_INFO("[MotorSkidSteerDrive::MotorSkidSteerDrive] getLogicBatteryLevel %f", getLogicBatteryLevel());

	ros::SubscribeOptions so = 
		ros::SubscribeOptions::create<geometry_msgs::Twist>(
				cmdVelTopic,
				1,
				boost::bind(&MotorSkidSteerDrive::cmdVelCallback, this, _1),
				ros::VoidPtr(),
				&queue
			);

	callbackQueueThread = boost::thread(boost::bind(&MotorSkidSteerDrive::queueThread, this));
	roboClawStatusReaderThreadThread = boost::thread(boost::bind(&MotorSkidSteerDrive::roboClawStatusReaderThread, this));
	roboClawMotorControllerThread = boost::thread(boost::bind(&MotorSkidSteerDrive::robotMotorControllerThread, this));
	roboClawOdometryThread = boost::thread(boost::bind(&MotorSkidSteerDrive::updateOdometryThread, this));

	cmdVelSubscriber = rosNode->subscribe(so);

}

MotorSkidSteerDrive::~MotorSkidSteerDrive() {
	roboClawLock.lock();
	if (clawPort) {
		flush();
		close(clawPort);
	}

	roboClawLock.unlock();
}

void MotorSkidSteerDrive::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg) {
	ROS_INFO_COND(false, "[MotorSkidSteerDrive::cmdVelCallback] cmd_msg.linear.x: %f, cmd_msg.angular.z: %f", cmd_msg->linear.x, cmd_msg->angular.z);
	geometry_msgs::Twist copy;
	copy.linear.x = cmd_msg->linear.x;
	copy.angular.z = cmd_msg->angular.z;
	twistQueue.Produce(copy);
	// float velocity = cmd_msg->linear.x;
	// float angle = cmd_msg->angular.z;
	// drive(velocity, angle);
}

void MotorSkidSteerDrive::configCallback(motor_controller::motor_controllerConfig &config, uint32_t level) {
	ROS_INFO("[MotorSkidSteerDrive::configCallback]\n\tlevel: %d\n\tcmd_vel_topic: %s\n\tKB_velocity:  %f\n\tmotorUSBPort: %s\n\t*updatePeriod: %f", 
			 level,
			 config.cmd_vel_topic.c_str(),
			 config.KP_velocity,
			 config.motor_usb_port.c_str(),
			 updatePeriod_);
	if (M1_P != config.M1_P) {
		M1_P = config.M1_P;
		ROS_INFO("[MotorSkidSteerDrive::configCallback] setting new M1_P value: %f", M1_P);
		setM1PID(M1_P, M1_I, 0, M1_QPPS);
	}

	if (M1_I != (float) config.M1_I) {
		M1_I = (float) config.M1_I;
		ROS_INFO("[MotorSkidSteerDrive::configCallback] setting new M1_I value: %f", M1_I);
		setM1PID(M1_I, M1_I, 0, M1_QPPS);
	}

	if (M1_QPPS != (float) config.M1_QPPS) {
		M1_QPPS = (float) config.M1_QPPS;
		ROS_INFO("[MotorSkidSteerDrive::configCallback] setting new M1_QPPS value: %ld", M1_QPPS);
		setM1PID(M1_P, M1_I, 0, M1_QPPS);
	}

	if (M2_P != (float) config.M2_P) {
		M2_P = (float) config.M2_P;
		ROS_INFO("[MotorSkidSteerDrive::configCallback] setting new M2_P value: %f", M2_P);
		setM2PID(M2_P, M2_I, 0, M2_QPPS);
	}

	if (M2_I != (float) config.M2_I) {
		M2_I = (float) config.M2_I;
		ROS_INFO("[MotorSkidSteerDrive::configCallback] setting new M2_I value: %f", M2_I);
		setM2PID(M2_P, M2_I, 0, M2_QPPS);
	}

	if (M2_QPPS != config.M2_QPPS) {
		M2_QPPS = config.M2_QPPS;
		ROS_INFO("[MotorSkidSteerDrive::configCallback] setting new M2_QPPS value: %ld", M2_QPPS);
		setM2PID(M2_P, M2_I, 0, M2_QPPS);
	}

	if (AXLE_WIDTH != config.AXLE_WIDTH) {
		AXLE_WIDTH = config.AXLE_WIDTH;
		ROS_INFO("[MotorSkidSteerDrive::configCallback] setting new AXLE_WIDTH value: %f", AXLE_WIDTH);
	}
}

void MotorSkidSteerDrive::drive(float velocity, float angle) {
	boost::mutex::scoped_lock scoped_lock(roboClawLock);
	ROS_INFO_COND(DEBUG, "-----> [MotorSkidSteerDrive::drive] velocity: %f, angle: %f", velocity, angle);
	int32_t m1_speed;
	int32_t m2_speed;
	setVelocities(velocity, angle, &m1_speed, &m2_speed);
	int32_t m1_abs_speed = m1_speed >= 0 ? m1_speed : -m1_speed;
	int32_t m2_abs_speed = m2_speed >= 0 ? m2_speed : -m2_speed;
	int32_t m1_max_distance = m1_abs_speed * MAX_SECONDS_TRAVEL; // Limit travel.
	int32_t m2_max_distance = m2_abs_speed * MAX_SECONDS_TRAVEL; // Limit travel.
	ROS_INFO_STREAM_COND(DEBUG, "[MotorSkidSteerDrive::drive] ---- command: " << MIXEDSPEEDDIST
		 << ", drive velocity: " << velocity
		 << ", angle: " << angle
		 << ", m1_speed: " << m1_speed
		 << ", m1_max_distance: " << m1_max_distance
		 << ", m2_speed: " << m2_speed
		 << ", m2_max_distance: " << m2_max_distance);

	expectedM1Speed = m1_speed;
	expectedM2Speed = m2_speed;
	maxM1Distance = m1_max_distance;
	maxM2Distance = m2_max_distance;

	int retry;
	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			writeN(true, 19, portAddress, MIXEDSPEEDDIST,
				   SetDWORDval(m1_speed),
				   SetDWORDval(m1_max_distance),
				   SetDWORDval(m2_speed),
				   SetDWORDval(m2_max_distance),
				   1 /* Cancel any previous command */
				   );
	    	//ROS_INFO_COND(DEBUG, "<----- [MotorSkidSteerDrive::drive] return");
			return;
		} catch (TRoboClawException* e) {
			ROS_ERROR_COND(DEBUG, "[MotorSkidSteerDrive::drive] Exception: %s, retry number %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorSkidSteerDrive::drive] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [MotorSkidSteerDrive::drive] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorSkidSteerDrive::drive] RETRY COUNT EXCEEDED");
}

void MotorSkidSteerDrive::flush() {
	int retval = tcflush(clawPort, TCIOFLUSH);
	//ROS_INFO_COND(DEBUG, "[MotorSkidSteerDrive::flush]");
	if (retval != 0) {
		ROS_ERROR("[MotorSkidSteerDrive::flush] Unable to flush device");
		throw new TRoboClawException("[MotorSkidSteerDrive::flush] Unable to flush device");
	}
}

void updateCrc(uint16_t& crc, uint8_t data) {
	crc = crc ^ ((uint16_t) data << 8);
	for (int i = 0; i < 8; i++)	{
		if (crc & 0x8000)
			crc = (crc << 1) ^ 0x1021;
		else
			crc <<= 1;
	}
}

unsigned short MotorSkidSteerDrive::get2ByteCommandResult(uint8_t command) {
	//ROS_INFO_COND(DEBUG, "[MotorSkidSteerDrive::get2ByteCommandResult] command: 0x%X", command);
	uint16_t crc = 0;
	updateCrc(crc, portAddress);
	updateCrc(crc, command);

	writeN(false, 2, portAddress, command);
	unsigned short result = 0;
	uint8_t datum = readByteWithTimeout();
	result |= datum << 8;
	updateCrc(crc, datum);

	if (datum != -1) {
		datum = readByteWithTimeout();
		result |= datum;
		updateCrc(crc, datum);
	}

	uint16_t responseCrc = 0;
	if (datum != -1) {
		datum = readByteWithTimeout();
		if (datum != -1) {
			responseCrc = datum << 8;
			datum = readByteWithTimeout();
			if (datum != -1) {
				responseCrc |= datum;
				if (responseCrc == crc) {
					return result;
				}
			}
		}
	}

	if (DEBUG) ROS_ERROR("[MotorSkidSteerDrive::get2ByteCommandResult] Expected CRC of: 0x%X, but got: 0x%X",
	    int(crc),
	    int(responseCrc));
	throw new TRoboClawException("[MotorSkidSteerDrive::get2ByteCommandResult] INVALID CRC");
}

MotorSkidSteerDrive::EncodeResult MotorSkidSteerDrive::getEncoderCommandResult(uint8_t command) {
	//ROS_INFO_COND(DEBUG, "[MotorSkidSteerDrive::getEncoderCommandResult] command: 0x%X", command);
	uint16_t crc = 0;
	updateCrc(crc, portAddress);
	updateCrc(crc, command);

	writeN(false, 2, portAddress, command);
	EncodeResult result = {0, 0};
	uint8_t datum = readByteWithTimeout();
	if (datum == -1) {
		ROS_ERROR("[MotorSkidSteerDrive::getEncoderCommandResult] Received -1 instead of expected data");
		throw new TRoboClawException("[MotorSkidSteerDrive::::getEncoderCommandResult] RECEIVED -1");
	}

	result.value |= datum << 24;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	if (datum == -1) {
		ROS_ERROR("[MotorSkidSteerDrive::getEncoderCommandResult] Received -1 instead of expected data");
		throw new TRoboClawException("[MotorSkidSteerDrive::::getEncoderCommandResult] RECEIVED -1");
	}

	result.value |= datum << 16;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	if (datum == -1) {
		ROS_ERROR("[MotorSkidSteerDrive::getEncoderCommandResult] Received -1 instead of expected data");
		throw new TRoboClawException("[MotorSkidSteerDrive::::getEncoderCommandResult] RECEIVED -1");
	}

	result.value |= datum << 8;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	if (datum == -1) {
		ROS_ERROR("[MotorSkidSteerDrive::getEncoderCommandResult] Received -1 instead of expected data");
		throw new TRoboClawException("[MotorSkidSteerDrive::::getEncoderCommandResult] RECEIVED -1");
	}

	result.value |= datum;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	if (datum == -1) {
		ROS_ERROR("[MotorSkidSteerDrive::getEncoderCommandResult] Received -1 instead of expected data");
		throw new TRoboClawException("[MotorSkidSteerDrive::::getEncoderCommandResult] RECEIVED -1");
	}

	result.status |= datum;
	updateCrc(crc, datum);
	if (datum == -1) {
		ROS_ERROR("[MotorSkidSteerDrive::getEncoderCommandResult] Received -1 instead of expected data");
		throw new TRoboClawException("[MotorSkidSteerDrive::::getEncoderCommandResult] RECEIVED -1");
	}

	uint16_t responseCrc = 0;
	datum = readByteWithTimeout();
	if (datum != -1) {
		responseCrc = datum << 8;
		datum = readByteWithTimeout();
		if (datum != -1) {
			responseCrc |= datum;
			if (responseCrc == crc) {
				return result;
			}
		}
	}

	if (DEBUG) ROS_ERROR("[MotorSkidSteerDrive::getEncoderCommandResult] Expected CRC of: 0x%X, but got: 0x%X",
	    int(crc),
	    int(responseCrc));
	throw new TRoboClawException("[MotorSkidSteerDrive::getEncoderCommandResult] INVALID CRC");

}

uint16_t MotorSkidSteerDrive::getErrorStatus() {
	boost::mutex::scoped_lock lock(roboClawLock);
	//ROS_INFO_COND(DEBUG, "-----> [MotorSkidSteerDrive::getErrorStatus]");
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			uint16_t crc = 0;
			updateCrc(crc, portAddress);
			updateCrc(crc, GETERROR);
			writeN(false, 2, portAddress, GETERROR);
			uint16_t result = 0;
			uint8_t datum = readByteWithTimeout();
			updateCrc(crc, datum);
			result = datum << 8;
			datum = readByteWithTimeout();
			updateCrc(crc, datum);
			result |= datum;

			uint16_t responseCrc = 0;
			if (datum != -1) {
				datum = readByteWithTimeout();
				if (datum != -1) {
					responseCrc = datum << 8;
					datum = readByteWithTimeout();
					if (datum != -1) {
						responseCrc |= datum;
						if (responseCrc == crc) {
							//ROS_INFO_COND(DEBUG, "<----- [MotorSkidSteerDrive::getErrorStatus] return: 0x%X", datum);
							return result;
						}
					}
				}
			}
		} catch (TRoboClawException* e) {
			ROS_ERROR_COND(DEBUG, "[MotorSkidSteerDrive::getErrorStatus] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorSkidSteerDrive::getErrorStatus] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [MotorSkidSteerDrive::getErrorStatus] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorSkidSteerDrive::getErrorStatus] RETRY COUNT EXCEEDED");
}

float MotorSkidSteerDrive::getLogicBatteryLevel() {
	boost::mutex::scoped_lock lock(roboClawLock);
	//ROS_INFO_COND(DEBUG, "-----> [MotorSkidSteerDrive::getLogicBatteryLevel]");
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			float result = ((float) get2ByteCommandResult(GETLBATT)) / 10.0;
			//ROS_INFO_COND(DEBUG, "<----- [MotorSkidSteerDrive::getLogicBatteryLevel] return %f", result);
			return result;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[MotorSkidSteerDrive::getLogicBatteryLevel] Exception: %s, retry number: %d", e->what(), retry);
			restartPort();
		} catch (...) {
		    ROS_ERROR("[MotorSkidSteerDrive::getLogicBatteryLevel] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [MotorSkidSteerDrive::getLogicBatteryLevel] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorSkidSteerDrive::getLogicBatteryLevel] RETRY COUNT EXCEEDED");
}

uint32_t MotorSkidSteerDrive::getLongCont(uint16_t& crc) {
	//ROS_INFO_COND(DEBUG, "[MotorSkidSteerDrive::getLongCont] CRC: 0x%X", crc);
	uint32_t result = 0;
	uint8_t datum = readByteWithTimeout();
	result |= datum << 24;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	result |= datum << 16;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	result |= datum << 8;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	result |= datum;
	updateCrc(crc, datum);
	return result;
}

int32_t MotorSkidSteerDrive::getM1Encoder() {
	boost::mutex::scoped_lock lock(roboClawLock);
	//ROS_INFO_COND(DEBUG, "-----> [MotorSkidSteerDrive::getM1Encoder]");
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			EncodeResult result = getEncoderCommandResult(GETM1ENC);
	    	//ROS_INFO_COND(DEBUG, "<----- [MotorSkidSteerDrive::getM1Encode] return: %d/0x%X", result.value, result.status);
			return result.value;
		} catch (TRoboClawException* e) {
			ROS_ERROR_COND(DEBUG, "[MotorSkidSteerDrive::getM1Encode] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorSkidSteerDrive::getM1Encoder] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [MotorSkidSteerDrive::getM1Encoder] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorSkidSteerDrive::getM1Encoder] RETRY COUNT EXCEEDED");
}

int32_t MotorSkidSteerDrive::getSpeedResult(uint8_t command) {
	//ROS_INFO_COND(DEBUG, "[MotorSkidSteerDrive::getSpeedResul] command: 0x%X", command);
	uint16_t crc = 0;
	updateCrc(crc, portAddress);
	updateCrc(crc, command);

	writeN(false, 2, portAddress, command);
	int32_t result = 0;
	uint8_t datum = readByteWithTimeout();
	result |= datum << 24;
	updateCrc(crc, datum);

	datum = readByteWithTimeout();
	result |= datum << 16;
	updateCrc(crc, datum);

	datum = readByteWithTimeout();
	result |= datum << 8;
	updateCrc(crc, datum);

	datum = readByteWithTimeout();
	result |= datum;
	updateCrc(crc, datum);

	uint8_t direction = readByteWithTimeout();
	updateCrc(crc, direction);
	if (direction != 0) result = -result;

	uint16_t responseCrc = 0;
	if (datum != -1) {
		datum = readByteWithTimeout();
		if (datum != -1) {
			responseCrc = datum << 8;
			datum = readByteWithTimeout();
			if (datum != -1) {
				responseCrc |= datum;
				if (responseCrc == crc) {
					return result;
				}
			}
		}
	}

	if (DEBUG) ROS_ERROR("[MotorSkidSteerDrive::getSpeedResult] Expected CRC of: 0x%X, but got: 0x%X",
	    int(crc),
	    int(responseCrc));
	throw new TRoboClawException("[MotorSkidSteerDrive::getSpeedResult] INVALID CRC");
}

int32_t MotorSkidSteerDrive::getM1Speed() {
	boost::mutex::scoped_lock lock(roboClawLock);
	//ROS_INFO_COND(DEBUG, "-----> [MotorSkidSteerDrive::getM1Speed]");
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			uint32_t result = getSpeedResult(GETM1SPEED);
	    	//ROS_INFO_COND(DEBUG, "<----- [MotorSkidSteerDrive::getM1Speed] return: %d", result);
		    if (result < 0) {
				m1MovingForward = false;
		    } else {
				m1MovingForward = true;
		    }

			return result;
		} catch (TRoboClawException* e) {
			ROS_ERROR_COND(DEBUG, "[MotorSkidSteerDrive::getM1Speed] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorSkidSteerDrive::getM1Speed] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [MotorSkidSteerDrive::getM1Speed] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorSkidSteerDrive::getM1Speed] RETRY COUNT EXCEEDED");
}

int32_t MotorSkidSteerDrive::getM2Encoder() {
	boost::mutex::scoped_lock lock(roboClawLock);
	//ROS_INFO_COND(DEBUG, "-----> [MotorSkidSteerDrive::getM2Encoder]");
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			EncodeResult result = getEncoderCommandResult(GETM2ENC);
	    	//ROS_INFO_COND(DEBUG, "<----- [MotorSkidSteerDrive::getM2Encoder] return: %d/0x%X", result.value, result.status);
			return result.value;
		} catch (TRoboClawException* e) {
			ROS_ERROR_COND(DEBUG, "[MotorSkidSteerDrive::getM2Encoder] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorSkidSteerDrive::getM2Encoder] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [MotorSkidSteerDrive::getM2Encoder] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorSkidSteerDrive::getM2Encoder] RETRY COUNT EXCEEDED");
}

int32_t MotorSkidSteerDrive::getM2Speed() {
	boost::mutex::scoped_lock lock(roboClawLock);
	//ROS_INFO_COND(DEBUG, "-----> [MotorSkidSteerDrive::getM2Speed]");
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			uint32_t result = getSpeedResult(GETM2SPEED);
		    //ROS_INFO_COND(DEBUG, "<----- [MotorSkidSteerDrive::getM2Speed] return: %d", result);
		    if (result < 0) {
				m2MovingForward = false;
		    } else {
				m2MovingForward = true;
		    }

			return result;
		} catch (TRoboClawException* e) {
			ROS_ERROR_COND(DEBUG, "[MotorSkidSteerDrive::getM2Speed] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorSkidSteerDrive::getM2Speed] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [MotorSkidSteerDrive::getM2Speed] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorSkidSteerDrive::getM2Speed] RETRY COUNT EXCEEDED");
}

float MotorSkidSteerDrive::getMainBatteryLevel() {
	boost::mutex::scoped_lock lock(roboClawLock);
	//ROS_INFO_COND(DEBUG, "-----> [MotorSkidSteerDrive::getMainBatteryLevel]");
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			float result = ((float) get2ByteCommandResult(GETMBATT)) / 10.0;
			//ROS_INFO_COND(DEBUG, "<----- [MotorSkidSteerDrive::getMainBatteryLevel] return: %f", result);
			return result;
		} catch (TRoboClawException* e) {
			ROS_ERROR_COND(DEBUG, "[MotorSkidSteerDrive::getMainBatteryLevel] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorSkidSteerDrive::getMainBatteryLevel] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [MotorSkidSteerDrive::getMainBatteryLevel] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorSkidSteerDrive::getMainBatteryLevel] RETRY COUNT EXCEEDED");
}

MotorSkidSteerDrive::TMotorCurrents MotorSkidSteerDrive::getMotorCurrents() {
	boost::mutex::scoped_lock lock(roboClawLock);
	//ROS_INFO_COND(DEBUG, "-----> [MotorSkidSteerDrive::getMotorCurrents]");
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			TMotorCurrents result;
			unsigned long currentPair = getUlongCommandResult(GETCURRENTS);
			result.m1Current = ((int16_t) (currentPair >> 16)) * 0.010;
			result.m2Current = ((int16_t) (currentPair & 0xFFFF)) * 0.010;
			//ROS_INFO_COND(DEBUG, "<----- [MotorSkidSteerDrive::getMotorCurrent] return: %f/%f", result.m1Current, result.m2Current);
			return result;
		} catch (TRoboClawException* e) {
			ROS_ERROR_COND(DEBUG, "[MotorSkidSteerDrive::getMotorCurrents] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorSkidSteerDrive::getMotorCurrents] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [MotorSkidSteerDrive::getMotorCurrents] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorSkidSteerDrive::getMotorCurrents] RETRY COUNT EXCEEDED");
}

unsigned long MotorSkidSteerDrive::getUlongCommandResult(uint8_t command) {
	//ROS_INFO_COND(DEBUG, "[MotorSkidSteerDrive::getUlongCommandResult] command: 0x%X", command);
	uint16_t crc = 0;
	updateCrc(crc, portAddress);
	updateCrc(crc, command);

	writeN(false, 2, portAddress, command);
	unsigned long result = 0;
	uint8_t datum = readByteWithTimeout();
	result |= datum << 24;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	result |= datum << 16;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	result |= datum << 8;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	result |= datum;
	updateCrc(crc, datum);

	uint16_t responseCrc = 0;
	if (datum != -1) {
		datum = readByteWithTimeout();
		if (datum != -1) {
			responseCrc = datum << 8;
			datum = readByteWithTimeout();
			if (datum != -1) {
				responseCrc |= datum;
				if (responseCrc == crc) {
					return result;
				}
			}
		}
	}

	if (DEBUG) ROS_ERROR("[MotorSkidSteerDrive::getUlongCommandResult] Expected CRC of: 0x%X, but got: 0x%X",
	    int(crc),
	    int(responseCrc));
	throw new TRoboClawException("[MotorSkidSteerDrive::getUlongCommandResult] INVALID CRC");
}

MotorSkidSteerDrive::ULongPair MotorSkidSteerDrive::getULongPairCommandResult(uint8_t command) {
	//ROS_INFO_COND(DEBUG, "[MotorSkidSteerDrive::getULongPairCommandResult] command: 0x%X", command);
	uint16_t crc = 0;
	updateCrc(crc, portAddress);
	updateCrc(crc, command);

	writeN(false, 2, portAddress, command);
	uint32_t result1 = getLongCont(crc);
	uint32_t result2 = getLongCont(crc);

	uint16_t responseCrc = 0;
	uint16_t datum = readByteWithTimeout();
	responseCrc = datum << 8;
	datum = readByteWithTimeout();
	if (datum != -1) {
		responseCrc |= datum;
		if (responseCrc == crc) {
			ULongPair result;
			result.p1 = result1;
			result.p2 = result2;
			return result;
		}
	}

	if (DEBUG) ROS_ERROR("[MotorSkidSteerDrive::getUlongCommandResult] Expected CRC of: 0x%X, but got: 0x%X",
	    int(crc),
	    int(responseCrc));
	throw new TRoboClawException("[MotorSkidSteerDrive::getUlongCommandResult] INVALID CRC");
}

MotorSkidSteerDrive::TPIDQ MotorSkidSteerDrive::getM1PIDQ() {
	boost::mutex::scoped_lock lock(roboClawLock);
	//ROS_INFO_COND(DEBUG, "-----> [MotorSkidSteerDrive::getM1PIDQ]");
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		TPIDQ result;
		try {
			uint16_t crc = 0;
			updateCrc(crc, portAddress);
			updateCrc(crc, GETM1PID);

			writeN(false, 2, portAddress, GETM1PID);
			result.p = (int32_t) getLongCont(crc);
			result.i = (int32_t) getLongCont(crc);
			result.d = (int32_t) getLongCont(crc);
			result.q = (int32_t) getLongCont(crc);

			uint16_t responseCrc = 0;
			uint16_t datum = readByteWithTimeout();
			if (datum != -1) {
				responseCrc = datum << 8;
				datum = readByteWithTimeout();
				if (datum != -1) {
					responseCrc |= datum;
					if (responseCrc == crc) {
						// ROS_INFO_COND(DEBUG, "<----- [MotorSkidSteerDrive::getM1PIDQ] return: %d, %d, %d, %d",
						//       result.p,
						//       result.i,
						//       result.d,
						//       result.q);
						return result;
					}
				}
			}
		} catch (TRoboClawException* e) {
			ROS_ERROR_COND(DEBUG, "[MotorSkidSteerDrive::getM1PIDQ] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorSkidSteerDrive::getM1PIDQ] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [MotorSkidSteerDrive::getM1PIDQ] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorSkidSteerDrive::getM1PIDQ] RETRY COUNT EXCEEDED");
}

MotorSkidSteerDrive::TPIDQ MotorSkidSteerDrive::getM2PIDQ() {
	boost::mutex::scoped_lock lock(roboClawLock);
	//ROS_INFO_COND(DEBUG, "-----> [MotorSkidSteerDrive::getM2PIDQ]");
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		TPIDQ result;
		try {
			uint16_t crc = 0;
			updateCrc(crc, portAddress);
			updateCrc(crc, GETM2PID);

			writeN(false, 2, portAddress, GETM2PID);
			result.p = (int32_t) getLongCont(crc);
			result.i = (int32_t) getLongCont(crc);
			result.d = (int32_t) getLongCont(crc);
			result.q = (int32_t) getLongCont(crc);

			uint16_t responseCrc = 0;
			uint16_t datum = readByteWithTimeout();
			if (datum != -1) {
				responseCrc = datum << 8;
				datum = readByteWithTimeout();
				if (datum != -1) {
					responseCrc |= datum;
					if (responseCrc == crc) {
						// ROS_INFO_COND(DEBUG, "<----- [MotorSkidSteerDrive::getM2PIDQ] return: %d, %d, %d, %d",
						//       result.p,
						//       result.i,
						//       result.d,
						//       result.q);
						return result;
					}
				}
			}
		} catch (TRoboClawException* e) {
			ROS_ERROR_COND(DEBUG, "[MotorSkidSteerDrive::getM2PID] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorSkidSteerDrive::getM2PIDQ] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [MotorSkidSteerDrive::getM2PIDQ] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorSkidSteerDrive::getM2PIDQ] RETRY COUNT EXCEEDED");
}

string MotorSkidSteerDrive::getVersion() {
	boost::mutex::scoped_lock lock(roboClawLock);
	//ROS_INFO_COND(DEBUG, "-----> [MotorSkidSteerDrive::getVersion]");
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			uint16_t crc = 0;
			updateCrc(crc, portAddress);
			updateCrc(crc, GETVERSION);
			writeN(false, 2, portAddress, GETVERSION);

			uint8_t i;
			uint8_t datum;
			stringstream version;

			for (i = 0; i < 32; i++) {
				if (datum != -1) {
					datum = readByteWithTimeout();
					version << (char) datum;
					updateCrc(crc, datum);
					if (datum == 0) {
						uint16_t responseCrc = 0;
						if (datum != -1) {
							datum = readByteWithTimeout();
							if (datum != -1) {
								responseCrc = datum << 8;
								datum = readByteWithTimeout();
								if (datum != -1) {
									responseCrc |= datum;
									if (responseCrc == crc) {
										// ROS_INFO_COND(DEBUG, "<----- [MotorSkidSteerDrive::getVersion] return: %s",
										//       version.str().c_str());
										return version.str();
									}
								}
							}
						}
					}
				}
			}

			ROS_ERROR("[MotorSkidSteerDrive::getVersion] unexpected long string");
			throw new TRoboClawException("[MotorSkidSteerDrive::getVersion] unexpected long string");
		} catch (TRoboClawException* e) {
			ROS_ERROR_COND(DEBUG, "[MotorSkidSteerDrive::getVersion] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorSkidSteerDrive::getVersion] Uncaught exception !!!");
		}
	}


	ROS_ERROR("<----- [MotorSkidSteerDrive::getVersion] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorSkidSteerDrive::getVersion] RETRY COUNT EXCEEDED");
}

void MotorSkidSteerDrive::queueThread() {
	static const double timeout = 0.01;

    ROS_INFO("[MotorSkidSteerDrive::queueThread] startup");
	while (alive && rosNode->ok()) {
		queue.callAvailable(ros::WallDuration(timeout));
	}

	ROS_INFO("[MotorSkidSteerDrive::queueThread] shutdown");
}

void MotorSkidSteerDrive::robotMotorControllerThread() {
    ROS_INFO("[MotorSkidSteerDrive::robotMotorControllerThread] startup");
    while (1) {
	geometry_msgs::Twist cmd_msg;
	twistQueue.Consume(cmd_msg);
	float velocity = cmd_msg.linear.x;
	float angle = cmd_msg.angular.z;
	try {
	  drive(velocity, angle);
	} catch (...) {
	    ROS_ERROR("[MotorSkidSteerDrive::drive] Uncaught exception !!!");
	}

        usleep(1000);
    }
    
    ROS_INFO("[MotorSkidSteerDrive::robotMotorControllerThread] shutdown");
}

void MotorSkidSteerDrive::roboClawStatusReaderThread() {
    ROS_INFO("[MotorSkidSteerDrive::roboClawStatusReaderThread] startup");
    uint32_t sequenceCount = 0;
	ros::Publisher statusPublisher = rosNode->advertise<motor_controller::RoboClawStatus>("/RoboClawStatus", 1);
	ros::Rate rate(1);
	uint32_t counter = 0;
	roboClawStatus.firmwareVersion = getVersion();
	while (rosNode->ok()) {
		try {
		    //ROS_INFO_COND(DEBUG, "[MotorSkidSteerDrive::roboClawStatusReaderThread] sequence: %d", sequenceCount++);
			uint8_t errorStatus = getErrorStatus();
			roboClawStatus.errorStatus = errorStatus;
			roboClawStatus.stickyErrorStatus |= errorStatus;
			if (errorStatus == 0) roboClawStatus.errorString = "normal";
			else {
				stringstream errorMessage;
				if (errorStatus & 0x80) {
					errorMessage << "[Logic Battery Low] ";
				}

				if (errorStatus & 0x40) {
					errorMessage << "[Logic Battery High] ";
				}

				if (errorStatus & 0x20) {
					errorMessage << "[Main Battery Low] ";
				}

				if (errorStatus & 0x10) {
					errorMessage << "[Main Battery High] ";
				}

				if (errorStatus & 0x08) {
					errorMessage << "[Temperature] ";
				}

				if (errorStatus & 0x04) {
					errorMessage << "[E-Stop] ";
				}

				if (errorStatus & 0x02) {
					errorMessage << "[M2 OverCurrent] ";
				}

				if (errorStatus & 0x01) {
					errorMessage << "[M1 OverCurrent] ";
				}

				if (errorStatus & 0xFF00) {
					errorMessage << "[INVALID EXTRA STATUS BITS]";
				}

				roboClawStatus.errorString = errorMessage.str();
			}

			roboClawStatus.logicBatteryVoltage = getLogicBatteryLevel();
			roboClawStatus.mainBatteryVoltage = getMainBatteryLevel();
			TMotorCurrents motorCurrents = getMotorCurrents();
			roboClawStatus.m1MotorCurrent = motorCurrents.m1Current;
			roboClawStatus.m2MotorCurrent = motorCurrents.m2Current;
			
			TPIDQ pidq = getM1PIDQ();
			roboClawStatus.m1P = pidq.p / 65536.0;
			roboClawStatus.m1I = pidq.i / 65536.0;
			roboClawStatus.m1D = pidq.d / 65536.0;
			roboClawStatus.m1Qpps = pidq.q;
			
			pidq = getM2PIDQ();
			roboClawStatus.m2P = pidq.p / 65536.0;
			roboClawStatus.m2I = pidq.i / 65536.0;
			roboClawStatus.m2D = pidq.d / 65536.0;
			roboClawStatus.m2Qpps = pidq.q;

            {
                boost::mutex::scoped_lock lock(roboClawLock);
                EncodeResult encoder = getEncoderCommandResult(GETM1ENC);
                roboClawStatus.encoderM1value = encoder.value;
                roboClawStatus.encoderM1Status = encoder.status;
            }

            {
                boost::mutex::scoped_lock lock(roboClawLock);
                EncodeResult encoder = getEncoderCommandResult(GETM2ENC);
                roboClawStatus.encoderM2value = encoder.value;
                roboClawStatus.encoderM2Status = encoder.status;
            }
			
			roboClawStatus.expectedM1Speed = expectedM1Speed;
			roboClawStatus.expectedM2Speed = expectedM2Speed;
			roboClawStatus.currentM1Speed = getM1Speed();
			roboClawStatus.currentM2Speed = getM2Speed();
			roboClawStatus.maxM1Distance = maxM1Distance;
			roboClawStatus.maxM1Distance = maxM2Distance;
			roboClawStatus.m1MovingForward = m1MovingForward;
			roboClawStatus.m2MovingForward = m2MovingForward;
			
			lastTime = ros::Time::now();

			// stringstream ss;
			// ss << "No error, counter: " << counter++;
			// roboClawStatus.firmwareVersion = ss.str();
			statusPublisher.publish(roboClawStatus);
			rate.sleep();
		} catch (TRoboClawException* e) {
			ROS_ERROR_COND(DEBUG, "[MotorSkidSteerDrive::roboClawStatusReaderThread] Exception: %s", e->what());
		} catch (...) {
		    ROS_ERROR("[MotorSkidSteerDrive::roboClawStatusReaderThread] Uncaught exception !!!");
		}
	}
	
	ROS_INFO("[MotorSkidSteerDrive::roboClawStatusReaderThread] shutdown");
}

uint8_t MotorSkidSteerDrive::readByteWithTimeout() {
	//ROS_INFO_COND(DEBUG, "[MotorSkidSteerDrive::readByteWithTimeout]");
	struct pollfd ufd[1];
	ufd[0].fd = clawPort;
	ufd[0].events = POLLIN;

	int retval = poll(ufd, 1, 11);
	if (retval < 0) {
		ROS_ERROR("[MotorSkidSteerDrive::readByteWithTimeout] Poll failed (%d) %s", errno, strerror(errno));
		throw new TRoboClawException("[MotorSkidSteerDrive::readByteWithTimeout] Read error");
	} else if (retval == 0) {
		stringstream ev;
		ev << "[MotorSkidSteerDrive::readByteWithTimeout] TIMEOUT revents: " << hex << ufd[0].revents;
		ROS_ERROR_STREAM_COND(DEBUG, ev.str());
		throw new TRoboClawException("[MotorSkidSteerDrive::readByteWithTimeout] TIMEOUT");
	} else if (ufd[0].revents & POLLERR) {
		ROS_ERROR("[MotorSkidSteerDrive::readByteWithTimeout] Error on socket");
		restartPort();
		throw new TRoboClawException("[MotorSkidSteerDrive::readByteWithTimeout] Error on socket");
	} else if (ufd[0].revents & POLLIN) {
		char buffer[1];
		int bytesRead = read(clawPort, buffer, sizeof(buffer));
		if (bytesRead != 1) {
			ROS_ERROR("[MotorSkidSteerDrive::readByteWithTimeout] Failed to read 1 byte, read: %d", bytesRead);
			throw TRoboClawException("[MotorSkidSteerDrive::readByteWithTimeout] Failed to read 1 byte");
		}

		//ROS_INFO_COND(DEBUG, "[MotorSkidSteerDrive::readByteWithTimeout] ...result: 0x%X", int(buffer[0]));
		return buffer[0];
	} else {
		ROS_ERROR("[MotorSkidSteerDrive::readByteWithTimeout] Unhandled case");
		throw new TRoboClawException("[MotorSkidSteerDrive::readByteWithTimeout] Unhandled case");
	}
}

void MotorSkidSteerDrive::openPort() {
	ROS_WARN("[MotorSkidSteerDrive::openPort] about to open port: %s", motorUSBPort.c_str());
	clawPort = open(motorUSBPort.c_str(), O_RDWR | O_NOCTTY);
	if (clawPort < 0) {
		ROS_ERROR("[MotorSkidSteerDrive::openPort] Unable to open USB port, errno: (%d) %s", errno, strerror(errno));
		throw new TRoboClawException("[MotorSkidSteerDrive::openPort] Unable to open USB port");
	}


    // Fetch the current port settings.
	struct termios portOptions;
	int ret = 0;

 	ret = tcgetattr(clawPort, &portOptions);
	if (ret < 0) {
		ROS_ERROR("[MotorSkidSteerDrive::openPort] Unable to get terminal options (tcgetattr), error: %d: %s", errno, strerror(errno));
		// throw new TRoboClawException("[MotorSkidSteerDrive::openPort] Unable to get terminal options (tcgetattr)");
	}

    //memset(&portOptions.c_cc, 0, sizeof(portOptions.c_cc));

    // c_cflag contains a few important things- CLOCAL and CREAD, to prevent
    //   this program from "owning" the port and to enable receipt of data.
    //   Also, it holds the settings for number of data bits, parity, stop bits,
    //   and hardware flow control. 
    portOptions.c_cflag &= ~HUPCL;
    portOptions.c_iflag |= BRKINT;
    portOptions.c_iflag |= IGNPAR;
    portOptions.c_iflag &= ~ICRNL;
    portOptions.c_oflag &= ~OPOST;
    portOptions.c_lflag &= ~ISIG;
    portOptions.c_lflag &= ~ICANON;
    portOptions.c_lflag &= ~ECHO;

    portOptions.c_cc[VKILL] = 8;
    portOptions.c_cc[VMIN] = 100;
    portOptions.c_cc[VTIME] = 2;
    
    if (cfsetispeed(&portOptions, B38400) < 0) {
		ROS_ERROR("[MotorSkidSteerDrive::openPort] Unable to set terminal speed (cfsetispeed)");
		throw new TRoboClawException("[MotorSkidSteerDrive::openPort] Unable to set terminal speed (cfsetispeed)");
    }

    if (cfsetospeed(&portOptions, B38400) < 0) {
		ROS_ERROR("[MotorSkidSteerDrive::openPort] Unable to set terminal speed (cfsetospeed)");
		throw new TRoboClawException("[MotorSkidSteerDrive::openPort] Unable to set terminal speed (cfsetospeed)");
    }

    // portOptions.c_iflag = IGNPAR;
    // portOptions.c_oflag = 0;
    // portOptions.c_lflag = 0;
    // usleep(200000);
    // tcflush(clawPort, TCIOFLUSH);

    // Now that we've populated our options structure, let's push it back to the system.
    if (tcsetattr(clawPort, TCSANOW, &portOptions) < 0) {
		ROS_ERROR("[MotorSkidSteerDrive::openPort] Unable to set terminal options (tcsetattr)");
		throw new TRoboClawException("[MotorSkidSteerDrive::openPort] Unable to set terminal options (tcsetattr)");
    }
}

void MotorSkidSteerDrive::restartPort() {
    // ROS_ERROR("-----> [MotorSkidSteerDrive::restartPort]");
    // int result = ioctl(clawPort, USBDEVFS_RESET, 0);
    // ROS_ERROR("[MotorSkidSteerDrive::restartPort] ioctl result: %d", result);
    close(clawPort);
    usleep(200000);
    openPort();
    // ROS_ERROR("<----- [MotorSkidSteerDrive::restartPort] ioctl result: %d", result);
}

void MotorSkidSteerDrive::setM1PID(float p, float i, float d, uint32_t qpps) {
	boost::mutex::scoped_lock lock(roboClawLock);
	//ROS_INFO_COND(DEBUG, "[MotorSkidSteerDrive::setM1PID] p: %f, i: %f, d: %f, qpps: %d", p, i, d, qpps);
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			uint32_t kp = int(p * 65536.0); // 14834322.6368 = E25A93
			uint32_t ki = int(i * 65536.0);
			uint32_t kd = int(d * 65536.0);
			writeN(true, 18, portAddress, SETM1PID, 
				   SetDWORDval(kd),
				   SetDWORDval(kp),
				   SetDWORDval(ki),
				   SetDWORDval(qpps));
        	//ROS_INFO_COND(DEBUG, "<----- [MotorSkidSteerDrive::setM1PID]");
			return;
		} catch (TRoboClawException* e) {
			ROS_ERROR_COND(DEBUG, "[MotorSkidSteerDrive::setM1PID] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorSkidSteerDrive::setM1PID] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [MotorSkidSteerDrive::setM1PID] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorSkidSteerDrive::setM1PID] RETRY COUNT EXCEEDED");
}

void MotorSkidSteerDrive::setM2PID(float p, float i, float d, uint32_t qpps) {
	boost::mutex::scoped_lock lock(roboClawLock);
	//ROS_INFO_COND(DEBUG, "[MotorSkidSteerDrive::setM2PID] p: %f, i: %f, d: %f, qpps: %d", p, i, d, qpps);
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			uint32_t kp = int(p * 65536.0); // 14834322.6368 = E25A93
			uint32_t ki = int(i * 65536.0);
			uint32_t kd = int(d * 65536.0);
			writeN(true, 18, portAddress, SETM2PID, 
				   SetDWORDval(kd),
				   SetDWORDval(kp),
				   SetDWORDval(ki),
				   SetDWORDval(qpps));
        	//ROS_INFO_COND(DEBUG, "<----- [MotorSkidSteerDrive::setM2PID]");
			return;
		} catch (TRoboClawException* e) {
			ROS_ERROR_COND(DEBUG, "[MotorSkidSteerDrive::setM2PID] Exception: %s, retry number: %d",  e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[MotorSkidSteerDrive::setM2PID] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [MotorSkidSteerDrive::setM2PID] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorSkidSteerDrive::setM2PID] RETRY COUNT EXCEEDED");
}

// Command motors to a given linear and angular velocity
void MotorSkidSteerDrive::setVelocities(double v, double w, int32_t* left_qpps, int32_t* right_qpps) {
    // ROS_INFO_COND(DEBUG, "[MotorSkidSteerDrive::setVelocities], v: %f, w: %f, left_qpps: %ld, right_qpps: %ld",
    //           v,
    //           w,
    //           (long int) left_qpps,
    //           (long int) right_qpps);
	double wmag = fabs(w);
	double vmag = fabs(v);

    /* Limit slow turning when you have no forward velocity
    if (vmag < 0.1) {
      if (wmag < 0.15) {
        w = 0.0;
      } else if (wmag < 0.5) {
        w = copysign(0.5, w);
      }
    }*/

    // Reset error terms if large change in velocities or stopping.
    // if (fabs(state_.v_sp - v) > pid_error_reset_v_step_threshold_ || 
    //     fabs(state_.w_sp - w) > pid_error_reset_w_step_threshold_ ||
    //     (vmag < pid_error_reset_min_v_ && wmag < pid_error_reset_min_w_)) {
    //   pid_left_.reset();
    //   pid_right_.reset();
    // }

    // state_.v_sp = v;
    // state_.w_sp = w;

	double left_sp;
	double right_sp;
    vwToWheelSpeed(v, w, &left_sp, &right_sp);
    // Convert speeds to quad pulses per second
    *left_qpps = static_cast<int32_t>(round(left_sp * quad_pulse_per_meter_));
	*right_qpps = static_cast<int32_t>(round(right_sp * quad_pulse_per_meter_));

    // Compute kinematic feedforward control input
    // state_.left_duty_sp = state_.left_qpps_sp * duty_per_qpps_;
    // state_.right_duty_sp = state_.right_qpps_sp * duty_per_qpps_;
}

void MotorSkidSteerDrive::stop() {
	boost::mutex::scoped_lock lock(roboClawLock);
	ROS_INFO_COND(DEBUG, "-----> [MotorSkidSteerDrive::stop] command: 0x%X", MIXEDSPEED);
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			writeN(true, 10, portAddress, MIXEDSPEED,
				SetDWORDval(0),
				SetDWORDval(0)
				);
            //ROS_INFO_COND(DEBUG, "<----- [MotorSkidSteerDrive::stop]");
            expectedM1Speed = 0;
            expectedM2Speed = 0;
            maxM1Distance = 0;
            maxM2Distance = 0;
			return;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[MotorSkidSteerDrive::stop] Exception: %s, retry number: %d", e->what(), retry);
			restartPort();
		} catch (...) {
		    ROS_ERROR("[MotorSkidSteerDrive::stop] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [MotorSkidSteerDrive::stop] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[MotorSkidSteerDrive::stop] RETRY COUNT EXCEEDED");
}

void MotorSkidSteerDrive::updateOdometryThread() {
    ROS_INFO_COND(DEBUG, "[MotorSkidSteerDrive::updateOdometryThread] startup");
	ros::Publisher odometryPublisher = rosNode->advertise<nav_msgs::Odometry>("odom", 50);
	tf::TransformBroadcaster odomBroadcaster;

	uint32_t sequenceCount = 0;
    ros::Time currentTime;
	ros::Time lastTime = ros::Time::now();

	// TODO: handle encoder overflow. ###
	int32_t lastM1Encoder = getM1Encoder();
	int32_t lastM2Encoder = getM2Encoder();
	nav_msgs::Odometry odom;
	ros::Rate rate(100);
	int retry;
	
	while (rosNode->ok()) {
		try {
		    ros::spinOnce();
		    currentTime = ros::Time::now();
		    double dt = (currentTime - lastTime).toSec();
  		    lastTime = currentTime;

		    int32_t m1Encoder = getM1Encoder();
		    int32_t m2Encoder = getM2Encoder();
		    double delta_distance_left_meters = (m1Encoder - lastM1Encoder) * calibration_correction_factor_ / quad_pulse_per_meter_;
		    double delta_distance_right_meters = (m2Encoder - lastM2Encoder) * calibration_correction_factor_ / quad_pulse_per_meter_;
		    lastM1Encoder = m1Encoder;
		    lastM2Encoder = m2Encoder;

		    double left_meters_per_sec = delta_distance_left_meters / dt;
		    double right_meters_per_sec = delta_distance_right_meters / dt;
		    double linear_speed_meters_per_sec = (left_meters_per_sec + right_meters_per_sec) / 2.0;
		    double center_dist_traveled_meters = linear_speed_meters_per_sec * dt;
		    double omega_r = (right_meters_per_sec - left_meters_per_sec) / 2.0;
		    double d_theta = (delta_distance_right_meters - delta_distance_left_meters) / axle_width_;
		    double r;
		    double robot_pose_vx_ = linear_speed_meters_per_sec * cos(theta_);
		    double robot_pose_vy_ = linear_speed_meters_per_sec * sin(theta_);
		    robot_pose_px_ += robot_pose_vx_ * dt;
		    robot_pose_py_ += robot_pose_vy_ * dt;
		 //    if (delta_distance_left_meters == delta_distance_right_meters) {
		 //    	d_theta = 0.0;
			//     robot_pose_px_ += center_dist_traveled_meters * cos(theta_);
			//     robot_pose_py_ += center_dist_traveled_meters * sin(theta_);
		 //    } else {
		 //    	r = center_dist_traveled_meters / d_theta;
			//     robot_pose_px_ += r * (sin(d_theta + theta_) - sin(theta_));
			//     robot_pose_py_ += r * (cos(d_theta + theta_) - cos(theta_));
			// }

		    theta_ += normalizeRadians(d_theta);

		    // Compute position;
		    // robot_pose_px_ += robot_pose_vx_ * dt;
		    // robot_pose_py_ += robot_pose_vy_ * dt;

		    double vth = d_theta / dt;
		    
		    ROS_INFO_COND(DEBUG, 
        	               "[MotorSkidSteerDrive::updateOdometryThread] "
        	               	  " lin m/s: %7.4f"
        	               	  ", ctr d: %7.4f"
        	                  ", d_theta: %7.4f"
        	                  ", r: %7.4f"
        	                  ", x: %7.4f"
        	                  ", y: %7.4f"
        	                  ", d_theta: %7.4f"
        	                  ", theta: %7.4f"
        	                  ", sequence: %d"
        	                  ", dt: %5.3f"
        	                  ", m1Encoder: %d"
        	                  ", delta_distance_left_meters: %7.4f"
        	                  ", m2Encoder: %d"
        	                  ", delta_distance_right_meters: %7.4f",
        	               linear_speed_meters_per_sec,
        	               center_dist_traveled_meters,
        	               d_theta,
        	               r,
        	               robot_pose_px_,
        	               robot_pose_py_,
        	               d_theta,
        	               theta_,
        	               sequenceCount++,
        	               dt,
        	               m1Encoder,
        	               delta_distance_left_meters,
        	               m2Encoder,
        	               delta_distance_right_meters);
        	ROS_INFO_COND(DEBUG, 
        	               "[MotorSkidSteerDrive::updateOdometryThread] "
        	                  "robot_pose_vx_: %7.4f"
        	                  ", robot_pose_vy_: %7.4f"
        	                  ", vth: %7.4f",
        	               robot_pose_vx_,
        	               robot_pose_vy_,
        	               vth);
		    
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_);
		    odom.pose.pose.position.x = robot_pose_px_;
		    odom.pose.pose.position.y = robot_pose_py_;
		    odom.pose.pose.position.z = 0.0;
		    odom.pose.pose.orientation = odom_quat;
		    odom.pose.covariance[0] = 0.0001;
		    odom.pose.covariance[7] = 0.0001;
		    odom.pose.covariance[14] = 1000000.0;
		    odom.pose.covariance[21] = 1000000.0;
		    odom.pose.covariance[28] = 1000000.0;
		    odom.pose.covariance[35] = 0.03;
		    odom.twist.twist.linear.x = robot_pose_vx_;
		    odom.twist.twist.linear.y = robot_pose_vy_;
		    odom.twist.twist.linear.z = 0.0;
		    odom.twist.twist.angular.z = vth;
		    // odom.twist.covariance[0] = 0.0001;
		    // odom.twist.covariance[7] = 0.0001;
		    // odom.twist.covariance[14] = 0.0001;
		    // odom.twist.covariance[21] = 1000000.0;
		    // odom.twist.covariance[28] = 1000000.0;
		    // odom.twist.covariance[35] = 0.03;
		    currentTime = ros::Time::now();
		    odom.header.stamp = currentTime; //###
		    odom.header.frame_id = "odom";
		    odom.child_frame_id = "base_footprint";
		    
		    //odom.twist.twist.linear.y = deltaY / dt;

            odometryPublisher.publish(odom);
            
            if (do_odom_tf_) {
	            geometry_msgs::TransformStamped odomTrans;
	            odomTrans.header.stamp = currentTime;
	            odomTrans.header.frame_id = "odom";
	            odomTrans.child_frame_id = "base_footprint";
	            odomTrans.transform.translation.x = robot_pose_px_;
	            odomTrans.transform.translation.y = robot_pose_py_;
	            odomTrans.transform.translation.z = 0.0;
	            odomTrans.transform.rotation = odom_quat;
            	odomBroadcaster.sendTransform(odomTrans);
            }

		    rate.sleep();
		} catch (TRoboClawException* e) {
			ROS_ERROR_COND(DEBUG, "[MotorSkidSteerDrive::updateOdometryThread] Exception: %s", e->what());
		} catch (...) {
		    ROS_ERROR("[MotorSkidSteerDrive::updateOdometryThread] Uncaught exception !!!");
		}
	}
}

// Convert linear / angular velocity to left / right motor speeds in meters / second
void MotorSkidSteerDrive::vwToWheelSpeed(double v, double w, double *left_mps, double *right_mps) {
    // Compute the differential drive speeds from the input
    *left_mps = v - (axle_width_ / 2.0) * w;
    *right_mps = v + (axle_width_ / 2.0) * w;

    // Scale the speeds to respect the wheel speed limit
    double limitk = 1.0;
    if (fabs(*left_mps) > max_wheel_vel_) {
      limitk = max_wheel_vel_ / fabs(*left_mps);
    }

    if (fabs(*right_mps) > max_wheel_vel_) {
      double rlimitk = max_wheel_vel_ / fabs(*right_mps);
      if (rlimitk < limitk) {
        limitk = rlimitk;
      }
    }

    if (limitk != 1.0) {
      *left_mps *= limitk;
      *right_mps *= limitk;
    }

    // Deal with min limits
    if (fabs(*left_mps) < min_wheel_vel_) {
      *left_mps = 0.0;
    } if (fabs(*right_mps) < min_wheel_vel_) {
      *right_mps = 0.0;
    }
}

void MotorSkidSteerDrive::writeByte(uint8_t byte) {
	//ROS_INFO_COND(DEBUG, "[MotorSkidSteerDrive::writeByte] byte: 0x%X", byte);
	ssize_t result = write(clawPort, &byte, 1);
	if (result != 1) {
	  ROS_ERROR("[MotorSkidSteerDrive::writeByte] Unable to write one byte, result: %d, errno: %d)", (int) result,  errno);
		restartPort();
		throw new TRoboClawException("[MotorSkidSteerDrive::writeByte] Unable to write one byte");
	}
}

void MotorSkidSteerDrive::writeN(bool sendCRC, uint8_t cnt, ...) {
	uint16_t crc = 0;
	va_list marker;
	va_start(marker, cnt);

	int origFlags = fcntl(clawPort, F_GETFL, 0);
	fcntl(clawPort, F_SETFL, origFlags & ~O_NONBLOCK);

   	// usleep(20000);

	for (uint8_t i = 0; i < cnt; i++) {
		uint8_t byte = va_arg(marker, int);
		writeByte(byte);
		updateCrc(crc, byte);
	}

	va_end(marker);

	if (sendCRC) {
		writeByte(crc >> 8);
		writeByte(crc);

		uint8_t response = readByteWithTimeout();
		if (response != 0xFF) {
			ROS_ERROR("[MotorSkidSteerDrive::writeN] Invalid ACK response");
			throw new TRoboClawException("[MotorSkidSteerDrive::writeN] Invalid ACK response");
		}
	}

	fcntl(clawPort, F_SETFL, origFlags);
}


