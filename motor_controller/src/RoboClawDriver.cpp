
// roslaunch motor_controller motor.launch
// rosrun teleop_twist_keyboard teleop_twist_keyboard.py
// rostopic echo /RoboClawStatus
// rostopic echo /cmd_vel

// On exception, clear queue by waiting 10ms.

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

#include "motor_controller/RoboClawDriver.h"

#define SetDWORDval(arg) (uint8_t)(arg>>24),(uint8_t)(arg>>16),(uint8_t)(arg>>8),(uint8_t)arg


RoboClawDriver::RoboClawDriver()
	: debug_(false)
	, expectedM1Velocity_(0)
    , expectedM2Velocity_(0)
    , m1MovingForward_(false)
    , m2MovingForward_(false)
    , maxM1Distance_(0)
    , maxM2Distance_(0)
    , maximumWheelVelocity_(0.4)
    , minimumWheelVelocity_(0.0)
 	{
    ROS_INFO_COND(debug_, "[RoboClawDriver::RoboClawDriver] constructor");

	portAddress = 0x80;
	MAX_COMMAND_RETRIES = 5;

	m1P_ =  8762.98571;
	m2P_ = 9542.41265;
	m1I_ = 1535.49646;
	m2I_ = 1773.65086;
	m1Qpps_ = 3562;
	m2Qpps_ = 3340;
	axleWidth_ =  0.285; //0.255 //0.21166666666667

	assert(ros::param::get("~cmd_vel_topic", cmdVelTopicName_));
	assert(ros::param::get("~motor_usb_port", roboClawDeviceName_));

	ROS_INFO("[RoboClawDriver] PARAM cmdVelTopicName_: %s", cmdVelTopicName_.c_str());
	ROS_INFO("[RoboClawDriver] PARAM roboClawDeviceName_: %s", roboClawDeviceName_.c_str());

	openPort();

	stop();

	setM1PID(m1P_, m1I_, 0, m1Qpps_);
	setM2PID(m2P_, m2I_, 0, m2Qpps_);
	ROS_INFO_COND(debug_, "[RoboClawDriver::RoboClawDriver] Starting");
	ROS_INFO("[RoboClawDriver::RoboClawDriver] getLogicBatteryLevel %f", getLogicBatteryLevel());

	ros::SubscribeOptions so = 
		ros::SubscribeOptions::create<geometry_msgs::Twist>(
				cmdVelTopicName_,
				1,
				boost::bind(&RoboClawDriver::cmdVelCallback, this, _1),
				ros::VoidPtr(),
				&cmdVelMessageQueue_
			);

	cmdVelQueueHandlerThread_ = boost::thread(boost::bind(&RoboClawDriver::cmdVelQueueHandler, this));
	roboClawStatusReaderThread_ = boost::thread(boost::bind(&RoboClawDriver::roboClawStatusReader, this));
	handleMotionCommandsThead_ = boost::thread(boost::bind(&RoboClawDriver::handleMotionCommands, this));

	cmdVelSubscriber_ = nh_.subscribe(so);

}

RoboClawDriver::~RoboClawDriver() {
	boost::mutex::scoped_lock lock(roboClawLock_);
	if (clawPort_) {
		flush();
		close(clawPort_);
	}

}

void RoboClawDriver::cmdVelQueueHandler() {
	static const double timeout = 0.01;

    ROS_INFO("[RoboClawDriver::cmdVelQueueHandler] startup");
	while (ros::ok()) {
		cmdVelMessageQueue_.callAvailable(ros::WallDuration(timeout));
	}

	ROS_INFO("[RoboClawDriver::cmdVelQueueHandler] shutdown");
}


void RoboClawDriver::roboClawStatusReader() {
    ROS_INFO("[RoboClawDriver::roboClawStatusReader] startup");
    uint32_t sequenceCount = 0;
	ros::Publisher statusPublisher = nh_.advertise<motor_controller::RoboClawStatus>("/RoboClawStatus", 1);
	ros::Rate rate(1);
	roboClawStatus.firmwareVersion = getVersion();

	while (ros::ok()) {
		try {
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
                boost::mutex::scoped_lock lock(roboClawLock_);
                EncodeResult encoder = getEncoderCommandResult(GETM1ENC);
                roboClawStatus.encoderM1value = encoder.value;
                roboClawStatus.encoderM1Status = encoder.status;
            }

            {
                boost::mutex::scoped_lock lock(roboClawLock_);
                EncodeResult encoder = getEncoderCommandResult(GETM2ENC);
                roboClawStatus.encoderM2value = encoder.value;
                roboClawStatus.encoderM2Status = encoder.status;
            }
			
			roboClawStatus.expectedM1Speed = expectedM1Velocity_;
			roboClawStatus.expectedM2Speed = expectedM2Velocity_;
			roboClawStatus.currentM1Speed = getM1Velocity();
			roboClawStatus.currentM2Speed = getM2Velocity();
			roboClawStatus.maxM1Distance = maxM1Distance_;
			roboClawStatus.maxM2Distance = maxM2Distance_;
			
			statusPublisher.publish(roboClawStatus);
			rate.sleep();
		} catch (RoboClawDriverException* e) {
			ROS_ERROR_COND(debug_, "[RoboClawDriver::roboClawStatusReader] Exception: %s", e->what());
		} catch (...) {
		    ROS_ERROR("[RoboClawDriver::roboClawStatusReader] Uncaught exception !!!");
		}
	}
	
	ROS_INFO("[RoboClawDriver::roboClawStatusReader] shutdown");
}


void RoboClawDriver::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg) {
	ROS_INFO_COND(debug_,
				  "[RoboClawDriver::cmdVelCallback] cmd_msg.linear.x: %f, cmd_msg.angular.z: %f",
				  cmd_msg->linear.x,
				  cmd_msg->angular.z);
	geometry_msgs::Twist copy;
	copy.linear.x = cmd_msg->linear.x;
	copy.angular.z = cmd_msg->angular.z;
	twistQueue_.Produce(copy);
}

void RoboClawDriver::configCallback(motor_controller::motor_controllerConfig &config, uint32_t level) {
	ROS_INFO("[RoboClawDriver::configCallback]\n\tlevel: %d\n\tcmd_vel_topic: %s\n\tKB_velocity:  %f\n\tmotorUSBPort: %s\n", 
			 level,
			 config.cmd_vel_topic.c_str(),
			 config.KP_velocity,
			 config.motor_usb_port.c_str());
	if (m1P_ != config.M1_P) {
		m1P_ = config.M1_P;
		ROS_INFO("[RoboClawDriver::configCallback] setting new m1P_ value: %f", m1P_);
		setM1PID(m1P_, m1I_, 0, m1Qpps_);
	}

	if (m1I_ != (float) config.M1_I) {
		m1I_ = (float) config.M1_I;
		ROS_INFO("[RoboClawDriver::configCallback] setting new m1I_ value: %f", m1I_);
		setM1PID(m1I_, m1I_, 0, m1Qpps_);
	}

	if (m1Qpps_ != (float) config.M1_QPPS) {
		m1Qpps_ = (float) config.M1_I;
		ROS_INFO("[RoboClawDriver::configCallback] setting new m1Qpps_ value: %ld", m1Qpps_);
		setM1PID(m1P_, m1I_, 0, m1Qpps_);
	}

	if (m2P_ != (float) config.M2_P) {
		m2P_ = (float) config.M2_P;
		ROS_INFO("[RoboClawDriver::configCallback] setting new m2P_ value: %f", m2P_);
		setM2PID(m2P_, m2I_, 0, m2Qpps_);
	}

	if (m2I_ != (float) config.M2_I) {
		m2I_ = (float) config.M2_I;
		ROS_INFO("[RoboClawDriver::configCallback] setting new m2I_ value: %f", m2I_);
		setM2PID(m2P_, m2I_, 0, m2Qpps_);
	}

	if (m2Qpps_ != config.M2_QPPS) {
		m2Qpps_ = config.M2_QPPS;
		ROS_INFO("[RoboClawDriver::configCallback] setting new m2Qpps_ value: %ld", m2Qpps_);
		setM2PID(m2P_, m2I_, 0, m2Qpps_);
	}

	if (axleWidth_ != config.AXLE_WIDTH) {
		axleWidth_ = config.AXLE_WIDTH;
		ROS_INFO("[RoboClawDriver::configCallback] setting new AXLE_WIDTH value: %f", axleWidth_);
	}
}


void RoboClawDriver::drive(float velocity, float angle) {
	boost::mutex::scoped_lock scoped_lock(roboClawLock_);
	ROS_INFO_COND(debug_, "-----> [RoboClawDriver::drive] velocity: %f, angle: %f", velocity, angle);
	float m1Velocity;
	float m2Velocity;
	vwToWheelVelocity(velocity, angle, &m1Velocity, &m2Velocity);
	float m1AbsVelocity = m1Velocity >= 0 ? m1Velocity : -m1Velocity;
	float m2AbsVelocity = m2Velocity >= 0 ? m2Velocity : -m2Velocity;
	float m1MaxDistance = m1AbsVelocity * kMAX_SECONDS_TRAVEL; // Limit travel.
	float m2MaxDistance = m2AbsVelocity * kMAX_SECONDS_TRAVEL; // Limit travel.
	ROS_INFO_STREAM_COND(debug_,
						 "[RoboClawDriver::drive] ---- command: " 
						 << MIXEDSPEEDDIST
						 << ", drive velocity: "  << velocity
						 << ", angle: " << angle
						 << ", m1Velocity: " << m1Velocity
						 << ", m1MaxDistance: " << m1MaxDistance
						 << ", m2Velocity: " << m2Velocity
						 << ", m2MaxDistance: " << m2MaxDistance);

	expectedM1Velocity_ = m1Velocity;
	expectedM2Velocity_ = m2Velocity;
	maxM1Distance_ = m1MaxDistance;
	maxM2Distance_ = m2MaxDistance;

	// ##### LOOK UP if qpps needs to be multiplied here
	int retry;
	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			writeN(true, 19, portAddress, MIXEDSPEEDDIST,
				   SetDWORDval((int) m1Velocity),
				   SetDWORDval((int) m1MaxDistance),
				   SetDWORDval((int) m2Velocity),
				   SetDWORDval((int) m2MaxDistance),
				   1 //Cancel any previous command
				   );
			return;
		} catch (RoboClawDriverException* e) {
			ROS_ERROR_COND(debug_, "[RoboClawDriver::drive] Exception: %s, retry number %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[RoboClawDriver::drive] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [RoboClawDriver::drive] RETRY COUNT EXCEEDED");
	throw new RoboClawDriverException("[RoboClawDriver::drive] RETRY COUNT EXCEEDED");
}


void RoboClawDriver::flush() {
	int retval = tcflush(clawPort_, TCIOFLUSH);
	if (retval != 0) {
		ROS_ERROR("[RoboClawDriver::flush] Unable to flush device");
		throw new RoboClawDriverException("[RoboClawDriver::flush] Unable to flush device");
	}
}


unsigned short RoboClawDriver::get2ByteCommandResult(uint8_t command) {
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

	if (debug_) ROS_ERROR("[RoboClawDriver::get2ByteCommandResult] Expected CRC of: 0x%X, but got: 0x%X",
	    int(crc),
	    int(responseCrc));
	throw new RoboClawDriverException("[RoboClawDriver::get2ByteCommandResult] INVALID CRC");
}


RoboClawDriver::EncodeResult RoboClawDriver::getEncoderCommandResult(uint8_t command) {
	uint16_t crc = 0;
	updateCrc(crc, portAddress);
	updateCrc(crc, command);

	writeN(false, 2, portAddress, command);
	EncodeResult result = {0, 0};
	uint8_t datum = readByteWithTimeout();
	if (datum == -1) {
		ROS_ERROR("[RoboClawDriver::getEncoderCommandResult] Received -1 instead of expected data");
		throw new RoboClawDriverException("[RoboClawDriver::::getEncoderCommandResult] RECEIVED -1");
	}

	result.value |= datum << 24;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	if (datum == -1) {
		ROS_ERROR("[RoboClawDriver::getEncoderCommandResult] Received -1 instead of expected data");
		throw new RoboClawDriverException("[RoboClawDriver::::getEncoderCommandResult] RECEIVED -1");
	}

	result.value |= datum << 16;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	if (datum == -1) {
		ROS_ERROR("[RoboClawDriver::getEncoderCommandResult] Received -1 instead of expected data");
		throw new RoboClawDriverException("[RoboClawDriver::::getEncoderCommandResult] RECEIVED -1");
	}

	result.value |= datum << 8;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	if (datum == -1) {
		ROS_ERROR("[RoboClawDriver::getEncoderCommandResult] Received -1 instead of expected data");
		throw new RoboClawDriverException("[RoboClawDriver::::getEncoderCommandResult] RECEIVED -1");
	}

	result.value |= datum;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	if (datum == -1) {
		ROS_ERROR("[RoboClawDriver::getEncoderCommandResult] Received -1 instead of expected data");
		throw new RoboClawDriverException("[RoboClawDriver::::getEncoderCommandResult] RECEIVED -1");
	}

	result.status |= datum;
	updateCrc(crc, datum);
	if (datum == -1) {
		ROS_ERROR("[RoboClawDriver::getEncoderCommandResult] Received -1 instead of expected data");
		throw new RoboClawDriverException("[RoboClawDriver::::getEncoderCommandResult] RECEIVED -1");
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

	if (debug_) {
		ROS_ERROR("[RoboClawDriver::getEncoderCommandResult] Expected CRC of: 0x%X, but got: 0x%X",
				   int(crc),
				   int(responseCrc));
	}

	throw new RoboClawDriverException("[RoboClawDriver::getEncoderCommandResult] INVALID CRC");
}


uint16_t RoboClawDriver::getErrorStatus() {
	boost::mutex::scoped_lock lock(roboClawLock_);
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
							return result;
						}
					}
				}
			}
		} catch (RoboClawDriverException* e) {
			ROS_ERROR_COND(debug_, "[RoboClawDriver::getErrorStatus] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[RoboClawDriver::getErrorStatus] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [RoboClawDriver::getErrorStatus] RETRY COUNT EXCEEDED");
	throw new RoboClawDriverException("[RoboClawDriver::getErrorStatus] RETRY COUNT EXCEEDED");
}


float RoboClawDriver::getLogicBatteryLevel() {
	boost::mutex::scoped_lock lock(roboClawLock_);
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			float result = ((float) get2ByteCommandResult(GETLBATT)) / 10.0;
			return result;
		} catch (RoboClawDriverException* e) {
			ROS_ERROR("[RoboClawDriver::getLogicBatteryLevel] Exception: %s, retry number: %d", e->what(), retry);
			restartRoboClaw();
		} catch (...) {
		    ROS_ERROR("[RoboClawDriver::getLogicBatteryLevel] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [RoboClawDriver::getLogicBatteryLevel] RETRY COUNT EXCEEDED");
	throw new RoboClawDriverException("[RoboClawDriver::getLogicBatteryLevel] RETRY COUNT EXCEEDED");
}


uint32_t RoboClawDriver::getULongCont(uint16_t& crc) {
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


int32_t RoboClawDriver::getM1Encoder() {
	boost::mutex::scoped_lock lock(roboClawLock_);
	//ROS_INFO_COND(debug_, "-----> [RoboClawDriver::getM1Encoder]");
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			EncodeResult result = getEncoderCommandResult(GETM1ENC);
	    	//ROS_INFO_COND(debug_, "<----- [RoboClawDriver::getM1Encode] return: %d/0x%X", result.value, result.status);
			return result.value;
		} catch (RoboClawDriverException* e) {
			ROS_ERROR_COND(debug_, "[RoboClawDriver::getM1Encode] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[RoboClawDriver::getM1Encoder] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [RoboClawDriver::getM1Encoder] RETRY COUNT EXCEEDED");
	throw new RoboClawDriverException("[RoboClawDriver::getM1Encoder] RETRY COUNT EXCEEDED");
}


int32_t RoboClawDriver::getM1Velocity() {
	boost::mutex::scoped_lock lock(roboClawLock_);
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			uint32_t result = getVelocityResult(GETM1SPEED);
		    if (result < 0) {
				m1MovingForward_ = false;
		    } else {
				m1MovingForward_ = true;
		    }

			return result;
		} catch (RoboClawDriverException* e) {
			ROS_ERROR_COND(debug_, "[RoboClawDriver::getM1Velocity] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[RoboClawDriver::getM1Velocity] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [RoboClawDriver::getM1Velocity] RETRY COUNT EXCEEDED");
	throw new RoboClawDriverException("[RoboClawDriver::getM1Velocity] RETRY COUNT EXCEEDED");
}

int32_t RoboClawDriver::getM2Encoder() {
	boost::mutex::scoped_lock lock(roboClawLock_);
	//ROS_INFO_COND(debug_, "-----> [RoboClawDriver::getM2Encoder]");
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			EncodeResult result = getEncoderCommandResult(GETM2ENC);
	    	//ROS_INFO_COND(debug_, "<----- [RoboClawDriver::getM2Encoder] return: %d/0x%X", result.value, result.status);
			return result.value;
		} catch (RoboClawDriverException* e) {
			ROS_ERROR_COND(debug_, "[RoboClawDriver::getM2Encoder] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[RoboClawDriver::getM2Encoder] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [RoboClawDriver::getM2Encoder] RETRY COUNT EXCEEDED");
	throw new RoboClawDriverException("[RoboClawDriver::getM2Encoder] RETRY COUNT EXCEEDED");
}


int32_t RoboClawDriver::getM2Velocity() {
	boost::mutex::scoped_lock lock(roboClawLock_);
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			uint32_t result = getVelocityResult(GETM2SPEED);
		    if (result < 0) {
				m2MovingForward_ = false;
		    } else {
				m2MovingForward_ = true;
		    }

			return result;
		} catch (RoboClawDriverException* e) {
			ROS_ERROR_COND(debug_, "[RoboClawDriver::getM2Velocity] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[RoboClawDriver::getM2Velocity] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [RoboClawDriver::getM2Velocity] RETRY COUNT EXCEEDED");
	throw new RoboClawDriverException("[RoboClawDriver::getM2Velocity] RETRY COUNT EXCEEDED");
}


float RoboClawDriver::getMainBatteryLevel() {
	boost::mutex::scoped_lock lock(roboClawLock_);
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			float result = ((float) get2ByteCommandResult(GETMBATT)) / 10.0;
			return result;
		} catch (RoboClawDriverException* e) {
			ROS_ERROR_COND(debug_, "[RoboClawDriver::getMainBatteryLevel] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[RoboClawDriver::getMainBatteryLevel] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [RoboClawDriver::getMainBatteryLevel] RETRY COUNT EXCEEDED");
	throw new RoboClawDriverException("[RoboClawDriver::getMainBatteryLevel] RETRY COUNT EXCEEDED");
}


RoboClawDriver::TMotorCurrents RoboClawDriver::getMotorCurrents() {
	boost::mutex::scoped_lock lock(roboClawLock_);
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			TMotorCurrents result;
			unsigned long currentPair = getUlongCommandResult(GETCURRENTS);
			result.m1Current = ((int16_t) (currentPair >> 16)) * 0.010;
			result.m2Current = ((int16_t) (currentPair & 0xFFFF)) * 0.010;
			return result;
		} catch (RoboClawDriverException* e) {
			ROS_ERROR_COND(debug_, "[RoboClawDriver::getMotorCurrents] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[RoboClawDriver::getMotorCurrents] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [RoboClawDriver::getMotorCurrents] RETRY COUNT EXCEEDED");
	throw new RoboClawDriverException("[RoboClawDriver::getMotorCurrents] RETRY COUNT EXCEEDED");
}


int32_t RoboClawDriver::getVelocityResult(uint8_t command) {
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

	if (debug_) ROS_ERROR("[RoboClawDriver::getVelocityResult] Expected CRC of: 0x%X, but got: 0x%X",
	    int(crc),
	    int(responseCrc));
	throw new RoboClawDriverException("[RoboClawDriver::getVelocityResult] INVALID CRC");
}


unsigned long RoboClawDriver::getUlongCommandResult(uint8_t command) {
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

	if (debug_) ROS_ERROR("[RoboClawDriver::getUlongCommandResult] Expected CRC of: 0x%X, but got: 0x%X",
	    int(crc),
	    int(responseCrc));
	throw new RoboClawDriverException("[RoboClawDriver::getUlongCommandResult] INVALID CRC");
}


RoboClawDriver::ULongPair RoboClawDriver::getULongPairCommandResult(uint8_t command) {
	uint16_t crc = 0;
	updateCrc(crc, portAddress);
	updateCrc(crc, command);

	writeN(false, 2, portAddress, command);
	uint32_t result1 = getULongCont(crc);
	uint32_t result2 = getULongCont(crc);

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

	if (debug_) ROS_ERROR("[RoboClawDriver::getUlongCommandResult] Expected CRC of: 0x%X, but got: 0x%X",
	    int(crc),
	    int(responseCrc));
	throw new RoboClawDriverException("[RoboClawDriver::getUlongCommandResult] INVALID CRC");
}


RoboClawDriver::TPIDQ RoboClawDriver::getM1PIDQ() {
	boost::mutex::scoped_lock lock(roboClawLock_);
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		TPIDQ result;
		try {
			uint16_t crc = 0;
			updateCrc(crc, portAddress);
			updateCrc(crc, GETM1PID);

			writeN(false, 2, portAddress, GETM1PID);
			result.p = (int32_t) getULongCont(crc);
			result.i = (int32_t) getULongCont(crc);
			result.d = (int32_t) getULongCont(crc);
			result.q = (int32_t) getULongCont(crc);

			uint16_t responseCrc = 0;
			uint16_t datum = readByteWithTimeout();
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
		} catch (RoboClawDriverException* e) {
			ROS_ERROR_COND(debug_, "[RoboClawDriver::getM1PIDQ] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[RoboClawDriver::getM1PIDQ] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [RoboClawDriver::getM1PIDQ] RETRY COUNT EXCEEDED");
	throw new RoboClawDriverException("[RoboClawDriver::getM1PIDQ] RETRY COUNT EXCEEDED");
}


RoboClawDriver::TPIDQ RoboClawDriver::getM2PIDQ() {
	boost::mutex::scoped_lock lock(roboClawLock_);
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		TPIDQ result;
		try {
			uint16_t crc = 0;
			updateCrc(crc, portAddress);
			updateCrc(crc, GETM2PID);

			writeN(false, 2, portAddress, GETM2PID);
			result.p = (int32_t) getULongCont(crc);
			result.i = (int32_t) getULongCont(crc);
			result.d = (int32_t) getULongCont(crc);
			result.q = (int32_t) getULongCont(crc);

			uint16_t responseCrc = 0;
			uint16_t datum = readByteWithTimeout();
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
		} catch (RoboClawDriverException* e) {
			ROS_ERROR_COND(debug_, "[RoboClawDriver::getM2PID] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[RoboClawDriver::getM2PIDQ] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [RoboClawDriver::getM2PIDQ] RETRY COUNT EXCEEDED");
	throw new RoboClawDriverException("[RoboClawDriver::getM2PIDQ] RETRY COUNT EXCEEDED");
}


string RoboClawDriver::getVersion() {
	boost::mutex::scoped_lock lock(roboClawLock_);
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
										return version.str();
									}
								}
							}
						}
					}
				}
			}

			ROS_ERROR("[RoboClawDriver::getVersion] unexpected long string");
			throw new RoboClawDriverException("[RoboClawDriver::getVersion] unexpected long string");
		} catch (RoboClawDriverException* e) {
			ROS_ERROR_COND(debug_, "[RoboClawDriver::getVersion] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[RoboClawDriver::getVersion] Uncaught exception !!!");
		}
	}


	ROS_ERROR("<----- [RoboClawDriver::getVersion] RETRY COUNT EXCEEDED");
	throw new RoboClawDriverException("[RoboClawDriver::getVersion] RETRY COUNT EXCEEDED");
}

void RoboClawDriver::openPort() {
	ROS_WARN("[RoboClawDriver::openPort] about to open port: %s", roboClawDeviceName_.c_str());
	clawPort_ = open(roboClawDeviceName_.c_str(), O_RDWR | O_NOCTTY);
	if (clawPort_ < 0) {
		ROS_ERROR("[RoboClawDriver::openPort] Unable to open USB port, errno: (%d) %s", errno, strerror(errno));
		throw new RoboClawDriverException("[RoboClawDriver::openPort] Unable to open USB port");
	}


    // Fetch the current port settings.
	struct termios portOptions;
	int ret = 0;

 	ret = tcgetattr(clawPort_, &portOptions);
	if (ret < 0) {
		ROS_ERROR("[RoboClawDriver::openPort] Unable to get terminal options (tcgetattr), error: %d: %s", errno, strerror(errno));
		throw new RoboClawDriverException("[RoboClawDriver::openPort] Unable to get terminal options (tcgetattr)");
	}

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
		ROS_ERROR("[RoboClawDriver::openPort] Unable to set terminal speed (cfsetispeed)");
		throw new RoboClawDriverException("[RoboClawDriver::openPort] Unable to set terminal speed (cfsetispeed)");
    }

    if (cfsetospeed(&portOptions, B38400) < 0) {
		ROS_ERROR("[RoboClawDriver::openPort] Unable to set terminal speed (cfsetospeed)");
		throw new RoboClawDriverException("[RoboClawDriver::openPort] Unable to set terminal speed (cfsetospeed)");
    }

    // Now that we've populated our options structure, let's push it back to the system.
    if (tcsetattr(clawPort_, TCSANOW, &portOptions) < 0) {
		ROS_ERROR("[RoboClawDriver::openPort] Unable to set terminal options (tcsetattr)");
		throw new RoboClawDriverException("[RoboClawDriver::openPort] Unable to set terminal options (tcsetattr)");
    }
}


void RoboClawDriver::handleMotionCommands() {
    ROS_INFO("[RoboClawDriver::handleMotionCommands] startup");
    while (ros::ok()) {
		geometry_msgs::Twist cmd_msg;
		twistQueue_.Consume(cmd_msg);
		float velocity = cmd_msg.linear.x;
		float angle = cmd_msg.angular.z;
		try {
		  drive(velocity, angle);
		} catch (...) {
		    ROS_ERROR("[RoboClawDriver::drive] Uncaught exception !!!");
		}

        usleep(1000);
    }
    
    ROS_INFO("[RoboClawDriver::handleMotionCommands] shutdown");
}


uint8_t RoboClawDriver::readByteWithTimeout() {
	//ROS_INFO_COND(debug_, "[RoboClawDriver::readByteWithTimeout]");
	struct pollfd ufd[1];
	ufd[0].fd = clawPort_;
	ufd[0].events = POLLIN;

	int retval = poll(ufd, 1, 11);
	if (retval < 0) {
		ROS_ERROR("[RoboClawDriver::readByteWithTimeout] Poll failed (%d) %s", errno, strerror(errno));
		throw new RoboClawDriverException("[RoboClawDriver::readByteWithTimeout] Read error");
	} else if (retval == 0) {
		stringstream ev;
		ev << "[RoboClawDriver::readByteWithTimeout] TIMEOUT revents: " << hex << ufd[0].revents;
		ROS_ERROR_STREAM_COND(debug_, ev.str());
		throw new RoboClawDriverException("[RoboClawDriver::readByteWithTimeout] TIMEOUT");
	} else if (ufd[0].revents & POLLERR) {
		ROS_ERROR("[RoboClawDriver::readByteWithTimeout] Error on socket");
		restartRoboClaw();
		throw new RoboClawDriverException("[RoboClawDriver::readByteWithTimeout] Error on socket");
	} else if (ufd[0].revents & POLLIN) {
		char buffer[1];
		int bytesRead = read(clawPort_, buffer, sizeof(buffer));
		if (bytesRead != 1) {
			ROS_ERROR("[RoboClawDriver::readByteWithTimeout] Failed to read 1 byte, read: %d", bytesRead);
			throw RoboClawDriverException("[RoboClawDriver::readByteWithTimeout] Failed to read 1 byte");
		}

		return buffer[0];
	} else {
		ROS_ERROR("[RoboClawDriver::readByteWithTimeout] Unhandled case");
		throw new RoboClawDriverException("[RoboClawDriver::readByteWithTimeout] Unhandled case");
	}
}


void RoboClawDriver::restartRoboClaw() {
    close(clawPort_);
    usleep(200000);
    openPort();
}


void RoboClawDriver::setM1PID(float p, float i, float d, uint32_t qpps) {
	boost::mutex::scoped_lock lock(roboClawLock_);
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			uint32_t kp = int(p * 65536.0); // 14834322.6368 = E25A93
			uint32_t ki = int(i * 65536.0);
			uint32_t kd = int(d * 65536.0);
			writeN(true, 
				   18, 
				   portAddress, 
				   SETM1PID, 
				   SetDWORDval(kd),
				   SetDWORDval(kp),
				   SetDWORDval(ki),
				   SetDWORDval(qpps));
			return;
		} catch (RoboClawDriverException* e) {
			ROS_ERROR_COND(debug_, "[RoboClawDriver::setM1PID] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[RoboClawDriver::setM1PID] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [RoboClawDriver::setM1PID] RETRY COUNT EXCEEDED");
	throw new RoboClawDriverException("[RoboClawDriver::setM1PID] RETRY COUNT EXCEEDED");
}


void RoboClawDriver::setM2PID(float p, float i, float d, uint32_t qpps) {
	boost::mutex::scoped_lock lock(roboClawLock_);
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			uint32_t kp = int(p * 65536.0); // 14834322.6368 = E25A93
			uint32_t ki = int(i * 65536.0);
			uint32_t kd = int(d * 65536.0);
			writeN(true, 
				   18, 
				   portAddress, 
				   SETM2PID, 
				   SetDWORDval(kd),
				   SetDWORDval(kp),
				   SetDWORDval(ki),
				   SetDWORDval(qpps));
			return;
		} catch (RoboClawDriverException* e) {
			ROS_ERROR_COND(debug_, "[RoboClawDriver::setM2PID] Exception: %s, retry number: %d",  e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[RoboClawDriver::setM2PID] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [RoboClawDriver::setM2PID] RETRY COUNT EXCEEDED");
	throw new RoboClawDriverException("[RoboClawDriver::setM2PID] RETRY COUNT EXCEEDED");
}


void RoboClawDriver::stop() {
	boost::mutex::scoped_lock lock(roboClawLock_);
	ROS_INFO_COND(debug_, "-----> [RoboClawDriver::stop] command: 0x%X", MIXEDSPEED);
	int retry;

	for (retry = 0; retry < MAX_COMMAND_RETRIES; retry++) {
		try {
			writeN(true, 
				   10, 
				   portAddress, 
				   MIXEDSPEED,
				   SetDWORDval(0),
				   SetDWORDval(0)
				   );
            expectedM1Velocity_ = 0;
            expectedM2Velocity_ = 0;
            maxM1Distance_ = 0;
            maxM2Distance_ = 0;
			return;
		} catch (RoboClawDriverException* e) {
			ROS_ERROR("[RoboClawDriver::stop] Exception: %s, retry number: %d", e->what(), retry);
			restartRoboClaw();
		} catch (...) {
		    ROS_ERROR("[RoboClawDriver::stop] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [RoboClawDriver::stop] RETRY COUNT EXCEEDED");
	throw new RoboClawDriverException("[RoboClawDriver::stop] RETRY COUNT EXCEEDED");
}


void RoboClawDriver::updateCrc(uint16_t& crc, uint8_t data) {
	crc = crc ^ ((uint16_t) data << 8);
	for (int i = 0; i < 8; i++)	{
		if (crc & 0x8000)
			crc = (crc << 1) ^ 0x1021;
		else
			crc <<= 1;
	}
}


void RoboClawDriver::vwToWheelVelocity(float v, float w, float *leftMotorVelocity, float *rightMotorVelocity) {
    // Compute the differential drive velocitiess from the input
    *leftMotorVelocity = v - (axleWidth_ / 2.0) * w;
    *rightMotorVelocity = v + (axleWidth_ / 2.0) * w;

    // Scale the velocities to respect the wheel velocity limit
    float limitk = 1.0;
    if (fabs(*leftMotorVelocity) > maximumWheelVelocity_) {
      limitk = maximumWheelVelocity_ / fabs(*leftMotorVelocity);
    }

    if (fabs(*rightMotorVelocity) > maximumWheelVelocity_) {
      float rlimitk = maximumWheelVelocity_ / fabs(*rightMotorVelocity);
      if (rlimitk < limitk) {
        limitk = rlimitk;
      }
    }

    if (limitk != 1.0) {
      *leftMotorVelocity *= limitk;
      *rightMotorVelocity *= limitk;
    }

    // Deal with min limits
    if (fabs(*leftMotorVelocity) < minimumWheelVelocity_) {
      *leftMotorVelocity = leftMotorVelocity > 0 ? minimumWheelVelocity_ : -minimumWheelVelocity_;
    } if (fabs(*rightMotorVelocity) < minimumWheelVelocity_) {
      *rightMotorVelocity = rightMotorVelocity > 0 ? minimumWheelVelocity_ : -minimumWheelVelocity_;
    }
}


void RoboClawDriver::writeByte(uint8_t byte) {
	ssize_t result = write(clawPort_, &byte, 1);
	if (result != 1) {
	  	ROS_ERROR("[RoboClawDriver::writeByte] Unable to write one byte, result: %d, errno: %d)",
	  			  (int) result,
	  			  errno);
		restartRoboClaw();
		throw new RoboClawDriverException("[RoboClawDriver::writeByte] Unable to write one byte");
	}
}


void RoboClawDriver::writeN(bool sendCRC, uint8_t cnt, ...) {
	uint16_t crc = 0;
	va_list marker;
	va_start(marker, cnt);

	int origFlags = fcntl(clawPort_, F_GETFL, 0);	// Capture original flags.
	fcntl(clawPort_, F_SETFL, origFlags & ~O_NONBLOCK);

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
			ROS_ERROR("[RoboClawDriver::writeN] Invalid ACK response");
			throw new RoboClawDriverException("[RoboClawDriver::writeN] Invalid ACK response");
		}
	}

	fcntl(clawPort_, F_SETFL, origFlags);	// Restore original flags.
}


const float RoboClawDriver::kMAX_SECONDS_TRAVEL = 0.5;
boost::mutex RoboClawDriver::roboClawLock_;