#include "motor_controller/LysanderMotor.h"
#include <boost/assign.hpp>
#include <fcntl.h>
#include <iostream>
#include <math.h>
#include <poll.h>
#include <sstream>
#include <stdio.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

//TODO 
// Further initialization?
// No command in n-sec should stop motor.

#define SetDWORDval(arg) (uint8_t)(arg>>24),(uint8_t)(arg>>16),(uint8_t)(arg>>8),(uint8_t)arg

LysanderMotor::LysanderMotor(ros::NodeHandle &nh, urdf::Model *urdf_model)
	: WRDifferentialDrive(nh, urdf_model)
	, controlLoopMaxAllowedDurationDeviation_(1.0)
	, nh_(nh)
	, urdf_model_(urdf_model) {

	assert(ros::param::get("motor_controller/control_loop_hz", controlLoopHz_));
	assert(ros::param::get("motor_controller/max_command_retries", maxCommandRetries_));
	assert(ros::param::get("motor_controller/max_seconds_uncommanded_travel", maxSecondsUncommandedTravel_));
	assert(ros::param::get("motor_controller/port_address", portAddress_));
	assert(ros::param::get("motor_controller/quad_pulses_per_meter", quadPulsesPerMeter_));
	assert(ros::param::get("motor_controller/quad_pulses_per_revolution", quadPulsesPerRevolution_));
	assert(ros::param::get("motor_controller/usb_device_name", motorUSBPort_));
	assert(ros::param::get("motor_controller/wheel_radius", wheelRadius_));
	ROS_INFO("[LysanderMotor::LysanderMotor] motor_controller/control_loop_hz: %6.3f", controlLoopHz_);
	ROS_INFO("[LysanderMotor::LysanderMotor] motor_controller/max_command_retries: %d", maxCommandRetries_);
	ROS_INFO("[LysanderMotor::LysanderMotor] motor_controller/max_seconds_uncommanded_travel: %6.3f", maxSecondsUncommandedTravel_);
	ROS_INFO_STREAM("[LysanderMotor::LysanderMotor] motor_controller/port_address: 0x" << std::hex << portAddress_);
	ROS_INFO("[LysanderMotor::LysanderMotor] motor_controller/quad_pulses_per_meter: %8.3f", quadPulsesPerMeter_);
	ROS_INFO("[LysanderMotor::LysanderMotor] motor_controller/quad_pulses_per_revolution: %8.3f", quadPulsesPerRevolution_);
	ROS_INFO("[LysanderMotor::LysanderMotor] motor_controller/usb_device_name: %s", motorUSBPort_.c_str());

	now_ = ros::Time::now();
	lastTime_ = now_;

    jointNames_.push_back("front_left_wheel");
    jointNames_.push_back("front_right_wheel");

	// Status
	jointPosition_.resize(jointNames_.size(), 0.0);
	jointVelocity_.resize(jointNames_.size(), 0.0);
	jointEffort_.resize(jointNames_.size(), 0.0);

	// Command
	jointPositionCommand_.resize(jointNames_.size(), 0.0);
	jointVelocityCommand_.resize(jointNames_.size(), 0.0);
	jointEffortCommand_.resize(jointNames_.size(), 0.0);

	// Limits
	jointPositionLowerLimits_.resize(jointNames_.size(), 0.0);
	jointPositionUpperLimits_.resize(jointNames_.size(), 0.0);
	jointVelocityLimits_.resize(jointNames_.size(), 0.0);
	jointEffortLimits_.resize(jointNames_.size(), 0.0);

	// Initialize interfaces for each joint
	for (std::size_t jointId = 0; jointId < jointNames_.size(); ++jointId) {
		ROS_INFO("[LysanderMotor::LysanderMotor] Loading joint name: %s", jointNames_[jointId].c_str());

		// Create joint state interface
		jointStateInterface_.registerHandle
		  (hardware_interface::JointStateHandle(jointNames_[jointId],
						    &jointPosition_[jointId],
						    &jointVelocity_[jointId],
						    &jointEffort_[jointId]));

		// Add command interfaces to joints
		// TODO: decide based on transmissions?
		hardware_interface::JointHandle jointHandlePosition =
		  hardware_interface::JointHandle(jointStateInterface_.getHandle(jointNames_[jointId]),
					      &jointPositionCommand_[jointId]);
		positionJointInterface_.registerHandle(jointHandlePosition);

		hardware_interface::JointHandle jointHandleVelocity =
		  hardware_interface::JointHandle(jointStateInterface_.getHandle(jointNames_[jointId]),
					      &jointVelocityCommand_[jointId]);
		velocityJointInterface_.registerHandle(jointHandleVelocity);

		hardware_interface::JointHandle jointHandleEffort =
		  hardware_interface::JointHandle(jointStateInterface_.getHandle(jointNames_[jointId]),
					      &jointEffortCommand_[jointId]);
		effortJointInterface_.registerHandle(jointHandleEffort);

		// Load the joint limits
		registerJointLimits(jointHandlePosition, jointHandleVelocity, jointHandleEffort, jointId);
	}

	registerInterface(&jointStateInterface_);     // From RobotHW base class.
	registerInterface(&positionJointInterface_);  // From RobotHW base class.
	registerInterface(&velocityJointInterface_);  // From RobotHW base class.
	registerInterface(&effortJointInterface_);    // From RobotHW base class.

	controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
	expectedControlLoopDuration_ = ros::Duration(1 / controlLoopHz_);

	openPort();

	float M1_P =  8762.98571;
	float M2_P = 9542.41265;
	float M1_I = 1535.49646;
	float M2_I = 1773.65086;
	float M1_QPPS = 3562;
	float M2_QPPS = 3340;

	setM1PID(M1_P, M1_I, 0, M1_QPPS);
	setM2PID(M2_P, M2_I, 0, M2_QPPS);

	ROS_INFO("[LysanderMotor::LysanderMotor] Initialized");
	ROS_INFO("[LysanderMotor::LysanderMotor] RoboClaw software version: %s", getVersion().c_str());
}


LysanderMotor::~LysanderMotor() {

}


void LysanderMotor::controlLoop() {
	ros::Rate rate(controlLoopHz_);
	while(ros::ok()) {
		update();
		rate.sleep();
	}
}


LysanderMotor::EncodeResult LysanderMotor::getEncoderCommandResult(uint8_t command) {
	uint16_t crc = 0;
	updateCrc(crc, portAddress_);
	updateCrc(crc, command);

	writeN(false, 2, portAddress_, command);
	EncodeResult result = {0, 0};
	uint8_t datum = readByteWithTimeout();

	if (datum == -1) {
		ROS_ERROR("[LysanderMotor::getEncoderCommandResult] Received -1 instead of expected data");
		throw new TRoboClawException("[LysanderMotor::::getEncoderCommandResult] RECEIVED -1");
	}

	result.value |= datum << 24;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	if (datum == -1) {
		ROS_ERROR("[LysanderMotor::getEncoderCommandResult] Received -1 instead of expected data");
		throw new TRoboClawException("[LysanderMotor::::getEncoderCommandResult] RECEIVED -1");
	}

	result.value |= datum << 16;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	if (datum == -1) {
		ROS_ERROR("[LysanderMotor::getEncoderCommandResult] Received -1 instead of expected data");
		throw new TRoboClawException("[LysanderMotor::::getEncoderCommandResult] RECEIVED -1");
	}

	result.value |= datum << 8;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	if (datum == -1) {
		ROS_ERROR("[LysanderMotor::getEncoderCommandResult] Received -1 instead of expected data");
		throw new TRoboClawException("[LysanderMotor::::getEncoderCommandResult] RECEIVED -1");
	}

	result.value |= datum;
	updateCrc(crc, datum);
	datum = readByteWithTimeout();
	if (datum == -1) {
		ROS_ERROR("[LysanderMotor::getEncoderCommandResult] Received -1 instead of expected data");
		throw new TRoboClawException("[LysanderMotor::::getEncoderCommandResult] RECEIVED -1");
	}

	result.status |= datum;
	updateCrc(crc, datum);
	if (datum == -1) {
		ROS_ERROR("[LysanderMotor::getEncoderCommandResult] Received -1 instead of expected data");
		throw new TRoboClawException("[LysanderMotor::::getEncoderCommandResult] RECEIVED -1");
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

	ROS_ERROR("[LysanderMotor::getEncoderCommandResult] Expected CRC of: 0x%X, but got: 0x%X"
	    	  , int(crc)
	    	  , int(responseCrc));
	throw new TRoboClawException("[LysanderMotor::getEncoderCommandResult] INVALID CRC");
}


uint16_t LysanderMotor::getErrorStatus() {
	boost::mutex::scoped_lock lock(roboClawLock_);
	int retry;

	for (retry = 0; retry < maxCommandRetries_; retry++) {
		try {
			uint16_t crc = 0;
			updateCrc(crc, portAddress_);
			updateCrc(crc, kGETERROR);
			writeN(false, 2, portAddress_, kGETERROR);
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
		} catch (TRoboClawException* e) {
			ROS_ERROR("[LysanderMotor::getErrorStatus] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[LysanderMotor::getErrorStatus] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [LysanderMotor::getErrorStatus] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[LysanderMotor::getErrorStatus] RETRY COUNT EXCEEDED");
}


std::string LysanderMotor::getErrorString() {
	uint8_t errorStatus = getErrorStatus();
	if (errorStatus == 0) return "normal";
	else {
		std::stringstream errorMessage;
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

		return errorMessage.str();
	}
}


int32_t LysanderMotor::getM1Encoder() {
	boost::mutex::scoped_lock lock(roboClawLock_);
	int retry;

	for (retry = 0; retry < maxCommandRetries_; retry++) {
		try {
			EncodeResult result = getEncoderCommandResult(kGETM1ENC);
			return result.value;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[LysanderMotor::getM1Encoder] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[LysanderMotor::getM1Encoder] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [LysanderMotor::getM1Encoder] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[LysanderMotor::getM1Encoder] RETRY COUNT EXCEEDED");
}


int32_t LysanderMotor::getM2Encoder() {
	boost::mutex::scoped_lock lock(roboClawLock_);
	int retry;

	for (retry = 0; retry < maxCommandRetries_; retry++) {
		try {
			EncodeResult result = getEncoderCommandResult(kGETM2ENC);
			return result.value;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[LysanderMotor::getM2Encoder] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[LysanderMotor::getM2Encoder] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [LysanderMotor::getM2Encoder] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[LysanderMotor::getM2Encoder] RETRY COUNT EXCEEDED");
}


std::string LysanderMotor::getVersion() {
	boost::mutex::scoped_lock lock(roboClawLock_);
	int retry;

	for (retry = 0; retry < maxCommandRetries_; retry++) {
		try {
			uint16_t crc = 0;
			updateCrc(crc, portAddress_);
			updateCrc(crc, kGETVERSION);
			writeN(false, 2, portAddress_, kGETVERSION);

			uint8_t i;
			uint8_t datum;
			std::stringstream version;

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

			ROS_ERROR("[LysanderMotor::getVersion] unexpected long string");
			throw new TRoboClawException("[LysanderMotor::getVersion] unexpected long string");
		} catch (TRoboClawException* e) {
			ROS_ERROR("[LysanderMotor::getVersion] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[LysanderMotor::getVersion] Uncaught exception !!!");
		}
	}


	ROS_ERROR("[LysanderMotor::getVersion] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[LysanderMotor::getVersion] RETRY COUNT EXCEEDED");
}


void LysanderMotor::openPort() {
	ROS_INFO("[LysanderMotor::openPort] about to open port: %s", motorUSBPort_.c_str());
	clawPort_ = open(motorUSBPort_.c_str(), O_RDWR | O_NOCTTY);
	if (clawPort_ < 0) {
		ROS_ERROR("[LysanderMotor::openPort] Unable to open USB port: %s, errno: (%d) %s"
				  , motorUSBPort_.c_str()
				  , errno
				  , strerror(errno));
		throw new TRoboClawException("[LysanderMotor::openPort] Unable to open USB port");
	}


    // Fetch the current port settings.
	struct termios portOptions;
	int ret = 0;

 	ret = tcgetattr(clawPort_, &portOptions);
	if (ret < 0) {
		ROS_ERROR("[LysanderMotor::openPort] Unable to get terminal options (tcgetattr), error: %d: %s", errno, strerror(errno));
		// throw new TRoboClawException("[LysanderMotor::openPort] Unable to get terminal options (tcgetattr)");
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
		ROS_ERROR("[LysanderMotor::openPort] Unable to set terminal speed (cfsetispeed)");
		throw new TRoboClawException("[LysanderMotor::openPort] Unable to set terminal speed (cfsetispeed)");
    }

    if (cfsetospeed(&portOptions, B38400) < 0) {
		ROS_ERROR("[LysanderMotor::openPort] Unable to set terminal speed (cfsetospeed)");
		throw new TRoboClawException("[LysanderMotor::openPort] Unable to set terminal speed (cfsetospeed)");
    }

    // Now that we've populated our options structure, let's push it back to the system.
    if (tcsetattr(clawPort_, TCSANOW, &portOptions) < 0) {
		ROS_ERROR("[LysanderMotor::openPort] Unable to set terminal options (tcsetattr)");
		throw new TRoboClawException("[LysanderMotor::openPort] Unable to set terminal options (tcsetattr)");
    }
}


void LysanderMotor::read(const ros::Time& time, const ros::Duration& period) {
	int32_t m1Encoder = getM1Encoder();
	int32_t m2Encoder = getM2Encoder();
	// double m1Distance = m1Encoder / quadPulsesPerMeter_;
	// double m2Distance = m2Encoder / quadPulsesPerMeter_;
	
	// jointPosition_[0] = m1Distance;
	// jointPosition_[1] = m2Distance;
	double m1Radians = (m1Encoder / quadPulsesPerRevolution_) * 2.0 * M_PI;
	double m2Radians = (m2Encoder / quadPulsesPerRevolution_) * 2.0 * M_PI;
	
	jointPosition_[0] = m1Radians;
	jointPosition_[1] = m2Radians;

	for (int i = 0; i < jointVelocityCommand_.size(); i++) {
		/*
		ROS_INFO(
		     "LysanderMotor::read joint: %d (%s), jointVelocityCommand_: %6.3f"
		     ", jointPositionCommand_: %6.3f"
		     ", jointEffortCommand_: %6.3f"
		     ", jointPosition_: %6.3f"
		     , i
		     , jointNames_[i].c_str()
		     , jointVelocityCommand_[i]
		     , jointPositionCommand_[i]
		     , jointEffortCommand_[i]
		     , jointPosition_[i]);
		     */
	}
}


uint8_t LysanderMotor::readByteWithTimeout() {
	struct pollfd ufd[1];
	ufd[0].fd = clawPort_;
	ufd[0].events = POLLIN;

	int retval = poll(ufd, 1, 11);
	if (retval < 0) {
		ROS_ERROR("[LysanderMotor::readByteWithTimeout] Poll failed (%d) %s", errno, strerror(errno));
		throw new TRoboClawException("[LysanderMotor::readByteWithTimeout] Read error");
	} else if (retval == 0) {
		std::stringstream ev;
		ev << "[LysanderMotor::readByteWithTimeout] TIMEOUT revents: " << std::hex << ufd[0].revents;
		ROS_ERROR_STREAM(ev.str());
		throw new TRoboClawException("[LysanderMotor::readByteWithTimeout] TIMEOUT");
	} else if (ufd[0].revents & POLLERR) {
		ROS_ERROR("[LysanderMotor::readByteWithTimeout] Error on socket");
		restartPort();
		throw new TRoboClawException("[LysanderMotor::readByteWithTimeout] Error on socket");
	} else if (ufd[0].revents & POLLIN) {
		unsigned char buffer[1];
		ssize_t bytesRead = ::read(clawPort_, buffer, sizeof(buffer));
		if (bytesRead != 1) {
			ROS_ERROR("[LysanderMotor::readByteWithTimeout] Failed to read 1 byte, read: %d", (int) bytesRead);
			throw TRoboClawException("[LysanderMotor::readByteWithTimeout] Failed to read 1 byte");
		}

		return buffer[0];
	} else {
		ROS_ERROR("[LysanderMotor::readByteWithTimeout] Unhandled case");
		throw new TRoboClawException("[LysanderMotor::readByteWithTimeout] Unhandled case");
	}
}


void LysanderMotor::restartPort() {
    close(clawPort_);
    usleep(200000);
    openPort();
}


void LysanderMotor::setM1PID(float p, float i, float d, uint32_t qpps) {
	boost::mutex::scoped_lock lock(roboClawLock_);
	int retry;

	for (retry = 0; retry < maxCommandRetries_; retry++) {
		try {
			uint32_t kp = int(p * 65536.0); // 14834322.6368 = E25A93
			uint32_t ki = int(i * 65536.0);
			uint32_t kd = int(d * 65536.0);
			writeN(true, 18, portAddress_, kSETM1PID, 
				   SetDWORDval(kd),
				   SetDWORDval(kp),
				   SetDWORDval(ki),
				   SetDWORDval(qpps));
			return;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[LysanderMotor::setM1PID] Exception: %s, retry number: %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[LysanderMotor::setM1PID] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [LysanderMotor::setM1PID] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[LysanderMotor::setM1PID] RETRY COUNT EXCEEDED");
}


void LysanderMotor::setM2PID(float p, float i, float d, uint32_t qpps) {
	boost::mutex::scoped_lock lock(roboClawLock_);
	int retry;

	for (retry = 0; retry < maxCommandRetries_; retry++) {
		try {
			uint32_t kp = int(p * 65536.0); // 14834322.6368 = E25A93
			uint32_t ki = int(i * 65536.0);
			uint32_t kd = int(d * 65536.0);
			writeN(true, 18, portAddress_, kSETM2PID, 
				   SetDWORDval(kd),
				   SetDWORDval(kp),
				   SetDWORDval(ki),
				   SetDWORDval(qpps));
        	//ROS_INFO_COND(DEBUG, "<----- [LysanderMotor::setM2PID]");
			return;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[LysanderMotor::setM2PID] Exception: %s, retry number: %d",  e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[LysanderMotor::setM2PID] Uncaught exception !!!");
		}
	}

	ROS_ERROR("<----- [LysanderMotor::setM2PID] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[LysanderMotor::setM2PID] RETRY COUNT EXCEEDED");
}


void LysanderMotor::stop() {
	boost::mutex::scoped_lock lock(roboClawLock_);
	//ROS_INFO("[LysanderMotor::stop]");
	int retry;

	for (retry = 0; retry < maxCommandRetries_; retry++) {
		try {
			writeN(true
				  , 10
				  , portAddress_
				  , kMIXEDSPEED
				  , SetDWORDval(0)
				  , SetDWORDval(0));
			return;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[LysanderMotor::stop] Exception: %s, retry number: %d", e->what(), retry);
			restartPort();
		} catch (...) {
		    ROS_ERROR("[LysanderMotor::stop] Uncaught exception !!!");
		}
	}

	ROS_ERROR("[LysanderMotor::stop] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[LysanderMotor::stop] RETRY COUNT EXCEEDED");
}


void LysanderMotor::update() {
	now_ = ros::Time::now();
	elapsedTime_ = ros::Duration(now_.sec - 
      							 lastTime_.sec +
      							 (now_.nsec - lastTime_.nsec) / kBILLION);
	lastTime_ = now_;
	const double controlLoopCycleDurationDeviation = (elapsedTime_ - expectedControlLoopDuration_).toSec();

	if (controlLoopCycleDurationDeviation > controlLoopMaxAllowedDurationDeviation_) {
		ROS_WARN_STREAM("[LysanderMotor::update] Control loop was too slow by "
						<< controlLoopCycleDurationDeviation
						<< ", actual loop time: "
						<< elapsedTime_
						<< ", allowed deviation: "
						<< controlLoopMaxAllowedDurationDeviation_);
	}

	read(ros::Time(now_.sec, now_.nsec), elapsedTime_);
	controller_manager_->update(ros::Time(now_.sec, now_.nsec), elapsedTime_);
	write(ros::Time(now_.sec, now_.nsec), elapsedTime_);
}


void LysanderMotor::updateCrc(uint16_t& crc, uint8_t data) {
	crc = crc ^ ((uint16_t) data << 8);
	for (int i = 0; i < 8; i++)	{
		if (crc & 0x8000)
			crc = (crc << 1) ^ 0x1021;
		else
			crc <<= 1;
	}
}


void LysanderMotor::write(const ros::Time& time, const ros::Duration& period) {
	int retry;
	int32_t leftMaxDistance;
	int32_t rightMaxDistance;

	double leftMetersPerSecond = jointVelocityCommand_[0] * wheelRadius_; // radians/sec * meters/radian.
	double rightMetersPerSecond = jointVelocityCommand_[1] * wheelRadius_; // radians/sec * meters/radian.
	int32_t leftQuadPulsesPerSecond = leftMetersPerSecond * quadPulsesPerMeter_;
	int32_t rightQuadPulsesPerSecond = rightMetersPerSecond * quadPulsesPerMeter_;

	leftMaxDistance = fabs(leftQuadPulsesPerSecond * maxSecondsUncommandedTravel_);
	rightMaxDistance = fabs(rightQuadPulsesPerSecond * maxSecondsUncommandedTravel_);

	if ((fabs(jointVelocityCommand_[0]) > 0.01) ||
		(fabs(jointVelocityCommand_[1]) > 0.01)) {
		/*
		ROS_INFO("[LysanderMotor::write] left_command: %6.3f"
			     ", leftMetersPerSecond: %7.3f"
			     ", leftQuadPulsesPerSecond: %d"
			     ", leftMaxDistance: %d"
			     ", right_command: %6.3f"
			     ", rightMetersPerSecond: %7.3f"
			     ", rightQuadPulsesPerSecond: %d"
			     ", rightMaxDistance: %d"
			     , jointVelocityCommand_[0]
			     , leftMetersPerSecond
			     , leftQuadPulsesPerSecond
			     , leftMaxDistance
			     , jointVelocityCommand_[1]
			     , rightMetersPerSecond
			     , rightQuadPulsesPerSecond
			     , rightMaxDistance);
			     */

		for (retry = 0; retry < maxCommandRetries_; retry++) {
			try {
				writeN(true
					   , 19
					   , portAddress_
					   , kMIXEDSPEEDDIST
					   , SetDWORDval(leftQuadPulsesPerSecond)
					   , SetDWORDval(leftMaxDistance)
					   , SetDWORDval(rightQuadPulsesPerSecond)
					   , SetDWORDval(rightMaxDistance)
					   , 1 /* Cancel any previous command */
					   );
				//ROS_INFO("[LysanderMotor::write] Error status: %s", getErrorString().c_str());
				return;
			} catch (TRoboClawException* e) {
				ROS_ERROR("[LysanderMotor::write] Exception: %s, retry number %d", e->what(), retry);
			} catch (...) {
			    ROS_ERROR("[LysanderMotor::write] Uncaught exception !!!");
			}
		}
	} else {
		//ROS_INFO("[LysanderMotor::write] both commands are near zero");
		return;
	}

	ROS_ERROR("[LysanderMotor::write] RETRY COUNT EXCEEDED");
	throw new TRoboClawException("[LysanderMotor::write] RETRY COUNT EXCEEDED");
}


void LysanderMotor::writeByte(uint8_t byte) {
	ssize_t result = ::write(clawPort_, &byte, 1);
	if (result != 1) {
	  	ROS_ERROR("[LysanderMotor::writeByte] Unable to write one byte, result: %d, errno: %d)", (int) result,  errno);
		restartPort();
		throw new TRoboClawException("[LysanderMotor::writeByte] Unable to write one byte");
	}
}

void LysanderMotor::writeN(bool sendCRC, uint8_t cnt, ...) {
	uint16_t crc = 0;
	va_list marker;
	va_start(marker, cnt);

	int origFlags = fcntl(clawPort_, F_GETFL, 0);
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
			ROS_ERROR("[LysanderMotor::writeN] Invalid ACK response");
			throw new TRoboClawException("[LysanderMotor::writeN] Invalid ACK response");
		}
	}

	fcntl(clawPort_, F_SETFL, origFlags);
}


const double LysanderMotor::kBILLION = 1000000000.0;
