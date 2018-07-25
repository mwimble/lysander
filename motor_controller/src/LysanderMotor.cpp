#include "motor_controller/LysanderMotor.h"
#include <boost/assign.hpp>
#include <fcntl.h>
#include <iostream>
#include <poll.h>
#include <sstream>
#include <stdio.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

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
	assert(ros::param::get("motor_controller/usb_device_name", motorUSBPort_));
	ROS_INFO("[LysanderMotor::LysanderMotor] motor_controller/control_loop_hz: %6.3f", controlLoopHz_);
	ROS_INFO("[LysanderMotor::LysanderMotor] motor_controller/max_command_retries: %d", maxCommandRetries_);
	ROS_INFO("[LysanderMotor::LysanderMotor] motor_controller/max_seconds_uncommanded_travel: %6.3f", maxSecondsUncommandedTravel_);
	ROS_INFO_STREAM("[LysanderMotor::LysanderMotor] motor_controller/port_address: " << std::hex << portAddress_);
	ROS_INFO("[LysanderMotor::LysanderMotor] motor_controller/quad_pulses_per_meter: %8.3f", quadPulsesPerMeter_);
	ROS_INFO("[LysanderMotor::LysanderMotor] motor_controller/usb_device_name: %s", motorUSBPort_.c_str());

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
	controlLoopHz_ = 5.0; //#####
	expectedControlLoopDuration_ = ros::Duration(1 / controlLoopHz_);

	openPort();
	ROS_INFO("[LysanderMotor::LysanderMotor] Initialized");
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
	double m1Distance = m1Encoder / quadPulsesPerMeter_;
	double m2Distance = m2Encoder / quadPulsesPerMeter_;
	
	jointPosition_[0] = m1Distance;
	jointPosition_[1] = m2Distance;

	for (int i = 0; i < jointVelocityCommand_.size(); i++) {
		ROS_INFO(
		     "LysanderMotor::read joint: %d (%s), jointVelocityCommand_: %6.3f"
		     ", jointPositionCommand_: %6.3f"
		     ", jointEffortCommand_: %6.3f"
		     , i
		     , jointNames_[i].c_str()
		     , jointVelocityCommand_[i]
		     , jointPositionCommand_[i]
		     , jointEffortCommand_[i]);
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


void LysanderMotor::stop() {
	boost::mutex::scoped_lock lock(roboClawLock_);
	ROS_INFO("[LysanderMotor::stop]");
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
	clock_gettime(CLOCK_MONOTONIC, &now_);
	elapsedTime_ = ros::Duration(now_.tv_sec - 
      							 lastTime_.tv_sec +
      							 (now_.tv_nsec - lastTime_.tv_nsec) / kBILLION);
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

	read(ros::Time(now_.tv_sec, now_.tv_nsec), elapsedTime_);
	controller_manager_->update(ros::Time(now_.tv_sec, now_.tv_nsec), elapsedTime_);
	write(ros::Time(now_.tv_sec, now_.tv_nsec), elapsedTime_);
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
	int32_t m1Speed;
	int32_t m1MaxDistance;
	int32_t m2Speed;
	int32_t m2MaxDistance;

	m1Speed = static_cast<int32_t>(round(jointVelocityCommand_[0] * quadPulsesPerMeter_));
	m2Speed = static_cast<int32_t>(round(jointVelocityCommand_[1] * quadPulsesPerMeter_));
	m1MaxDistance = fabs(m1Speed * maxSecondsUncommandedTravel_);
	m2MaxDistance = fabs(m2Speed * maxSecondsUncommandedTravel_);

	for (retry = 0; retry < maxCommandRetries_; retry++) {
		try {
			writeN(true
				   , 19
				   , portAddress_
				   , kMIXEDSPEEDDIST
				   , SetDWORDval(m1Speed)
				   , SetDWORDval(m1MaxDistance)
				   , SetDWORDval(m2Speed)
				   , SetDWORDval(m2MaxDistance)
				   , 1 /* Cancel any previous command */
				   );
			return;
		} catch (TRoboClawException* e) {
			ROS_ERROR("[LysanderMotor::write] Exception: %s, retry number %d", e->what(), retry);
		} catch (...) {
		    ROS_ERROR("[LysanderMotor::write] Uncaught exception !!!");
		}
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
