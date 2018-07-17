#include "ros/ros.h"

#include "motor_controller/WRDifferentialDrive.h"
#include <regex>

WRDifferentialDrive::WRDifferentialDrive(ros::NodeHandle &nh, urdf::Model *urdf_model) 
  : nh_(nh)
  , useRosparamJointLimits_(false)
  , useSoftLimitsIfAvailable_(false) {
  if (urdf_model == NULL) {
    loadURDF(nh, "robot_description");
  } else {
    model_ = urdf_model;
  }
  
  ROS_INFO_STREAM("[WRDifferentialDrive::WRDifferentialDrive] constructor");

  // std::vector< boost::shared_ptr< urdf::Link > > links;
  // model_->getLinks(links);
  // std::vector< boost::shared_ptr< urdf::Link > >::iterator it;
  // ROS_INFO_STREAM("[WRDifferentialDrive::WRDifferentialDrive] found " << links.size() << " links");
  // for (it = links.begin(); it != links.end(); it++) {
  // 	int foo = 3;
  // 	//ROS_INFO_STREAM("[WRDifferentialDrive::WRDifferentialDrive] link name: " << *it.name);
  // }
  //
  // 	for (auto link: links) {
  // 		auto l = *link;
  // 		ROS_INFO_STREAM("[WRDifferentialDrive::WRDifferentialDrive] link name: " << l.name);
  // 		for (auto joint: l.child_joints) {
  // 			auto j = *joint;
  // 			ROS_INFO_STREAM("[WRDifferentialDrive::WRDifferentialDrive] ... joint name: " << j.name);
  // 		}
  // 	}
  
  std::string jointNamesParameter = "/" + model_->getName() + "/joint_names";
  if (!nh.getParam(jointNamesParameter, jointNames_)) {
    ROS_ERROR_STREAM("[WRDifferentialDrive::WRDifferentialDrive] Missing " << jointNamesParameter << " parameter");
  }
  
  for (auto jointName: jointNames_) {
    ROS_INFO_STREAM("[WRDifferentialDrive::WRDifferentialDrive] joint name: " << jointName);
  }
}

WRDifferentialDrive::~WRDifferentialDrive() {
}


void WRDifferentialDrive::init() {
  ROS_INFO("[WRDifferentialDrive::init]");
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
    ROS_DEBUG_STREAM("[WRDifferentialDrive::init] Loading joint name: " << jointNames_[jointId]);
    
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
  ROS_DEBUG_STREAM("[WRDifferentialDrive::init] Initialized");
}


void WRDifferentialDrive::loadURDF(ros::NodeHandle &nh, std::string param_name) {
  std::string urdf_string;
  model_ = new urdf::Model();
  
  // search and wait for robot_description on param server
  while (urdf_string.empty()/*### && ros::ok()###*/) {
    std::string search_param_name;
    if (nh.searchParam(param_name, search_param_name)) {
      ROS_INFO_STREAM("[WRDifferentialDrive::loadURDF] In namespace: "
		      << nh.getNamespace()
		      << ", waiting for model URDF"
		      " on the ROS param server at search_param_name: "
		      << search_param_name);
      nh.getParam(search_param_name, urdf_string);
      break;
    } else {
      ROS_INFO_STREAM("[WRDifferentialDrive::loadURDF] In namespace: "
		      << nh.getNamespace()
		      << ", waiting for model URDF"
		      " on the ROS param server at param_name: "
		      << param_name);
      nh.getParam(param_name, urdf_string);
      break;
    }
    
    usleep(10000);
  }
  
  ROS_INFO_STREAM("[WRDifferentialDrive::loadURDF] fetched: " << urdf_string.length() << " bytes");
  ROS_INFO_STREAM("[WRDifferentialDrive::loadURDF] about to initialize model with string");
  if (!model_->initString(urdf_string)) {
    ROS_ERROR_STREAM("[WRDifferentialDrive::loadURDF] Unable to initialize model with loaded URDF model");
  } else {
    ROS_INFO_STREAM("[WRDifferentialDrive::loadURDF] URDF model successfully initialized");
  }
}

void WRDifferentialDrive::registerJointLimits(const hardware_interface::JointHandle &jointHandlePosition,
                                              const hardware_interface::JointHandle &jointHandleVelocity,
                                              const hardware_interface::JointHandle &jointHandleEffort,
                                              std::size_t jointId) {
  ROS_INFO_STREAM("[WRDifferentialDrive::registerJointLimits] jointId: " 
		  << jointId
		  << ": name: "
		  << jointNames_[jointId]);
  // Default values
  jointPositionLowerLimits_[jointId] = -std::numeric_limits<double>::max();
  jointPositionUpperLimits_[jointId] = std::numeric_limits<double>::max();
  jointVelocityLimits_[jointId] = std::numeric_limits<double>::max();
  jointEffortLimits_[jointId] = std::numeric_limits<double>::max();
  
  // Limits datastructures
  joint_limits_interface::JointLimits jointLimits;     // Position
  joint_limits_interface::SoftJointLimits softLimits;  // Soft Position
  bool hasJointLimits = false;
  bool hasSoftLimits = false;
  
  // Get limits from URDF
  if (model_ == NULL) {
    ROS_ERROR_STREAM("[WRDifferentialDrive::registerJointLimits] No URDF model loaded, unable to get joint limits");
    return;
  }
  
  // Get limits from URDF
  urdf::JointConstSharedPtr urdfJoint = model_->getJoint(jointNames_[jointId]);
  
  // Get main joint limits
  if (urdfJoint == NULL)	{
    ROS_ERROR_STREAM("[WRDifferentialDrive::registerJointLimits] URDF joint not found " << jointNames_[jointId]);
    return;
  }
  
  // Get limits from URDF
  if (joint_limits_interface::getJointLimits(urdfJoint, jointLimits)) {
    hasJointLimits = true;
    if (jointLimits.has_position_limits) {
      ROS_INFO_STREAM("[WRDifferentialDrive::registerJointLimits] Joint " 
		      << jointNames_[jointId] 
		      << " has URDF position limits ["
		      << jointLimits.min_position 
		      << ", "
		      << jointLimits.max_position 
		      << "]");
    }
    
    if (jointLimits.has_velocity_limits) {
      ROS_INFO_STREAM("[WRDifferentialDrive::registerJointLimits] Joint " 
		      << jointNames_[jointId]
		      << " has URDF velocity limit ["
		      << jointLimits.max_velocity 
		      << "]");
    }
  } else {
    if (urdfJoint->type != urdf::Joint::CONTINUOUS) {
      ROS_WARN_STREAM("[WRDifferentialDrive::registerJointLimits] Joint "
		      << jointNames_[jointId] 
		      << " does not have a URDF "
		      "position limit");
    } else {
      ROS_INFO_STREAM("[WRDifferentialDrive::registerJointLimits] Joint "
		      << jointNames_[jointId]
		      << " is a CONTINUOUS joint type");
    }
  }
  
  // Get limits from ROS param
  if (useRosparamJointLimits_) {
    if (joint_limits_interface::getJointLimits(jointNames_[jointId], nh_, jointLimits)) {
      hasJointLimits = true;
      ROS_INFO_STREAM("[WRDifferentialDrive::registerJointLimits] Joint " 
		      << jointNames_[jointId]
		      << " has rosparam position limits ["
		      << jointLimits.min_position
		      << ", "
		      << jointLimits.max_position
		      << "]");
      if (jointLimits.has_velocity_limits)
	ROS_INFO_STREAM("[WRDifferentialDrive::registerJointLimits] Joint " 
			<< jointNames_[jointId]
			<< " has rosparam velocity limit ["
			<< jointLimits.max_velocity 
			<< "]");
    }
  } else {
    ROS_INFO_STREAM("[WRDifferentialDrive::registerJointLimits] useRosparamJointLimits_ is FALSE");
  }
  
  // Get soft limits from URDF
  if (useSoftLimitsIfAvailable_) {
    if (joint_limits_interface::getSoftJointLimits(urdfJoint, softLimits)) {
      hasSoftLimits = true;
      ROS_INFO_STREAM("[WRDifferentialDrive::registerJointLimits] Joint " 
		      << jointNames_[jointId] 
		      << " has soft joint limits.");
    } else {
      ROS_INFO_STREAM("[WRDifferentialDrive::registerJointLimits] Joint " 
		      << jointNames_[jointId] 
		      << " does not have soft joint "
		      "limits");
    }
  } else {
    ROS_INFO_STREAM("[WRDifferentialDrive::registerJointLimits] useSoftLimitsIfAvailable_ is FALSE");
  }
  
  // Quit we we haven't found any limits in URDF or rosparam server
  if (!hasJointLimits) {
    ROS_INFO_STREAM("[WRDifferentialDrive::registerJointLimits] Has no joint limits");
    return;
  }
  
  // Copy position limits if available
  if (jointLimits.has_position_limits) {
    // Slighly reduce the joint limits to prevent floating point errors
    jointLimits.min_position += std::numeric_limits<double>::epsilon();
    jointLimits.max_position -= std::numeric_limits<double>::epsilon();
    
    jointPositionLowerLimits_[jointId] = jointLimits.min_position;
    jointPositionUpperLimits_[jointId] = jointLimits.max_position;
  }
  
  // Copy velocity limits if available
  if (jointLimits.has_velocity_limits) {
    jointVelocityLimits_[jointId] = jointLimits.max_velocity;
  }
  
  // Copy effort limits if available
  if (jointLimits.has_effort_limits) {
    jointEffortLimits_[jointId] = jointLimits.max_effort;
  }
  
  if (hasSoftLimits) {
    // Use soft limits
    ROS_INFO_STREAM("[WRDifferentialDrive::registerJointLimits] Using soft saturation limits");
    const joint_limits_interface::PositionJointSoftLimitsHandle softHandlePosition(jointHandlePosition, jointLimits, softLimits);
    positionJointSoftLimits_.registerHandle(softHandlePosition);
    const joint_limits_interface::VelocityJointSoftLimitsHandle softHandleVelocity(jointHandleVelocity, jointLimits, softLimits);
    velocityJointSoftLimits_.registerHandle(softHandleVelocity);
    const joint_limits_interface::EffortJointSoftLimitsHandle softHandleEffort(jointHandleEffort, jointLimits, softLimits);
    effortJointSoftLimits_.registerHandle(softHandleEffort);
  } else {
    // Use saturation limits
    ROS_INFO_STREAM("[WRDifferentialDrive::registerJointLimits] Using saturation limits (not soft limits)");
    const joint_limits_interface::PositionJointSaturationHandle satHandlePosition(jointHandlePosition, jointLimits);
    positionJointSaturationInterface_.registerHandle(satHandlePosition);
    
    const joint_limits_interface::VelocityJointSaturationHandle satHandleVelocity(jointHandleVelocity, jointLimits);
    velocityJointSaturationInterface_.registerHandle(satHandleVelocity);
    
    const joint_limits_interface::EffortJointSaturationHandle satHandleEffort(jointHandleEffort, jointLimits);
    effortJointSaturationInterface_.registerHandle(satHandleEffort);
  }
}

void WRDifferentialDrive::reset() {
  // Reset joint limits state, in case of mode switch or e-stop
  ROS_INFO_STREAM("[WRDifferentialDrive::reset]");
  positionJointSaturationInterface_.reset();
  positionJointSoftLimits_.reset();
}


