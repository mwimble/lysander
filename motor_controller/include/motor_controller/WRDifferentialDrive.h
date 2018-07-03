#ifndef __WRDIFFERENTIAL_DRIVE
#define __WRDIFFERENTIAL_DRIVE

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <urdf/model.h>

class WRDifferentialDrive : public hardware_interface::RobotHW {
public:
  WRDifferentialDrive(ros::NodeHandle &nh, urdf::Model *urdf_model = NULL) ;
  ~WRDifferentialDrive();

  /**
    * Initialize the robot hardware interface.
    */
  virtual void init();

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];

  urdf::Model* model_;

  void loadURDF(ros::NodeHandle &nh, std::string param_name);
};

#endif