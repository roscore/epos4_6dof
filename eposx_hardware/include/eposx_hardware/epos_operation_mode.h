#ifndef EPOSX_HARDWARE_EPOS_OPERATION_MODE_H
#define EPOSX_HARDWARE_EPOS_OPERATION_MODE_H

#include <string>
#include <vector>

#include <dynamic_joint_limits_interface/joint_limits_interface.h>
#include <eposx_hardware/utils.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>

namespace eposx_hardware {

class EposOperationMode {
public:
  virtual ~EposOperationMode();

  // configure operation mode (e.g. register command handle or load parameters)
  virtual void init(hardware_interface::RobotHW &hw, ros::NodeHandle &root_nh,
                    ros::NodeHandle &motor_nh, const std::string &motor_name,
                    eposx_hardware::NodeHandle &epos_handle) = 0;

  // activate operation mode
  virtual void activate() = 0;

  // read something required for operation mode
  virtual void read() = 0;

  // write commands of operation mode
  virtual void write() = 0;
};

class EposProfilePositionMode : public EposOperationMode {
public:
  virtual ~EposProfilePositionMode();

  virtual void init(hardware_interface::RobotHW &hw, ros::NodeHandle &root_nh,
                    ros::NodeHandle &motor_nh, const std::string &motor_name,
                    eposx_hardware::NodeHandle &epos_handle);
  virtual void activate();
  virtual void read();
  virtual void write();

private:
  std::vector< std::string > joint_names_;
  dynamic_joint_limits_interface::PositionJointSaturationInterface *pos_sat_iface_;
  eposx_hardware::NodeHandle epos_handle_;
  bool rw_ros_units_;
  int encoder_resolution_;
  double position_cmd_;
};

class EposProfileVelocityMode : public EposOperationMode {
public:
  virtual ~EposProfileVelocityMode();

  virtual void init(hardware_interface::RobotHW &hw, ros::NodeHandle &root_nh,
                    ros::NodeHandle &motor_nh, const std::string &motor_name,
                    eposx_hardware::NodeHandle &epos_handle);
  virtual void activate();
  virtual void read();
  virtual void write();

private:
  eposx_hardware::NodeHandle epos_handle_;
  bool rw_ros_units_;
  bool halt_velocity_;
  double velocity_cmd_;
};

class EposCurrentMode : public EposOperationMode {
public:
  virtual ~EposCurrentMode();

  virtual void init(hardware_interface::RobotHW &hw, ros::NodeHandle &root_nh,
                    ros::NodeHandle &motor_nh, const std::string &motor_name,
                    eposx_hardware::NodeHandle &epos_handle);
  virtual void activate();
  virtual void read();
  virtual void write();

private:
  eposx_hardware::NodeHandle epos_handle_;
  bool rw_ros_units_;
  double torque_constant_;
  double effort_cmd_;
};

class EposCyclicSynchronoustTorqueMode : public EposOperationMode {
public:
  virtual ~EposCyclicSynchronoustTorqueMode();

  virtual void init(hardware_interface::RobotHW &hw, ros::NodeHandle &root_nh,
                    ros::NodeHandle &motor_nh, const std::string &motor_name,
                    eposx_hardware::NodeHandle &epos_handle);
  virtual void activate();
  virtual void read();
  virtual void write();

private:
  eposx_hardware::NodeHandle epos_handle_;
  bool rw_ros_units_;
  double motor_rated_torque_;
  double effort_cmd_;
};

} // namespace eposx_hardware

#endif