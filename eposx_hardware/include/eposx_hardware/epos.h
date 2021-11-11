#ifndef EPOSX_HARDWARE_EPOS_H_
#define EPOSX_HARDWARE_EPOS_H_

#include <list>
#include <map>
#include <string>
#include <vector>

#include <eposx_hardware/epos_diagnostic_updater.h>
#include <eposx_hardware/epos_operation_mode.h>
#include <eposx_hardware/utils.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <sensor_msgs/BatteryState.h>

#include <boost/cstdint.hpp>
#include <boost/shared_ptr.hpp>

namespace eposx_hardware {

class Epos {
public:
  Epos();
  virtual ~Epos();

  void init(hardware_interface::RobotHW &hw, ros::NodeHandle &root_nh, ros::NodeHandle &motor_nh,
            const std::string &motor_name);
  void doSwitch(const std::list< hardware_interface::ControllerInfo > &start_list,
                const std::list< hardware_interface::ControllerInfo > &stop_list);
  void read();
  void write();

private:
  // subfunctions for init()
  void initHardwareInterface(hardware_interface::RobotHW &hw, ros::NodeHandle &motor_nh);
  void initEposNodeHandle(ros::NodeHandle &motor_nh);
  void initProtocolStackSettings(ros::NodeHandle &motor_nh);
  void initOperationMode(hardware_interface::RobotHW &hw, ros::NodeHandle &root_nh,
                         ros::NodeHandle &motor_nh);
  void initFaultReaction(ros::NodeHandle &motor_nh);
  void initMotorParameter(ros::NodeHandle &motor_nh);
  void initSensorParameter(ros::NodeHandle &motor_nh);
  void initSafetyParameter(ros::NodeHandle &motor_nh);
  void initPositionRegulator(ros::NodeHandle &motor_nh);
  void initVelocityRegulator(ros::NodeHandle &motor_nh);
  void initCurrentRegulator(ros::NodeHandle &motor_nh);
  void initPositionProfile(ros::NodeHandle &motor_nh);
  void initVelocityProfile(ros::NodeHandle &motor_nh);
  void initDeviceError(ros::NodeHandle &motor_nh);
  void initMiscParameters(ros::NodeHandle &motor_nh);

  // subfunctions for read()
  void readJointState();
  void readPowerSupply();
  void readDiagnostic();

private:
  typedef boost::shared_ptr< EposOperationMode > OperationModePtr;
  typedef std::map< std::string, OperationModePtr > OperationModeMap;
  typedef boost::shared_ptr< EposDiagnosticData > DiagnosticDataPtr;

  std::string motor_name_;

  eposx_hardware::NodeHandle epos_handle_;
  OperationModeMap operation_mode_map_;
  OperationModePtr operation_mode_;

  // state: epos -> ros
  double position_;
  double velocity_;
  double effort_;
  double current_;
  sensor_msgs::BatteryStatePtr power_supply_state_;
  DiagnosticDataPtr diagnostic_data_;

  bool rw_ros_units_;
  double torque_constant_;
  int encoder_resolution_;
};
} // namespace eposx_hardware

#endif
