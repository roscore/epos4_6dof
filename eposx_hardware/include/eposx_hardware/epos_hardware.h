#ifndef EPOSX_HARDWARE_EPOS_HARDWARE_H_
#define EPOSX_HARDWARE_EPOS_HARDWARE_H_

#include <list>
#include <string>
#include <vector>

#include <battery_state_interface/battery_state_interface.hpp>
#include <dynamic_joint_limits_interface/joint_limits_interface.h>
#include <eposx_hardware/epos_diagnostic_updater.h>
#include <eposx_hardware/epos_manager.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <ros/duration.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_interface_loader.h>

#include <boost/scoped_ptr.hpp>

namespace eposx_hardware {

class EposHardware : public hardware_interface::RobotHW {
public:
  EposHardware();
  virtual ~EposHardware();

  bool init(ros::NodeHandle &root_nh, ros::NodeHandle &hw_nh,
            const std::vector< std::string > &motor_names);
  virtual void doSwitch(const std::list< hardware_interface::ControllerInfo > &start_list,
                        const std::list< hardware_interface::ControllerInfo > &stop_list);
  virtual void read(const ros::Time &time, const ros::Duration &period);
  virtual void write(const ros::Time &time, const ros::Duration &period);
  void updateDiagnostics();

private:
  // subfunctions for init()
  void initInterfaces();
  void initMotors(ros::NodeHandle &hw_nh, const std::vector< std::string > &motor_names);
  void initTransmissions(const std::string &urdf_str);
  void initJointLimits(const std::string &urdf_str);

private:
  ros::NodeHandle root_nh_;

  // low level interfaces motors actually have
  hardware_interface::ActuatorStateInterface ator_state_iface_;
  hardware_interface::PositionActuatorInterface pos_ator_iface_;
  hardware_interface::VelocityActuatorInterface vel_ator_iface_;
  hardware_interface::EffortActuatorInterface eff_ator_iface_;
  battery_state_interface::BatteryStateInterface bat_state_iface_;
  EposDiagnosticInterface epos_diag_iface_;

  // bridge between actuator and joint interfaces
  transmission_interface::RobotTransmissions robot_trans_;
  boost::scoped_ptr< transmission_interface::TransmissionInterfaceLoader > trans_iface_loader_;

  // limits related to joint interfaces
  dynamic_joint_limits_interface::PositionJointSaturationInterface pos_jnt_sat_iface_;
  dynamic_joint_limits_interface::VelocityJointSaturationInterface vel_jnt_sat_iface_;
  dynamic_joint_limits_interface::EffortJointSaturationInterface eff_jnt_sat_iface_;

  // motor hardware
  EposManager epos_manager_;
};

} // namespace eposx_hardware

#endif
