#include <set>
#include <stdexcept>

#include <eposx_hardware/epos_hardware.h>
#include <eposx_hardware/utils.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <ros/console.h>
#include <transmission_interface/transmission_info.h>
#include <urdf/model.h>

#include <boost/foreach.hpp>

namespace eposx_hardware {

EposHardware::EposHardware() {}

EposHardware::~EposHardware() {}

//
// init()
//

bool EposHardware::init(ros::NodeHandle &root_nh, ros::NodeHandle &hw_nh,
                        const std::vector< std::string > &motor_names) {
  root_nh_ = root_nh;

  // wait for URDF which contains transmission and limits information
  std::string urdf_str;
  root_nh_.getParam("robot_description", urdf_str);
  while (urdf_str.empty() && ros::ok()) {
    ROS_INFO_STREAM_ONCE("Waiting for robot_description");
    root_nh_.getParam("robot_description", urdf_str);
    ros::Duration(0.1).sleep();
  }

  try {
    initInterfaces();
    initMotors(hw_nh, motor_names);
    initTransmissions(urdf_str);
    initJointLimits(urdf_str);
  } catch (const std::exception &error) {
    ROS_ERROR_STREAM(error.what());
    return false;
  }
  return true;
}

void EposHardware::initInterfaces() {
  registerInterface(&ator_state_iface_);
  registerInterface(&pos_ator_iface_);
  registerInterface(&vel_ator_iface_);
  registerInterface(&eff_ator_iface_);
  registerInterface(&bat_state_iface_);
  registerInterface(&epos_diag_iface_);
  registerInterface(&pos_jnt_sat_iface_);
  registerInterface(&vel_jnt_sat_iface_);
  registerInterface(&eff_jnt_sat_iface_);
}

void EposHardware::initMotors(ros::NodeHandle &hw_nh,
                              const std::vector< std::string > &motor_names) {
  // register state/command/diagnostic handles to hardware interfaces
  // and configure motors
  epos_manager_.init(*this, root_nh_, hw_nh, motor_names);
}

// helper function to populate actuator names registered in interfaces
template < typename ActuatorInterface >
void insertNames(std::set< std::string > &names, const ActuatorInterface &ator_iface) {
  const std::vector< std::string > new_names(ator_iface.getNames());
  names.insert(new_names.begin(), new_names.end());
}

void EposHardware::initTransmissions(const std::string &urdf_str) {
  // load transmission infomations from URDF
  transmission_interface::TransmissionParser trans_parser;
  std::vector< transmission_interface::TransmissionInfo > trans_infos;
  if (!trans_parser.parse(urdf_str, trans_infos)) {
    throw EposException("Failed to parse urdf");
  }

  // build a list of actuator names in this hardware
  std::set< std::string > hw_ator_names;
  insertNames(hw_ator_names, ator_state_iface_);
  insertNames(hw_ator_names, pos_ator_iface_);
  insertNames(hw_ator_names, vel_ator_iface_);
  insertNames(hw_ator_names, eff_ator_iface_);

  // load all transmissions that are for the motors in this hardware
  trans_iface_loader_.reset(
      new transmission_interface::TransmissionInterfaceLoader(this, &robot_trans_));
  BOOST_FOREACH (const transmission_interface::TransmissionInfo &trans_info, trans_infos) {
    // check the transmission is for some of actuators in this hardware
    bool trans_has_non_epos_ator(false);
    BOOST_FOREACH (const transmission_interface::ActuatorInfo &trans_ator, trans_info.actuators_) {
      if (hw_ator_names.count(trans_ator.name_) == 0) {
        trans_has_non_epos_ator = true;
        break;
      }
    }
    if (trans_has_non_epos_ator) {
      ROS_INFO_STREAM("Skip loading " << trans_info.name_ << " because it has non epos actuator");
      continue;
    }
    // load the transmission
    if (!trans_iface_loader_->load(trans_info)) {
      throw EposException("Failed to load " + trans_info.name_);
    }
    ROS_INFO_STREAM("Loaded transmission: " << trans_info.name_);
  }
}

// register commnad saturation handles to the saturation interface.
// each handle corresponds to a command handle in the command interface.
// each handle is initialized by the URDF description if possible.
template < typename SaturationHandle, typename CommandInterface, typename SaturationInterface >
void registerHandles(CommandInterface &cmd_iface, SaturationInterface &sat_iface,
                     const urdf::Model &urdf_model) {
  const std::vector< std::string > jnt_names(cmd_iface.getNames());
  BOOST_FOREACH (const std::string &jnt_name, jnt_names) {
    joint_limits_interface::JointLimits jnt_limits;
    joint_limits_interface::getJointLimits(
        urdf_model.getJoint(jnt_name) /* this is ptr. null is ok */, jnt_limits);
    sat_iface.registerHandle(SaturationHandle(cmd_iface.getHandle(jnt_name), jnt_limits));
  }
}

void EposHardware::initJointLimits(const std::string &urdf_str) {
  // make sure joint command data is available
  if (!trans_iface_loader_) {
    throw EposException("Null transmission loader");
  }
  transmission_interface::TransmissionLoaderData *const trans_loader_data(
      trans_iface_loader_->getData());
  if (!trans_loader_data) {
    throw EposException("Null transmission loader data");
  }

  // load URDF model which contains joint limits information
  urdf::Model urdf_model;
  if (!urdf_model.initString(urdf_str)) {
    throw EposException("Failed to init URDF model");
  }

  // initialize limits by URDF & register all possible joint limits
  transmission_interface::JointInterfaces &jnt_ifaces(trans_loader_data->joint_interfaces);
  registerHandles< dynamic_joint_limits_interface::PositionJointSaturationHandle >(
      jnt_ifaces.position_joint_interface, pos_jnt_sat_iface_, urdf_model);
  registerHandles< dynamic_joint_limits_interface::VelocityJointSaturationHandle >(
      jnt_ifaces.velocity_joint_interface, vel_jnt_sat_iface_, urdf_model);
  registerHandles< dynamic_joint_limits_interface::EffortJointSaturationHandle >(
      jnt_ifaces.effort_joint_interface, eff_jnt_sat_iface_, urdf_model);

  // do first update of limits which performs blocking access to the parameter server
  pos_jnt_sat_iface_.updateLimits(root_nh_);
  vel_jnt_sat_iface_.updateLimits(root_nh_);
  eff_jnt_sat_iface_.updateLimits(root_nh_);
}

//
// doSwitch()
//

void EposHardware::doSwitch(const std::list< hardware_interface::ControllerInfo > &start_list,
                            const std::list< hardware_interface::ControllerInfo > &stop_list) {
  epos_manager_.doSwitch(start_list, stop_list);
}

//
// read()
//

// helper function to propagate between joint and actuator states/commands
template < typename BridgeInterface >
void propagate(transmission_interface::RobotTransmissions &robot_trans) {
  BridgeInterface *const bridge_iface(robot_trans.get< BridgeInterface >());
  if (bridge_iface) {
    bridge_iface->propagate();
  }
}

void EposHardware::read(const ros::Time &time, const ros::Duration &period) {
  // read actutor states
  epos_manager_.read();

  // update joint stats by actuator states
  propagate< transmission_interface::ActuatorToJointStateInterface >(robot_trans_);
}

//
// write()
//

void EposHardware::write(const ros::Time &time, const ros::Duration &period) {
  // update limits with cached parameters subscribed in background
  pos_jnt_sat_iface_.updateLimits(root_nh_);
  vel_jnt_sat_iface_.updateLimits(root_nh_);
  eff_jnt_sat_iface_.updateLimits(root_nh_);

  // saturate joint commands
  pos_jnt_sat_iface_.enforceLimits(period);
  vel_jnt_sat_iface_.enforceLimits(period);
  eff_jnt_sat_iface_.enforceLimits(period);

  // update actuator commands by joint commands
  propagate< transmission_interface::JointToActuatorVelocityInterface >(robot_trans_);
  propagate< transmission_interface::JointToActuatorPositionInterface >(robot_trans_);
  propagate< transmission_interface::JointToActuatorEffortInterface >(robot_trans_);

  // write actuator commands
  epos_manager_.write();
}

//
// updateDiagnostics()
//

void EposHardware::updateDiagnostics() { epos_manager_.updateDiagnostics(); }

} // namespace eposx_hardware
