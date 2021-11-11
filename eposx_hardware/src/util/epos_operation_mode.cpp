#include <algorithm>
#include <cmath>
#include <typeinfo>

#include <eposx_hardware/epos_operation_mode.h>
#include <eposx_hardware/utils.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface/transmission_parser.h>

#include <boost/core/demangle.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

namespace eposx_hardware {

//
// util
//

template < typename CommandInterface >
void registerHandleTo(hardware_interface::RobotHW &hw, const std::string &motor_name,
                      double *const command) {
  // make sure hardware has state interface
  hardware_interface::ActuatorStateInterface *const state_interface(
      hw.get< hardware_interface::ActuatorStateInterface >());
  if (!state_interface) {
    throw EposException("No ActuatorStateInterface");
  }

  // make sure state handle already registered
  const std::vector< std::string > state_names(state_interface->getNames());
  if (std::find(state_names.begin(), state_names.end(), motor_name) == state_names.end()) {
    throw EposException("No ActuatorStateHandle named " + motor_name);
  }

  // make sure hardware has command interface
  CommandInterface *const command_interface(hw.get< CommandInterface >());
  if (!command_interface) {
    throw EposException("No " + boost::core::demangle(typeid(CommandInterface).name()));
  }

  // register command handle
  command_interface->registerHandle(
      hardware_interface::ActuatorHandle(state_interface->getHandle(motor_name), command));
}

// helper function to load actuator-joint mappings from urdf
std::vector< transmission_interface::TransmissionInfo >
getTransmissionInfos(ros::NodeHandle &urdf_nh) {
  typedef std::vector< transmission_interface::TransmissionInfo > TransmissionInfos;
  typedef std::map< std::string, TransmissionInfos > TransmissionInfosMap;
  static TransmissionInfosMap trans_infos_map;

  const std::string urdf_ns(urdf_nh.getNamespace());
  if (trans_infos_map.count(urdf_ns) == 0) {
    // load urdf
    std::string urdf_str;
    urdf_nh.getParam("robot_description", urdf_str);
    while (urdf_str.empty() && ros::ok()) {
      ROS_INFO_STREAM_ONCE("Waiting for robot_description");
      urdf_nh.getParam("robot_description", urdf_str);
      ros::Duration(0.1).sleep();
    }

    // load transmission infos which map joints and actuators from urdf
    transmission_interface::TransmissionParser trans_parser;
    TransmissionInfos trans_infos;
    if (!trans_parser.parse(urdf_str, trans_infos)) {
      throw EposException("Failed to parse urdf");
    }
    trans_infos_map[urdf_ns] = trans_infos;
  }

  return trans_infos_map[urdf_ns];
}

// helper function to get joint names corresponding actuator name in urdf
std::vector< std::string > getJointNames(ros::NodeHandle &urdf_nh,
                                         const std::string &actuator_name) {
  // get actuator-joint mappings
  const std::vector< transmission_interface::TransmissionInfo > trans_infos(
      getTransmissionInfos(urdf_nh));

  // pick names of joints related to the actuator
  std::vector< std::string > joint_names;
  BOOST_FOREACH (const transmission_interface::TransmissionInfo &trans_info, trans_infos) {
    bool has_actuator(false);
    BOOST_FOREACH (const transmission_interface::ActuatorInfo &actuator_info,
                   trans_info.actuators_) {
      if (actuator_info.name_ == actuator_name) {
        has_actuator = true;
        break;
      }
    }
    if (!has_actuator) {
      continue;
    }
    BOOST_FOREACH (const transmission_interface::JointInfo &joint_info, trans_info.joints_) {
      if (std::find(joint_names.begin(), joint_names.end(), joint_info.name_) ==
          joint_names.end()) {
        joint_names.push_back(joint_info.name_);
      }
    }
  }

  return joint_names;
}

//
// operation mode base
//

EposOperationMode::~EposOperationMode() {}

//
// profile position mode
//

EposProfilePositionMode::~EposProfilePositionMode() {}

void EposProfilePositionMode::init(hardware_interface::RobotHW &hw, ros::NodeHandle &root_nh,
                                   ros::NodeHandle &motor_nh, const std::string &motor_name,
                                   eposx_hardware::NodeHandle &epos_handle) {
  // register position command handle
  registerHandleTo< hardware_interface::PositionActuatorInterface >(hw, motor_name, &position_cmd_);

  // init objects required when the mode is activated
  joint_names_ = getJointNames(root_nh, motor_name);
  pos_sat_iface_ = hw.get< dynamic_joint_limits_interface::PositionJointSaturationInterface >();

  // init epos handle
  epos_handle_ = epos_handle;
  const std::string device_name(getDeviceName(epos_handle_));
  if (device_name != "EPOS" && device_name != "EPOS2" && device_name != "EPOS4") {
    throw EposException(device_name + " does not support profile position mode");
  }

  // use ros unit for position command
  rw_ros_units_ = motor_nh.param("rw_ros_units", false);

  // get encoder resolution for unit conversion
  if (rw_ros_units_) {
    ros::NodeHandle sensor_nh(motor_nh, "sensor");
    int type;
    GET_PARAM_V(sensor_nh, type);
    if (type == 1 || type == 2 /* INC ENCODER */) {
      int resolution;
      bool inverted_polarity;
      GET_PARAM_V(sensor_nh, resolution);
      GET_PARAM_V(sensor_nh, inverted_polarity);
      encoder_resolution_ = inverted_polarity ? -resolution : resolution;
    } else if (type == 4 || type == 5 /* SSI ABS ENCODER */) {
      int number_of_singleturn_bits;
      bool inverted_polarity;
      GET_PARAM_V(sensor_nh, number_of_singleturn_bits);
      GET_PARAM_V(sensor_nh, inverted_polarity);
      encoder_resolution_ =
          inverted_polarity ? -(1 << number_of_singleturn_bits) : (1 << number_of_singleturn_bits);
    } else {
      throw EposException("Invalid sensor type (" + boost::lexical_cast< std::string >(type) + ")");
    }
  }
}

void EposProfilePositionMode::activate() {
  if (pos_sat_iface_) {
    // reset command saturation handle because position version is stateful.
    // we don't have to reset velocity & effort versions.
    BOOST_FOREACH (const std::string &joint_name, joint_names_) {
      pos_sat_iface_->reset(joint_name);
    }
  }
  VCS_N0(ActivateProfilePositionMode, epos_handle_);
}

void EposProfilePositionMode::read() { /* nothing to do */
}

void EposProfilePositionMode::write() {
  if (std::isnan(position_cmd_)) {
    return;
  }

  int cmd;
  if (rw_ros_units_) {
    // rad -> quad-counts of the encoder
    cmd = static_cast< int >(position_cmd_ * 2. * encoder_resolution_ / M_PI);
  } else {
    cmd = static_cast< int >(position_cmd_);
  }
  VCS_NN(MoveToPosition, epos_handle_, cmd, true /* target position is absolute */,
         true /* overwrite old target position */);
}

//
// profile velocity mode
//

EposProfileVelocityMode::~EposProfileVelocityMode() {}

void EposProfileVelocityMode::init(hardware_interface::RobotHW &hw, ros::NodeHandle &root_nh,
                                   ros::NodeHandle &motor_nh, const std::string &motor_name,
                                   eposx_hardware::NodeHandle &epos_handle) {
  // register velocity command handle
  registerHandleTo< hardware_interface::VelocityActuatorInterface >(hw, motor_name, &velocity_cmd_);

  // init epos handle
  epos_handle_ = epos_handle;
  const std::string device_name(getDeviceName(epos_handle_));
  if (device_name != "EPOS" && device_name != "EPOS2" && device_name != "EPOS4") {
    throw EposException(device_name + " does not support profile velocity mode");
  }

  // use ros unit for position command
  rw_ros_units_ = motor_nh.param("rw_ros_units", false);

  // halt velocity when command is 0
  halt_velocity_ = motor_nh.param("halt_velocity", false);
}

void EposProfileVelocityMode::activate() { VCS_N0(ActivateProfileVelocityMode, epos_handle_); }

void EposProfileVelocityMode::read() { /* nothing to do*/
}

void EposProfileVelocityMode::write() {
  if (std::isnan(velocity_cmd_)) {
    return;
  }

  int cmd;
  if (rw_ros_units_) {
    // rad/s -> rpm
    cmd = static_cast< int >(velocity_cmd_ * 30. / M_PI);
  } else {
    cmd = static_cast< int >(velocity_cmd_);
  }
  if (cmd == 0 && halt_velocity_) {
    VCS_N0(HaltVelocityMovement, epos_handle_);
  } else {
    VCS_NN(MoveWithVelocity, epos_handle_, cmd);
  }
}

//
// current mode
//

EposCurrentMode::~EposCurrentMode() {}

void EposCurrentMode::init(hardware_interface::RobotHW &hw, ros::NodeHandle &root_nh,
                           ros::NodeHandle &motor_nh, const std::string &motor_name,
                           eposx_hardware::NodeHandle &epos_handle) {
  // register effort command handle
  registerHandleTo< hardware_interface::EffortActuatorInterface >(hw, motor_name, &effort_cmd_);

  // init epos handle
  epos_handle_ = epos_handle;
  const std::string device_name(getDeviceName(epos_handle_));
  if (device_name != "EPOS" && device_name != "EPOS2") {
    throw EposException(device_name + " does not support current mode");
  }

  // use ros unit for position command
  rw_ros_units_ = motor_nh.param("rw_ros_units", false);

  // torque-current constant for unit conversion
  GET_PARAM_KV(motor_nh, "motor/torque_constant", torque_constant_);
}

void EposCurrentMode::activate() { VCS_N0(ActivateCurrentMode, epos_handle_); }

void EposCurrentMode::read() { /* nothing to do */
}

void EposCurrentMode::write() {
  if (std::isnan(effort_cmd_)) {
    return;
  }

  int cmd;
  if (rw_ros_units_) {
    // Nm -> mNm & A -> mA
    cmd = static_cast< int >((effort_cmd_ * 1000.) / torque_constant_ * 1000.);
  } else {
    // A -> mA
    cmd = static_cast< int >(effort_cmd_ / torque_constant_ * 1000.);
  }
  VCS_NN(SetCurrentMust, epos_handle_, cmd);
}

//
// cyclic synchronoust torque mode
//

EposCyclicSynchronoustTorqueMode::~EposCyclicSynchronoustTorqueMode() {}

void EposCyclicSynchronoustTorqueMode::init(hardware_interface::RobotHW &hw,
                                            ros::NodeHandle &root_nh, ros::NodeHandle &motor_nh,
                                            const std::string &motor_name,
                                            eposx_hardware::NodeHandle &epos_handle) {
  // register effort command handle
  registerHandleTo< hardware_interface::EffortActuatorInterface >(hw, motor_name, &effort_cmd_);

  // init epos handle
  epos_handle_ = epos_handle;
  const std::string device_name(getDeviceName(epos_handle_));
  if (device_name != "EPOS4") {
    throw EposException(device_name + " does not support cyclic synchronoust torque mode");
  }

  // use ros unit for position command
  rw_ros_units_ = motor_nh.param("rw_ros_units", false);

  // set torque constant for unit conversion in epos
  double torque_constant;
  GET_PARAM_KV(motor_nh, "motor/torque_constant", torque_constant);
  {
    // mAm/A -> uAm/A
    boost::uint32_t data(torque_constant * 1000.);
    VCS_OBJ(SetObject, epos_handle_, 0x3001, 0x05, &data, 4);
  }

  // load motor-rated-torque
  double nominal_current;
  GET_PARAM_KV(motor_nh, "motor/nominal_current", nominal_current);
  motor_rated_torque_ = nominal_current * torque_constant;
}

void EposCyclicSynchronoustTorqueMode::activate() { VCS_NN(SetOperationMode, epos_handle_, 10); }

void EposCyclicSynchronoustTorqueMode::read() { /* nothing to do */
}

void EposCyclicSynchronoustTorqueMode::write() {
  if (std::isnan(effort_cmd_)) {
    return;
  }

  boost::int16_t cmd;
  if (rw_ros_units_) {
    // Nm -> mNm -> per mille of motor rated torque
    cmd = static_cast< boost::int16_t >(effort_cmd_ * 1000. / motor_rated_torque_ * 1000.);
  } else {
    // mNm -> per mille of motor rated torque
    cmd = static_cast< boost::int16_t >(effort_cmd_ / motor_rated_torque_ * 1000.);
  }
  VCS_OBJ(SetObject, epos_handle_, 0x6071, 0x00, &cmd, 2);
}

} // namespace eposx_hardware