#include <ios>
#include <limits>
#include <sstream>
#include <typeinfo>

#include <battery_state_interface/battery_state_interface.hpp>
#include <eposx_hardware/epos.h>
#include <eposx_hardware/epos_diagnostic_updater.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>

#include <boost/bind.hpp>
#include <boost/core/demangle.hpp>
#include <boost/cstdint.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

namespace eposx_hardware {

Epos::Epos() : position_(0), velocity_(0), effort_(0), current_(0) {}

Epos::~Epos() {
  try {
    VCS_N0(SetDisableState, epos_handle_);
  } catch (const EposException &error) {
    ROS_ERROR_STREAM(motor_name_ << " (id=" << epos_handle_.node_id << "): " << error.what());
  }
}

//
// init() and subfunctions
//

void Epos::init(hardware_interface::RobotHW &hw, ros::NodeHandle &root_nh,
                ros::NodeHandle &motor_nh, const std::string &motor_name) {
  motor_name_ = motor_name;

  initHardwareInterface(hw, motor_nh);

  initEposNodeHandle(motor_nh);
  initProtocolStackSettings(motor_nh);

  VCS_N0(SetDisableState, epos_handle_);

  initOperationMode(hw, root_nh, motor_nh);
  initFaultReaction(motor_nh);
  initMotorParameter(motor_nh);
  initSensorParameter(motor_nh);
  initSafetyParameter(motor_nh);
  initPositionRegulator(motor_nh);
  initVelocityRegulator(motor_nh);
  initCurrentRegulator(motor_nh);
  initPositionProfile(motor_nh);
  initVelocityProfile(motor_nh);
  initDeviceError(motor_nh);
  initMiscParameters(motor_nh);

  VCS_N0(SetEnableState, epos_handle_);
}

// helper function to register a handle to a hardware interface in hardware
template < typename HWInterface, typename HWHandle >
void registerTo(hardware_interface::RobotHW &hw, const HWHandle &hw_handle) {
  HWInterface *const hw_interface(hw.get< HWInterface >());
  if (!hw_interface) {
    throw EposException("No " + boost::core::demangle(typeid(HWInterface).name()));
  }
  hw_interface->registerHandle(hw_handle);
}

void Epos::initHardwareInterface(hardware_interface::RobotHW &hw, ros::NodeHandle &motor_nh) {
  namespace bsi = battery_state_interface;
  namespace hi = hardware_interface;

  // register actuator state handle
  registerTo< hi::ActuatorStateInterface >(
      hw, hi::ActuatorStateHandle(motor_name_, &position_, &velocity_, &effort_));

  // register diagnostic handle
  if (motor_nh.param("detailed_diagnostic", false)) {
    diagnostic_data_.reset(new EposDiagnosticData);
    registerTo< EposDiagnosticInterface >(
        hw, EposDiagnosticHandle(motor_name_, diagnostic_data_.get()));
  }

  // if power_supply/name is given, additionally register power supply hardware
  std::string power_supply_name;
  if (motor_nh.getParam("power_supply/name", power_supply_name)) {
    power_supply_state_.reset(new sensor_msgs::BatteryState);
    registerTo< bsi::BatteryStateInterface >(
        hw, bsi::BatteryStateHandle(power_supply_name, power_supply_state_.get()));
  }
}

void Epos::initEposNodeHandle(ros::NodeHandle &motor_nh) {
  // load optional device info
  const DeviceInfo device_info(motor_nh.param< std::string >("device", "EPOS4"),
                               motor_nh.param< std::string >("protocol_stack", "MAXON SERIAL V2"),
                               motor_nh.param< std::string >("interface", "USB"),
                               motor_nh.param< std::string >("port", ""));
  const unsigned short node_id(motor_nh.param("node_id", 0));
  const std::string serial_number_str(motor_nh.param< std::string >("serial_number", "0"));

  // serial number from string
  boost::uint64_t serial_number;
  {
    std::istringstream iss(serial_number_str);
    iss >> std::hex >> serial_number;
    if (!iss) {
      throw EposException("Invalid serial number (" + serial_number_str + ")");
    }
  }

  // create epos handle
  epos_handle_ = createNodeHandle(device_info, node_id, serial_number);
}

void Epos::initProtocolStackSettings(ros::NodeHandle &motor_nh) {
  // load optional settings
  const unsigned int baudrate(motor_nh.param("baudrate", 0));
  const unsigned int timeout(motor_nh.param("timeout", 0));
  if (baudrate == 0 && timeout == 0) {
    return;
  }

  // check if the node is the first one initialized in the device
  if (epos_handle_.ptr.use_count() != 1) {
    ROS_WARN_STREAM(
        motor_nh.getNamespace()
        << "/{baudrate,timeout} is ignored. "
        << "Only the first-initialized node in a device can set protocol stack settings.");
    return;
  }

  // apply settings
  if (baudrate > 0 && timeout > 0) {
    VCS(SetProtocolStackSettings, epos_handle_.ptr.get(), baudrate, timeout);
  } else {
    unsigned int current_baudrate, current_timeout;
    VCS(GetProtocolStackSettings, epos_handle_.ptr.get(), &current_baudrate, &current_timeout);
    VCS(SetProtocolStackSettings, epos_handle_.ptr.get(),
        baudrate > 0 ? baudrate : current_baudrate, timeout > 0 ? timeout : current_timeout);
  }
}

void Epos::initOperationMode(hardware_interface::RobotHW &hw, ros::NodeHandle &root_nh,
                             ros::NodeHandle &motor_nh) {
  // load map from ros-controller name to epos's operation mode name
  typedef std::map< std::string, std::string > StrMap;
  StrMap str_map;
  GET_PARAM_KV(motor_nh, "operation_mode_map", str_map);

  // make map from mode name to mode object
  typedef std::map< std::string, boost::shared_ptr< EposOperationMode > > PtrMap;
  PtrMap ptr_map;
  BOOST_FOREACH (const StrMap::value_type &str_pair, str_map) {
    if (ptr_map.count(str_pair.second) > 0) {
      continue;
    }
    boost::shared_ptr< EposOperationMode > mode;
    if (str_pair.second == "profile_position") {
      mode.reset(new EposProfilePositionMode());
    } else if (str_pair.second == "profile_velocity") {
      mode.reset(new EposProfileVelocityMode());
    } else if (str_pair.second == "current") {
      mode.reset(new EposCurrentMode());
    } else if (str_pair.second == "cyclic_synchronoust_torque") {
      mode.reset(new EposCyclicSynchronoustTorqueMode());
    } else {
      throw EposException("Unsupported operation mode (" + str_pair.second + ")");
    }
    mode->init(hw, root_nh, motor_nh, motor_name_, epos_handle_);
    ptr_map[str_pair.second] = mode;
  }

  // set operation_mode_map_ from controller name to mode object
  BOOST_FOREACH (const StrMap::value_type &str_pair, str_map) {
    operation_mode_map_[str_pair.first] = ptr_map[str_pair.second];
  }
}

void Epos::initFaultReaction(ros::NodeHandle &motor_nh) {
  // try load fault reaction param
  std::string fault_reaction_str;
  if (!motor_nh.getParam("fault_reaction_option", fault_reaction_str)) {
    return;
  }

  // check fault reaction is supported by the device name
  const std::string device_name(getDeviceName(epos_handle_));
  if (device_name != "EPOS2" && device_name != "EPOS4") {
    ROS_WARN_STREAM("Skip initializing fault reaction on "
                    << motor_name_ << " because " << device_name
                    << " does not support fault reaction options");
    return;
  }

  // set fault reaction
  if (fault_reaction_str == "signal_only") {
    boost::int16_t data(-1);
    VCS_OBJ(SetObject, epos_handle_, 0x605E, 0x00, &data, 2);
  } else if (fault_reaction_str == "disable_drive") {
    boost::int16_t data(0);
    VCS_OBJ(SetObject, epos_handle_, 0x605E, 0x00, &data, 2);
  } else if (fault_reaction_str == "slow_down_ramp") {
    boost::int16_t data(1);
    VCS_OBJ(SetObject, epos_handle_, 0x605E, 0x00, &data, 2);
  } else if (fault_reaction_str == "slow_down_quickstop") {
    boost::int16_t data(2);
    VCS_OBJ(SetObject, epos_handle_, 0x605E, 0x00, &data, 2);
  } else {
    throw EposException("Invalid fault reaction option (" + fault_reaction_str + ")");
  }
}

void Epos::initMotorParameter(ros::NodeHandle &motor_nh) {
  ros::NodeHandle motor_param_nh(motor_nh, "motor");
  // set motor type
  int type;
  GET_PARAM_V(motor_param_nh, type);
  VCS_NN(SetMotorType, epos_handle_, type);
  // set motor parameters
  if (type == 1 /* DC MOTOR */) {
    double nominal_current, max_output_current, thermal_time_constant;
    GET_PARAM_V(motor_param_nh, nominal_current);
    GET_PARAM_V(motor_param_nh, max_output_current);
    GET_PARAM_V(motor_param_nh, thermal_time_constant);
    VCS_NN(SetDcMotorParameter, epos_handle_,
           static_cast< int >(1000 * nominal_current),      // A -> mA
           static_cast< int >(1000 * max_output_current),   // A -> mA
           static_cast< int >(10 * thermal_time_constant)); // s -> 100ms
  } else if (type == 10 || type == 11 /*EC MOTOR*/) {
    double nominal_current, max_output_current, thermal_time_constant;
    int number_of_pole_pairs;
    GET_PARAM_V(motor_param_nh, nominal_current);
    GET_PARAM_V(motor_param_nh, max_output_current);
    GET_PARAM_V(motor_param_nh, thermal_time_constant);
    GET_PARAM_V(motor_param_nh, number_of_pole_pairs);
    VCS_NN(SetEcMotorParameter, epos_handle_,
           static_cast< int >(1000 * nominal_current),     // A -> mA
           static_cast< int >(1000 * max_output_current),  // A -> mA
           static_cast< int >(10 * thermal_time_constant), // s -> 100ms
           number_of_pole_pairs);
  } else {
    throw EposException("Invalid motor type (" + boost::lexical_cast< std::string >(type) + ")");
  }
  // set motor max speed
  double max_speed;
  if (motor_param_nh.getParam("max_speed", max_speed)) {
    const std::string device_name(getDeviceName(epos_handle_));
    if (device_name == "EPOS2") {
      boost::uint32_t data(max_speed);
      VCS_OBJ(SetObject, epos_handle_, 0x6410, 0x04, &data, 4);
    } else if (device_name == "EPOS4") {
      boost::uint32_t data(max_speed);
      VCS_OBJ(SetObject, epos_handle_, 0x6080, 0x00, &data, 4);
    } else {
      ROS_WARN_STREAM("Skip initializing max motor speed on " << motor_name_ << " because "
                                                              << device_name
                                                              << " does not support this function");
    }
  }
}

void Epos::initSensorParameter(ros::NodeHandle &motor_nh) {
  ros::NodeHandle sensor_nh(motor_nh, "sensor");
  // set sensor type
  int type;
  GET_PARAM_V(sensor_nh, type);
  VCS_NN(SetSensorType, epos_handle_, type);
  // set sensor parameters (TODO: support hall sensors)
  encoder_resolution_ = 0;
  if (type == 1 || type == 2 /* INC ENCODER */) {
    bool inverted_polarity;
    GET_PARAM_KV(sensor_nh, "resolution", encoder_resolution_);
    GET_PARAM_V(sensor_nh, inverted_polarity);
    VCS_NN(SetIncEncoderParameter, epos_handle_, encoder_resolution_, inverted_polarity);
    if (inverted_polarity) {
      encoder_resolution_ = -encoder_resolution_;
    }
  } else if (type == 4 || type == 5 /* SSI ABS ENCODER */) {
    int data_rate, number_of_multiturn_bits, number_of_singleturn_bits;
    bool inverted_polarity;
    GET_PARAM_V(sensor_nh, data_rate);
    GET_PARAM_V(sensor_nh, number_of_multiturn_bits);
    GET_PARAM_V(sensor_nh, number_of_singleturn_bits);
    GET_PARAM_V(sensor_nh, inverted_polarity);
    VCS_NN(SetSsiAbsEncoderParameter, epos_handle_, data_rate, number_of_multiturn_bits,
           number_of_singleturn_bits, inverted_polarity);
    if (inverted_polarity) {
      encoder_resolution_ = -(1 << number_of_singleturn_bits);
    } else {
      encoder_resolution_ = (1 << number_of_singleturn_bits);
    }
  } else {
    throw EposException("Invalid sensor type (" + boost::lexical_cast< std::string >(type) + ")");
  }
}

void Epos::initSafetyParameter(ros::NodeHandle &motor_nh) {
  ros::NodeHandle safety_nh(motor_nh, "safety");

  int max_following_error;
  GET_PARAM_V(safety_nh, max_following_error);
  VCS_NN(SetMaxFollowingError, epos_handle_, max_following_error);

  int max_profile_velocity;
  GET_PARAM_V(safety_nh, max_profile_velocity);
  VCS_NN(SetMaxProfileVelocity, epos_handle_, max_profile_velocity);

  int max_acceleration;
  GET_PARAM_V(safety_nh, max_acceleration);
  VCS_NN(SetMaxAcceleration, epos_handle_, max_acceleration);
}

void Epos::initPositionRegulator(ros::NodeHandle &motor_nh) {
  ros::NodeHandle position_regulator_nh(motor_nh, "position_regulator");
  if (position_regulator_nh.hasParam("gain")) {
    ros::NodeHandle gain_nh(position_regulator_nh, "gain");
    int p, i, d;
    GET_PARAM_V(gain_nh, p);
    GET_PARAM_V(gain_nh, i);
    GET_PARAM_V(gain_nh, d);
    VCS_NN(SetPositionRegulatorGain, epos_handle_, p, i, d);
  }
  if (position_regulator_nh.hasParam("feed_forward")) {
    ros::NodeHandle feed_forward_nh(position_regulator_nh, "feed_forward");
    int velocity, acceleration;
    GET_PARAM_V(feed_forward_nh, velocity);
    GET_PARAM_V(feed_forward_nh, acceleration);
    VCS_NN(SetPositionRegulatorFeedForward, epos_handle_, velocity, acceleration);
  }
}

void Epos::initVelocityRegulator(ros::NodeHandle &motor_nh) {
  ros::NodeHandle velocity_regulator_nh(motor_nh, "velocity_regulator");
  if (velocity_regulator_nh.hasParam("gain")) {
    ros::NodeHandle gain_nh(velocity_regulator_nh, "gain");
    int p, i;
    GET_PARAM_V(gain_nh, p);
    GET_PARAM_V(gain_nh, i);
    VCS_NN(SetVelocityRegulatorGain, epos_handle_, p, i);
  }
  if (velocity_regulator_nh.hasParam("feed_forward")) {
    ros::NodeHandle feed_forward_nh(velocity_regulator_nh, "feed_forward");
    int velocity, acceleration;
    GET_PARAM_V(feed_forward_nh, velocity);
    GET_PARAM_V(feed_forward_nh, acceleration);
    VCS_NN(SetVelocityRegulatorFeedForward, epos_handle_, velocity, acceleration);
  }
}

void Epos::initCurrentRegulator(ros::NodeHandle &motor_nh) {
  ros::NodeHandle current_regulator_nh(motor_nh, "current_regulator");
  if (current_regulator_nh.hasParam("gain")) {
    ros::NodeHandle gain_nh(current_regulator_nh, "gain");
    int p, i;
    GET_PARAM_V(gain_nh, p);
    GET_PARAM_V(gain_nh, i);
    VCS_NN(SetCurrentRegulatorGain, epos_handle_, p, i);
  }
}

void Epos::initPositionProfile(ros::NodeHandle &motor_nh) {
  ros::NodeHandle position_profile_nh(motor_nh, "position_profile");
  if (position_profile_nh.hasParam("velocity")) {
    int velocity, acceleration, deceleration;
    GET_PARAM_V(position_profile_nh, velocity);
    GET_PARAM_V(position_profile_nh, acceleration);
    GET_PARAM_V(position_profile_nh, deceleration);
    VCS_NN(SetPositionProfile, epos_handle_, velocity, acceleration, deceleration);
  }
  if (position_profile_nh.hasParam("window")) {
    ros::NodeHandle window_nh(position_profile_nh, "window");
    int window;
    double time;
    GET_PARAM_V(window_nh, window);
    GET_PARAM_V(window_nh, time);
    VCS_NN(EnablePositionWindow, epos_handle_, window,
           static_cast< int >(1000 * time) /* s -> ms */);
  }
}

void Epos::initVelocityProfile(ros::NodeHandle &motor_nh) {
  ros::NodeHandle velocity_profile_nh(motor_nh, "velocity_profile");
  if (velocity_profile_nh.hasParam("acceleration")) {
    int acceleration, deceleration;
    GET_PARAM_V(velocity_profile_nh, acceleration);
    GET_PARAM_V(velocity_profile_nh, deceleration);
    VCS_NN(SetVelocityProfile, epos_handle_, acceleration, deceleration);
  }
  if (velocity_profile_nh.hasParam("window")) {
    ros::NodeHandle window_nh(velocity_profile_nh, "window");
    int window;
    double time;
    GET_PARAM_V(window_nh, window);
    GET_PARAM_V(window_nh, time);
    VCS_NN(EnableVelocityWindow, epos_handle_, window,
           static_cast< int >(1000 * time) /* s -> ms */);
  }
}

void Epos::initDeviceError(ros::NodeHandle &motor_nh) {
  unsigned char num_device_errors;
  VCS_NN(GetNbOfDeviceError, epos_handle_, &num_device_errors);
  for (int i = 1; i <= num_device_errors; ++i) {
    unsigned int device_error_code;
    VCS_NN(GetDeviceErrorCode, epos_handle_, i, &device_error_code);
    ROS_WARN_STREAM(motor_name_ << " (id=" << epos_handle_.node_id << "): "
                                << "EPOS Device Error: 0x" << std::hex << device_error_code);
  }

  if (motor_nh.param("clear_faults", false)) {
    VCS_N0(ClearFault, epos_handle_);
  }

  VCS_NN(GetNbOfDeviceError, epos_handle_, &num_device_errors);
  if (num_device_errors > 0) {
    throw EposException(boost::lexical_cast< std::string >(num_device_errors) +
                        " faults uncleared on the device");
  }
}

void Epos::initMiscParameters(ros::NodeHandle &motor_nh) {
  // constant whose unit is mNm/A
  GET_PARAM_KV(motor_nh, "motor/torque_constant", torque_constant_);

  // unit of outgoing states
  motor_nh.param("rw_ros_units", rw_ros_units_, false);

  // constants in battery state
  if (power_supply_state_) {
    power_supply_state_->power_supply_technology = motor_nh.param< int >(
        "power_supply/technology", sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN);
    power_supply_state_->location = motor_nh.param< std::string >("power_supply/location", "");
    power_supply_state_->serial_number =
        motor_nh.param< std::string >("power_supply/serial_number", "");
  }
}

//
// doSwitch()
//

void Epos::doSwitch(const std::list< hardware_interface::ControllerInfo > &start_list,
                    const std::list< hardware_interface::ControllerInfo > &stop_list) {
  // switch epos's operation mode according to starting controllers
  BOOST_FOREACH (const hardware_interface::ControllerInfo &starting_controller, start_list) {
    const OperationModeMap::const_iterator mode_to_switch(
        operation_mode_map_.find(starting_controller.name));
    if (mode_to_switch == operation_mode_map_.end()) {
      continue;
    }
    try {
      mode_to_switch->second->activate();
      operation_mode_ = mode_to_switch->second;
      ROS_INFO_STREAM(motor_name_ << " switched to operation mode associated with "
                                  << mode_to_switch->first);
    } catch (const EposException &error) {
      ROS_ERROR_STREAM(motor_name_ << " (id=" << epos_handle_.node_id << "): " << error.what());
    }
  }
}

//
// read() and subfunctions
//

void Epos::read() {
  try {
    if (operation_mode_) {
      operation_mode_->read();
    }
    readJointState();
    readPowerSupply();
    readDiagnostic();
  } catch (const EposException &error) {
    ROS_ERROR_STREAM(motor_name_ << " (id=" << epos_handle_.node_id << "): " << error.what());
  }
}

void Epos::readJointState() {
  int position_raw;
  int velocity_raw;
  short current_raw;
  VCS_NN(GetPositionIs, epos_handle_, &position_raw);
  VCS_NN(GetVelocityIs, epos_handle_, &velocity_raw);
  VCS_NN(GetCurrentIs, epos_handle_, &current_raw);
  if (rw_ros_units_) {
    // quad-counts of the encoder -> rad
    position_ = position_raw * M_PI / (2. * encoder_resolution_);
    // rpm -> rad/s
    velocity_ = velocity_raw * M_PI / 30.;
    // mA -> A
    current_ = current_raw / 1000.;
    // mNm -> Nm
    effort_ = torque_constant_ * current_ / 1000.;
  } else {
    position_ = position_raw;
    velocity_ = velocity_raw;
    current_ = current_raw / 1000.0; // mA -> A
    effort_ = torque_constant_ * current_;
  }
}

void Epos::readPowerSupply() {
  if (!power_supply_state_) {
    return;
  }

  const std::string device_name(getDeviceName(epos_handle_));
  if (device_name == "EPOS4") {
    boost::uint16_t voltage10x;
    VCS_OBJ(GetObject, epos_handle_, 0x2200, 0x01, &voltage10x, 2);
    // measured variables
    power_supply_state_->voltage = voltage10x / 10.;
    power_supply_state_->present = true;
  } else {
    ROS_WARN_STREAM_ONCE("Power supply voltage of " << motor_name_ << " cannot be measured because "
                                                    << device_name
                                                    << " does not offer voltage information");
    // read something from the node to make sure power supply is present
    boost::uint16_t statusword;
    VCS_OBJ(GetObject, epos_handle_, 0x6041, 0x00, &statusword, 2);
    power_supply_state_->voltage = std::numeric_limits< float >::quiet_NaN();
    power_supply_state_->present = true;
  }
  // unmeasured variables
  power_supply_state_->current = std::numeric_limits< float >::quiet_NaN();
  power_supply_state_->charge = std::numeric_limits< float >::quiet_NaN();
  power_supply_state_->capacity = std::numeric_limits< float >::quiet_NaN();
  power_supply_state_->design_capacity = std::numeric_limits< float >::quiet_NaN();
  power_supply_state_->percentage = std::numeric_limits< float >::quiet_NaN();
  power_supply_state_->power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  power_supply_state_->power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
}

void Epos::readDiagnostic() {
  if (!diagnostic_data_) {
    return;
  }

  // read actual operation mode (this is common in all types of devices)
  VCS_OBJ(GetObject, epos_handle_, 0x6061, 0x00, &diagnostic_data_->operation_mode_display, 1);

  // read statusword (this is common in all types of devices)
  VCS_OBJ(GetObject, epos_handle_, 0x6041, 0x00, &diagnostic_data_->statusword, 2);

  // read fault info
  unsigned char num_device_errors;
  VCS_NN(GetNbOfDeviceError, epos_handle_, &num_device_errors);
  diagnostic_data_->device_errors.resize(num_device_errors, 0);
  for (unsigned char i = 1; i <= num_device_errors; ++i) {
    VCS_NN(GetDeviceErrorCode, epos_handle_, i, &diagnostic_data_->device_errors[i]);
  }
}

//
// write() and subfunctions
//

void Epos::write() {
  try {
    if (operation_mode_) {
      operation_mode_->write();
    }
  } catch (const EposException &error) {
    ROS_ERROR_STREAM(motor_name_ << " (id=" << epos_handle_.node_id << "): " << error.what());
  }
}

} // namespace eposx_hardware
