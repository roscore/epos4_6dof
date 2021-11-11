#include <ios>
#include <sstream>

#include <eposx_hardware/epos_diagnostic_updater.h>
#include <eposx_hardware/utils.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

namespace eposx_hardware {

EposDiagnosticUpdater::EposDiagnosticUpdater() {}

EposDiagnosticUpdater::~EposDiagnosticUpdater() {}

// helper function to get a hardware handle from an interface in the hardware
template < typename HWInterface, typename HWHandle >
bool getFrom(hardware_interface::RobotHW &hw, const std::string &motor_name, HWHandle &hw_handle) {
  // get an interface from the hardware
  HWInterface *const hw_interface(hw.get< HWInterface >());
  if (!hw_interface) {
    return false;
  }

  // check a handle exists in the interface by the handle's name
  const std::vector< std::string > hw_handle_names(hw_interface->getNames());
  if (std::find(hw_handle_names.begin(), hw_handle_names.end(), motor_name) ==
      hw_handle_names.end()) {
    return false;
  }

  // get the handle
  hw_handle = hw_interface->getHandle(motor_name);
  return true;
}

void EposDiagnosticUpdater::init(hardware_interface::RobotHW &hw, ros::NodeHandle &root_nh,
                                 ros::NodeHandle &motor_nh, const std::string &motor_name) {
  namespace hi = hardware_interface;

  // set motor name
  motor_name_ = motor_name;

  // try sniff motor state
  hi::ActuatorStateHandle motor_state_handle;
  if (getFrom< hi::ActuatorStateInterface >(hw, motor_name, motor_state_handle)) {
    position_ = motor_state_handle.getPositionPtr();
    velocity_ = motor_state_handle.getVelocityPtr();
    effort_ = motor_state_handle.getEffortPtr();
  } else {
    position_ = NULL;
    velocity_ = NULL;
    effort_ = NULL;
  }

  // try sniff position command
  hi::ActuatorHandle position_handle;
  if (getFrom< hi::PositionActuatorInterface >(hw, motor_name, position_handle)) {
    position_cmd_ = position_handle.getCommandPtr();
  } else {
    position_cmd_ = NULL;
  }

  // try sniff velocity command
  hi::ActuatorHandle velocity_handle;
  if (getFrom< hi::VelocityActuatorInterface >(hw, motor_name, velocity_handle)) {
    velocity_cmd_ = velocity_handle.getCommandPtr();
  } else {
    velocity_cmd_ = NULL;
  }

  // try sniff effort command
  hi::ActuatorHandle effort_handle;
  if (getFrom< hi::EffortActuatorInterface >(hw, motor_name, effort_handle)) {
    effort_cmd_ = effort_handle.getCommandPtr();
  } else {
    effort_cmd_ = NULL;
  }

  // try sniff diagnostic data
  EposDiagnosticHandle diagnostic_handle;
  if (getFrom< EposDiagnosticInterface >(hw, motor_name, diagnostic_handle)) {
    diagnostic_data_ = diagnostic_handle.getDataPtr();
  } else {
    diagnostic_data_ = NULL;
  }

  // units in states and commands
  rw_ros_units_ = motor_nh.param("rw_ros_units", false);

  // load motor params (are they required params for diagnostics??)
  GET_PARAM_KV(motor_nh, "motor/torque_constant", torque_constant_);
  GET_PARAM_KV(motor_nh, "motor/nominal_current", nominal_current_);
  GET_PARAM_KV(motor_nh, "motor/max_output_current", max_output_current_);

  // setup diagnostic updater
  diagnostic_updater_.reset(new diagnostic_updater::Updater(root_nh, motor_nh));
  diagnostic_updater_->setHardwareID("EPOS operating " + motor_name_);
  diagnostic_updater_->add(motor_name_ + ": Motor",
                           boost::bind(&EposDiagnosticUpdater::updateMotorDiagnostic, this, _1));
  diagnostic_updater_->add(
      motor_name_ + ": Motor Output",
      boost::bind(&EposDiagnosticUpdater::updateMotorOutputDiagnostic, this, _1));
}

void EposDiagnosticUpdater::update() { diagnostic_updater_->update(); }

#define STATUSWORD(b, v) ((v >> b) & 1)
#define READY_TO_SWITCH_ON (0)
#define SWITCHED_ON (1)
#define ENABLE (2)
#define FAULT (3)
#define VOLTAGE_ENABLED (4)
#define QUICKSTOP (5)
#define WARNING (7)
#define TARGET_REACHED (10)
#define CURRENT_LIMIT_ACTIVE (11)

void EposDiagnosticUpdater::updateMotorDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
  stat.add("Motor Name", motor_name_);

  if (diagnostic_data_) {
    const boost::uint16_t statusword(diagnostic_data_->statusword);
    const bool enabled = STATUSWORD(READY_TO_SWITCH_ON, statusword) &&
                         STATUSWORD(SWITCHED_ON, statusword) && STATUSWORD(ENABLE, statusword);
    if (enabled) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Enabled");
    } else {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Disabled");
    }

    // Quickstop is enabled when bit is unset (only read quickstop when enabled)
    if (!STATUSWORD(QUICKSTOP, statusword) && enabled) {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Quickstop");
    }

    if (STATUSWORD(WARNING, statusword)) {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Warning");
    }

    if (STATUSWORD(FAULT, statusword)) {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Fault");
    }

    stat.add< bool >("Enabled", STATUSWORD(ENABLE, statusword));
    stat.add< bool >("Fault", STATUSWORD(FAULT, statusword));
    stat.add< bool >("Voltage Enabled", STATUSWORD(VOLTAGE_ENABLED, statusword));
    stat.add< bool >("Quickstop", STATUSWORD(QUICKSTOP, statusword));
    stat.add< bool >("Warning", STATUSWORD(WARNING, statusword));
  } else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No status read");
  }

  if (diagnostic_data_) {
    BOOST_FOREACH (const unsigned int &device_error, diagnostic_data_->device_errors) {
      std::ostringstream error_msg;
      error_msg << "EPOS Device Error: 0x" << std::hex << device_error;
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, error_msg.str());
    }
  } else {
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "No device errors read");
  }
}

void EposDiagnosticUpdater::updateMotorOutputDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat) {
  stat.add("Motor Name", motor_name_);

  // add stat of operation mode name
  if (diagnostic_data_) {
    switch (diagnostic_data_->operation_mode_display) {
    case -6:
      stat.add("Operation Mode", "Step/Direction");
      break;
    case -5:
      stat.add("Operation Mode", "MasterEncoder");
      break;
    case -4:
      stat.add("Operation Mode", "Diagnostic");
      break;
    case -3:
      stat.add("Operation Mode", "Current");
      break;
    case -2:
      stat.add("Operation Mode", "Velocity");
      break;
    case -1:
      stat.add("Operation Mode", "Position");
      break;
    case 1:
      stat.add("Operation Mode", "Profile Position");
      break;
    case 3:
      stat.add("Operation Mode", "Profile Velocity");
      break;
    case 6:
      stat.add("Operation Mode", "Homing");
      break;
    case 7:
      stat.add("Operation Mode", "Interpolated Position");
      break;
    case 10:
      stat.add("Operation Mode", "Cyclic Synchronous Torque");
      break;
    default:
      stat.add("Operation Mode", "Unknown (" +
                                     boost::lexical_cast< std::string >(static_cast< int >(
                                         diagnostic_data_->operation_mode_display)) +
                                     ")");
      break;
    }
  }

  // add stats of commands
  if (position_cmd_) {
    stat.add("Commanded Position",
             boost::lexical_cast< std::string >(*position_cmd_) + (rw_ros_units_ ? " rad" : " qc"));
  }
  if (velocity_cmd_) {
    stat.add("Commanded Velocity", boost::lexical_cast< std::string >(*velocity_cmd_) +
                                       (rw_ros_units_ ? " rad/s" : " rpm"));
  }
  if (effort_cmd_) {
    stat.add("Commanded Effort",
             boost::lexical_cast< std::string >(*effort_cmd_) + (rw_ros_units_ ? " Nm" : " mNm"));
    stat.add("Commanded Current",
             boost::lexical_cast< std::string >(
                 (rw_ros_units_ ? *effort_cmd_ * 1000. : *effort_cmd_) / torque_constant_) +
                 " A");
  }

  stat.add("Torque Constant", boost::lexical_cast< std::string >(torque_constant_) + " mNm/A");
  stat.add("Nominal Current", boost::lexical_cast< std::string >(nominal_current_) + " A");
  stat.add("Max Output Current", boost::lexical_cast< std::string >(max_output_current_) + " A");

  // add stat of state
  if (position_) {
    stat.add("Position",
             boost::lexical_cast< std::string >(*position_) + (rw_ros_units_ ? " rad" : " qc"));
  }
  if (velocity_) {
    stat.add("Velocity",
             boost::lexical_cast< std::string >(*velocity_) + (rw_ros_units_ ? " rad/s" : " rpm"));
  }
  if (effort_) {
    stat.add("Effort",
             boost::lexical_cast< std::string >(*effort_) + (rw_ros_units_ ? " Nm" : " mNm"));
    stat.add("Current", boost::lexical_cast< std::string >(
                            (rw_ros_units_ ? *effort_ * 1000. : *effort_cmd_) / torque_constant_) +
                            " A");
  }

  // show status about motor operation
  if (diagnostic_data_) {
    const boost::uint16_t statusword(diagnostic_data_->statusword);
    if (STATUSWORD(CURRENT_LIMIT_ACTIVE, statusword)) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Current Limit Active");
    } else {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Operating within current limit");
    }
    stat.add< bool >("Target Reached", STATUSWORD(TARGET_REACHED, statusword));
    stat.add< bool >("Current Limit Active", STATUSWORD(CURRENT_LIMIT_ACTIVE, statusword));
  } else {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "No status read");
  }
}

} // namespace eposx_hardware