#include <eposx_hardware/epos_manager.h>

#include <boost/foreach.hpp>

namespace eposx_hardware {

EposManager::EposManager() {}

EposManager::~EposManager() {}

void EposManager::init(hardware_interface::RobotHW &hw, ros::NodeHandle &root_nh,
                       ros::NodeHandle &motors_nh, const std::vector< std::string > &motor_names) {
  BOOST_FOREACH (const std::string &motor_name, motor_names) {
    ROS_INFO_STREAM("Loading EPOS: " << motor_name);
    ros::NodeHandle motor_nh(motors_nh, motor_name);

    boost::shared_ptr< Epos > motor(new Epos());
    motor->init(hw, root_nh, motor_nh, motor_name);
    motors_.push_back(motor);

    boost::shared_ptr< EposDiagnosticUpdater > diagnostic_updater(new EposDiagnosticUpdater());
    diagnostic_updater->init(hw, root_nh, motor_nh, motor_name);
    diagnostic_updaters_.push_back(diagnostic_updater);
  }
}

void EposManager::doSwitch(const std::list< hardware_interface::ControllerInfo > &start_list,
                           const std::list< hardware_interface::ControllerInfo > &stop_list) {
  BOOST_FOREACH (const boost::shared_ptr< Epos > &motor, motors_) {
    motor->doSwitch(start_list, stop_list);
  }
}

void EposManager::read() {
  BOOST_FOREACH (const boost::shared_ptr< Epos > &motor, motors_) { motor->read(); }
}

void EposManager::write() {
  BOOST_FOREACH (const boost::shared_ptr< Epos > &motor, motors_) { motor->write(); }
}

void EposManager::updateDiagnostics() {
  BOOST_FOREACH (const boost::shared_ptr< EposDiagnosticUpdater > &diagnostic_updater_,
                 diagnostic_updaters_) {
    diagnostic_updater_->update();
  }
}

} // namespace eposx_hardware
