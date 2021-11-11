#ifndef EPOSX_HARDWARE_EPOS_DIAGNOSTIC_UPDATER_H
#define EPOSX_HARDWARE_EPOS_DIAGNOSTIC_UPDATER_H

#include <string>
#include <vector>

#include <diagnostic_updater/DiagnosticStatusWrapper.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <eposx_hardware/epos_operation_mode.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>

#include <boost/cstdint.hpp>
#include <boost/scoped_ptr.hpp>

namespace eposx_hardware {

struct EposDiagnosticData {
  boost::int8_t operation_mode_display;
  boost::uint16_t statusword;
  std::vector< unsigned int > device_errors;
};

class EposDiagnosticHandle {
public:
  EposDiagnosticHandle() : name_(), data_(NULL) {}
  EposDiagnosticHandle(const std::string &name, const EposDiagnosticData *data)
      : name_(name), data_(data) {}
  virtual ~EposDiagnosticHandle() {}

  std::string getName() const { return name_; }
  EposDiagnosticData getData() const { return *data_; }
  const EposDiagnosticData *getDataPtr() { return data_; }

private:
  std::string name_;
  const EposDiagnosticData *data_;
};

class EposDiagnosticInterface
    : public hardware_interface::HardwareResourceManager< EposDiagnosticHandle > {};

class EposDiagnosticUpdater {
public:
  EposDiagnosticUpdater();
  virtual ~EposDiagnosticUpdater();

  void init(hardware_interface::RobotHW &hw, ros::NodeHandle &root_nh, ros::NodeHandle &motor_nh,
            const std::string &motor_name);
  void update();

private:
  void updateMotorDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);
  void updateMotorOutputDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);

private:
  std::string motor_name_;

  boost::scoped_ptr< diagnostic_updater::Updater > diagnostic_updater_;

  bool rw_ros_units_;
  double torque_constant_, nominal_current_, max_output_current_;

  const double *position_, *velocity_, *effort_;
  const double *position_cmd_, *velocity_cmd_, *effort_cmd_;
  const EposDiagnosticData *diagnostic_data_;
};

} // namespace eposx_hardware

#endif