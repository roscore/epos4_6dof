#ifndef EPOSX_HARDWARE_EPOS_MANAGER_H_
#define EPOSX_HARDWARE_EPOS_MANAGER_H_

#include <list>
#include <string>
#include <vector>

#include <eposx_hardware/epos.h>
#include <eposx_hardware/epos_diagnostic_updater.h>
#include <hardware_interface/controller_info.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>

#include <boost/shared_ptr.hpp>

namespace eposx_hardware {

class EposManager {
public:
  EposManager();
  virtual ~EposManager();

  void init(hardware_interface::RobotHW &hw, ros::NodeHandle &root_nh, ros::NodeHandle &motors_nh,
            const std::vector< std::string > &motor_names);
  void read();
  void write();
  void doSwitch(const std::list< hardware_interface::ControllerInfo > &start_list,
                const std::list< hardware_interface::ControllerInfo > &stop_list);
  void updateDiagnostics();

private:
  std::vector< boost::shared_ptr< Epos > > motors_;
  std::vector< boost::shared_ptr< EposDiagnosticUpdater > > diagnostic_updaters_;
};

} // namespace eposx_hardware

#endif
