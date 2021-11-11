#include <string>
#include <vector>

#include <controller_manager/controller_manager.h>
#include <eposx_hardware/epos_hardware.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/spinner.h>
#include <ros/time.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "epos_hardware");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::vector< std::string > motor_names;
  ros::removeROSArgs(argc, argv, motor_names);
  motor_names.erase(motor_names.begin()); // remove exec path

  eposx_hardware::EposHardware hardware;
  if (!hardware.init(nh, pnh, motor_names)) {
    ROS_FATAL("Failed to initialize motors");
    return 1;
  }
  ROS_INFO("Motors Initialized");

  controller_manager::ControllerManager controllers(&hardware, nh);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate control_rate(50);
  ros::Time last(ros::Time::now());
  while (ros::ok()) {
    const ros::Time now(ros::Time::now());
    const ros::Duration period(now - last);
    hardware.read(now, period);
    controllers.update(now, period);
    hardware.write(now, period);
    hardware.updateDiagnostics();
    last = now;
    control_rate.sleep();
  }
}
