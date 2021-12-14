epos4_hardware
============

# Origin Repo is ...
https://github.com/RIVeR-Lab/epos_hardware
https://github.com/yoshito-n-students/eposx_hardware

The copyright of the original code belongs to the repository of the link above. In the event of copyright or license problems in the future, this repository may be deleted without notice.

# eposx_hardware is ...
* ROS interface for [Maxon EPOS motor drivers](https://www.maxonmotor.com/maxon/view/content/EPOS-Detailsite)
* forked from [RIVeR-Lab/epos_hardware](https://github.com/RIVeR-Lab/epos_hardware), with a lot of enhancements and refactors
* difference from the original package
  * supports all EPOS drivers (EPOS/EPOS2/EPOS4)
  * supports all phisycal interface to EPOS (USB/RS232/CANOpen)
  * supports online switching of EPOS control modes
  * supports reading status of EPOS's power supply
  * supports joint limit interface
  * uses the latest offical command library from Maxon in backend (as of Jun. 2018)
  * refactored codes including an useful C++ wrapper of backend C library

# Node: epos_hardware_node
## Usage
`rosrun eposx_hardware epos_hardware_node [motor_name...]`
* [motor-specific parameters](#parameters) must be properly set

## Parameters
* all parameters below are motor-specific and must be in the namespace `~motor_name`

`device` (string, default: "EPOS4")
* type of device like "EPOS", "EPOS2", or "EPOS4"

`protocol_stack` (string, default: "MAXON SERIAL V2")
* type of protocol stack like "MAXON SERIAL V2", or "MAXON RS232"

`interface` (string, default: "USB")
* type of physical interface like "USB", "RS232", or "CANOpen"

`port` (string, default: "")
* path to physical port like "/dev/ttyUSB0"
* if empty string, all possible port will be scanned

`node_id` (int, default: 0)
* EPOS's node id
* if 0, all possible node indices will be tried

`serial_number` (string, default: "")
* EPOS's serial number in hex without "0x"
* if empty string, `port` and/or `node_id` will be used to search the device

`baudrate` (int, default: 0)
* baudrate of communication via physical interface
* if 0, keep current baudrate
* ignored if another device belonging to the interface is already initialized

`timeout` (int, default: 0)
* timeout of communication via physical interface in ms
* if 0, keep current timeout
* ignored if another device belonging to the interface is already initialized

`clear_faults` (bool, default: false)
* clear faults recorded in the device on startup

`rw_ros_units` (bool, default: false)
* use ROS standard units (rad, rad/s, Nm) in hardware interfaces or EPOS standard units (quad count of encoder pulse(qc), rpm, mNm)

`detailed_diagnostic` (bool, default: false)
* additionally read actual operation mode, device status, and fault info

remaining parameters wiil be described soon

# Commandline tool: list_nodes
will be described soon

# Commandline tool: get_state
will be described soon

# Examples
* see [eposx_hardware/launch](eposx_hardware/launch)
