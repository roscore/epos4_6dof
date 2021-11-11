#ifndef EPOSX_HARDWARE_UTILS_H_
#define EPOSX_HARDWARE_UTILS_H_

#include <stdexcept>
#include <string>
#include <vector>

#include <eposx_library/Definitions.h>
#include <ros/node_handle.h>

#include <boost/cstdint.hpp>
#include <boost/shared_ptr.hpp>

namespace eposx_hardware {

//
// exception which this c++ wrapper may throw
//

class EposException : public std::runtime_error {
public:
  EposException(const std::string &what_arg);
  EposException(const std::string &what_arg, const unsigned int error_code);
  virtual ~EposException() throw();

  bool hasErrorCode() const;
  unsigned int getErrorCode() const;

  static std::string toErrorInfo(const unsigned int error_code);

private:
  bool has_error_code_;
  unsigned int error_code_;
};

//
// information of device (node chain)
//

class DeviceInfo {
public:
  DeviceInfo();
  DeviceInfo(const std::string &device_name, const std::string &protocol_stack_name,
             const std::string &interface_name, const std::string &port_name);
  virtual ~DeviceInfo();

public:
  std::string device_name;
  std::string protocol_stack_name;
  std::string interface_name;
  std::string port_name;
};

//
// handle of device (node chain) which finalizes itself on destruction
//

class DeviceHandle {
public:
  DeviceHandle();
  DeviceHandle(const DeviceInfo &device_info);
  virtual ~DeviceHandle();

private:
  static boost::shared_ptr< void > makePtr(const DeviceInfo &device_info);
  static void *openDevice(const DeviceInfo &device_info);
  static void closeDevice(void *ptr);

public:
  boost::shared_ptr< void > ptr;
};

//
// information of node
//

class NodeInfo : public DeviceInfo {
public:
  NodeInfo();
  NodeInfo(const DeviceInfo &device_info, const unsigned short node_id);
  virtual ~NodeInfo();

public:
  unsigned short node_id;
  boost::uint64_t serial_number;
  unsigned short hardware_version;
  unsigned short software_version;
  unsigned short application_number;
  unsigned short application_version;
};

//
// handle of node
//

class NodeHandle : public DeviceHandle {
public:
  NodeHandle();
  NodeHandle(const NodeInfo &node_info);
  NodeHandle(const DeviceHandle &device_handle, unsigned short node_id);
  virtual ~NodeHandle();

public:
  unsigned short node_id;
};

//
// DeviceInfo helper functions
//

std::vector< std::string > getDeviceNameList();

std::vector< std::string > getProtocolStackNameList(const std::string &device_name);

std::vector< std::string > getInterfaceNameList(const std::string &device_name,
                                                const std::string &protocol_stack_name);

std::vector< std::string > getPortNameList(const std::string &device_name,
                                           const std::string &protocol_stack_name,
                                           const std::string &interface_name);

std::vector< unsigned int > getBaudrateList(const std::string &device_name,
                                            const std::string &protocol_stack_name,
                                            const std::string &interface_name,
                                            const std::string &port_name);

std::vector< DeviceInfo > enumerateDevices(const std::string &device_name,
                                           const std::string &protocol_stack_name,
                                           const std::string &interface_name);

//
// DeviceHandle helper functions
//

std::string getDeviceName(const DeviceHandle &device_handle);

std::string getProtocolStackName(const DeviceHandle &device_handle);

std::string getInterfaceName(const DeviceHandle &device_handle);

std::string getPortName(const DeviceHandle &device_handle);

//
// NodeInfo helper functions
//

#define MAX_NODE_ID 127 // range of node id is [1,127]

// list existing nodes assuming port name and/or node id may be missing
std::vector< NodeInfo > enumerateNodes(const DeviceInfo &device_info, const unsigned short node_id,
                                       const unsigned short max_node_id = MAX_NODE_ID);

//
// NodeHandle helper functions
//

// create if the node can be identified by some of port name, node id, and serial number
NodeHandle createNodeHandle(const DeviceInfo &device_info, const unsigned short node_id,
                            const boost::uint64_t serial_number,
                            const unsigned short max_node_id = MAX_NODE_ID);

boost::uint64_t getSerialNumber(const NodeHandle &node_handle);

} // namespace eposx_hardware

//
// useful macros
//

// boolean value in VCS_xxx functions
#define VCS_FALSE 0

// call a VCS_xxx function or die
#define VCS(func, ...)                                                                             \
  do {                                                                                             \
    unsigned int _error_code;                                                                      \
    if (VCS_##func(__VA_ARGS__, &_error_code) == VCS_FALSE) {                                      \
      throw ::eposx_hardware::EposException(#func, _error_code);                                    \
    }                                                                                              \
  } while (false)

// call a VCS_xxx function with eposx_hardware::DeviceHandle or die
#define VCS_DN(func, epos_device_handle, ...) VCS(func, epos_device_handle.ptr.get(), __VA_ARGS__)

// call a VCS_xxx function with eposx_hardware::NodeHandle or die (no more arguments)
#define VCS_N0(func, epos_node_handle) VCS_DN(func, epos_node_handle, epos_node_handle.node_id)

// call a VCS_xxx function with eposx_hardware::NodeHandle or die
#define VCS_NN(func, epos_node_handle, ...)                                                        \
  VCS_DN(func, epos_node_handle, epos_node_handle.node_id, __VA_ARGS__)

// call a VCS_XxxObject function with eposx_hardware::NodeHandle or die
#define VCS_OBJ(func, epos_node_handle, index, subindex, data, length)                             \
  do {                                                                                             \
    unsigned int _bytes_transferred;                                                               \
    VCS_NN(func, epos_node_handle, index, subindex, data, length, &_bytes_transferred);            \
  } while (false)

// get a ros param with given key and value pair or die
#define GET_PARAM_KV(ros_node_handle, name, value)                                                 \
  do {                                                                                             \
    if (!ros_node_handle.getParam(name, value)) {                                                  \
      throw EposException("ros::NodeHandle::getParam(" + ros_node_handle.resolveName(name) + ")"); \
    }                                                                                              \
  } while (false)

// get a ros param or die, assuming the param name is same as the value name
#define GET_PARAM_V(ros_node_handle, value) GET_PARAM_KV(ros_node_handle, #value, value)

#endif
