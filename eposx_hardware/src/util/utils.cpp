#include <eposx_hardware/utils.h>

#include <ios>
#include <map>
#include <sstream>

#include <boost/foreach.hpp>
#include <boost/weak_ptr.hpp>

#include <ros/console.h>

namespace eposx_hardware {

//
// EposException
//

EposException::EposException(const std::string &what_arg)
    : std::runtime_error(what_arg), has_error_code_(false), error_code_(0) {}

EposException::EposException(const std::string &what_arg, const unsigned int error_code)
    : std::runtime_error(what_arg + " (" + toErrorInfo(error_code) + ")"), has_error_code_(true),
      error_code_(error_code) {}

EposException::~EposException() throw() {}

bool EposException::hasErrorCode() const { return has_error_code_; }

unsigned int EposException::getErrorCode() const { return error_code_; }

std::string EposException::toErrorInfo(const unsigned int error_code) {
  std::ostringstream oss;
  oss << "0x" << std::hex << error_code;
  char error_info[1024];
  if (VCS_GetErrorInfo(error_code, error_info, 1024) != VCS_FALSE) {
    oss << ": " << error_info;
  }
  return oss.str();
}

//
// DeviceInfo
//

DeviceInfo::DeviceInfo() : device_name(), protocol_stack_name(), interface_name(), port_name() {}

DeviceInfo::DeviceInfo(const std::string &device_name, const std::string &protocol_stack_name,
                       const std::string &interface_name, const std::string &port_name)
    : device_name(device_name), protocol_stack_name(protocol_stack_name),
      interface_name(interface_name), port_name(port_name) {}

DeviceInfo::~DeviceInfo() {}

struct LessDeviceInfo {
  bool operator()(const DeviceInfo &a, const DeviceInfo &b) const {
    if (a.device_name != b.device_name) {
      return a.device_name < b.device_name;
    }
    if (a.protocol_stack_name != b.protocol_stack_name) {
      return a.protocol_stack_name < b.protocol_stack_name;
    }
    if (a.interface_name != b.interface_name) {
      return a.interface_name < b.interface_name;
    }
    return a.port_name < b.port_name;
  }
};

//
// DeviceHandle
//

DeviceHandle::DeviceHandle() : ptr() {}

DeviceHandle::DeviceHandle(const DeviceInfo &device_info) : ptr(makePtr(device_info)) {}

DeviceHandle::~DeviceHandle() {}

boost::shared_ptr< void > DeviceHandle::makePtr(const DeviceInfo &device_info) {
  // shared storage of opened devices
  static std::map< DeviceInfo, boost::weak_ptr< void >, LessDeviceInfo > existing_device_ptrs;

  // try find an existing device
  const boost::shared_ptr< void > existing_device_ptr(existing_device_ptrs[device_info].lock());
  if (existing_device_ptr) {
    return existing_device_ptr;
  }
  // open new device if not exists
  const boost::shared_ptr< void > new_device_ptr(/*raw ptr*/ openDevice(device_info),
                                                 /*deleter*/ closeDevice);
  existing_device_ptrs[device_info] = new_device_ptr;
  return new_device_ptr;
}

void *DeviceHandle::openDevice(const DeviceInfo &device_info) {
  unsigned int error_code;
  void *const raw_device_ptr(
      VCS_OpenDevice(const_cast< char * >(device_info.device_name.c_str()),
                     const_cast< char * >(device_info.protocol_stack_name.c_str()),
                     const_cast< char * >(device_info.interface_name.c_str()),
                     const_cast< char * >(device_info.port_name.c_str()), &error_code));
  if (!raw_device_ptr) {
    throw EposException("OpenDevice", error_code);
  }
  return raw_device_ptr;
}

void DeviceHandle::closeDevice(void *raw_device_ptr) {
  unsigned int error_code;
  if (VCS_CloseDevice(raw_device_ptr, &error_code) == VCS_FALSE) {
    // deleter of shared_ptr must not throw
    ROS_ERROR_STREAM("CloseDevice (" + EposException::toErrorInfo(error_code) + ")");
  }
}

//
// NodeInfo
//

NodeInfo::NodeInfo() : DeviceInfo(), node_id(0) {}

NodeInfo::NodeInfo(const DeviceInfo &device_info, const unsigned short node_id)
    : DeviceInfo(device_info), node_id(node_id) {}

NodeInfo::~NodeInfo() {}

//
// NodeHandle
//

NodeHandle::NodeHandle() : DeviceHandle(), node_id(0) {}

NodeHandle::NodeHandle(const NodeInfo &node_info)
    : DeviceHandle(node_info), node_id(node_info.node_id) {}

NodeHandle::NodeHandle(const DeviceHandle &device_handle, unsigned short node_id)
    : DeviceHandle(device_handle), node_id(node_id) {}

NodeHandle::~NodeHandle() {}

//
// DeviceInfo helper functions
//

std::vector< std::string > getDeviceNameList() {
  char buffer[1024];
  int end_of_selection; // BOOL
  std::vector< std::string > device_names;
  VCS(GetDeviceNameSelection, true /* request start of selection */, buffer, 1024,
      &end_of_selection);
  device_names.push_back(buffer);
  while (end_of_selection == VCS_FALSE) {
    VCS(GetDeviceNameSelection, false /* request the next selection */, buffer, 1024,
        &end_of_selection);
    device_names.push_back(buffer);
  }
  return device_names;
}

std::vector< std::string > getProtocolStackNameList(const std::string &device_name) {
  char buffer[1024];
  int end_of_selection; // BOOL
  std::vector< std::string > protocol_stack_names;
  VCS(GetProtocolStackNameSelection, const_cast< char * >(device_name.c_str()), true, buffer, 1024,
      &end_of_selection);
  protocol_stack_names.push_back(buffer);
  while (end_of_selection == VCS_FALSE) {
    VCS(GetProtocolStackNameSelection, const_cast< char * >(device_name.c_str()), false, buffer,
        1024, &end_of_selection);
    protocol_stack_names.push_back(buffer);
  }
  return protocol_stack_names;
}

std::vector< std::string > getInterfaceNameList(const std::string &device_name,
                                                const std::string &protocol_stack_name) {
  char buffer[1024];
  int end_of_selection; // BOOL
  std::vector< std::string > interface_names;
  VCS(GetInterfaceNameSelection, const_cast< char * >(device_name.c_str()),
      const_cast< char * >(protocol_stack_name.c_str()), true, buffer, 1024, &end_of_selection);
  interface_names.push_back(buffer);
  while (end_of_selection == VCS_FALSE) {
    VCS(GetInterfaceNameSelection, const_cast< char * >(device_name.c_str()),
        const_cast< char * >(protocol_stack_name.c_str()), false, buffer, 1024, &end_of_selection);
    interface_names.push_back(buffer);
  }
  return interface_names;
}

std::vector< std::string > getPortNameList(const std::string &device_name,
                                           const std::string &protocol_stack_name,
                                           const std::string &interface_name) {
  char buffer[1024];
  int end_of_selection; // BOOL
  std::vector< std::string > port_names;
  VCS(GetPortNameSelection, const_cast< char * >(device_name.c_str()),
      const_cast< char * >(protocol_stack_name.c_str()),
      const_cast< char * >(interface_name.c_str()), true, buffer, 1024, &end_of_selection);
  port_names.push_back(buffer);
  while (end_of_selection == VCS_FALSE) {
    VCS(GetPortNameSelection, const_cast< char * >(device_name.c_str()),
        const_cast< char * >(protocol_stack_name.c_str()),
        const_cast< char * >(interface_name.c_str()), false, buffer, 1024, &end_of_selection);
    port_names.push_back(buffer);
  }
  return port_names;
}

std::vector< unsigned int > getBaudrateList(const std::string &device_name,
                                            const std::string &protocol_stack_name,
                                            const std::string &interface_name,
                                            const std::string &port_name) {
  unsigned int baudrate;
  int end_of_selection; // BOOL
  std::vector< unsigned int > baudrates;
  VCS(GetBaudrateSelection, const_cast< char * >(device_name.c_str()),
      const_cast< char * >(protocol_stack_name.c_str()),
      const_cast< char * >(interface_name.c_str()), const_cast< char * >(port_name.c_str()), true,
      &baudrate, &end_of_selection);
  baudrates.push_back(baudrate);
  while (end_of_selection == VCS_FALSE) {
    VCS(GetBaudrateSelection, const_cast< char * >(device_name.c_str()),
        const_cast< char * >(protocol_stack_name.c_str()),
        const_cast< char * >(interface_name.c_str()), const_cast< char * >(port_name.c_str()),
        false, &baudrate, &end_of_selection);
    baudrates.push_back(baudrate);
  }
  return baudrates;
}

std::vector< DeviceInfo > enumerateDevices(const std::string &device_name,
                                           const std::string &protocol_stack_name,
                                           const std::string &interface_name) {
  std::vector< DeviceInfo > device_infos;
  const std::vector< std::string > port_names(
      getPortNameList(device_name, protocol_stack_name, interface_name));
  BOOST_FOREACH (const std::string &port_name, port_names) {
    device_infos.push_back(DeviceInfo(device_name, protocol_stack_name, interface_name, port_name));
  }
  return device_infos;
}

//
// DeviceHandle helper functions
//

std::string getDeviceName(const DeviceHandle &device_handle) {
  char buffer[1024];
  VCS_DN(GetDeviceName, device_handle, buffer, 1024);
  return buffer;
}

std::string getProtocolStackName(const DeviceHandle &device_handle) {
  char buffer[1024];
  VCS_DN(GetProtocolStackName, device_handle, buffer, 1024);
  return buffer;
}

std::string getInterfaceName(const DeviceHandle &device_handle) {
  char buffer[1024];
  VCS_DN(GetInterfaceName, device_handle, buffer, 1024);
  return buffer;
}

std::string getPortName(const DeviceHandle &device_handle) {
  char buffer[1024];
  VCS_DN(GetPortName, device_handle, buffer, 1024);
  return buffer;
}

//
// NodeInfo helper functions
//

std::vector< NodeInfo > enumerateNodes(const DeviceInfo &device_info, const unsigned short node_id,
                                       const unsigned short max_node_id) {
  // enumerate all possible devices (assuming port name may be missed)
  const std::vector< DeviceInfo > possible_device_infos(
      device_info.port_name.empty()
          ? enumerateDevices(device_info.device_name, device_info.protocol_stack_name,
                             device_info.interface_name)
          : std::vector< DeviceInfo >(1, device_info));

  // enumerate all possible nodes (assuming node id may be missed)
  std::vector< NodeInfo > possible_node_infos;
  BOOST_FOREACH (const DeviceInfo &possible_device_info, possible_device_infos) {
    if (node_id == 0) {
      for (unsigned short possible_node_id = 1; possible_node_id <= max_node_id;
           ++possible_node_id) {
        possible_node_infos.push_back(NodeInfo(possible_device_info, possible_node_id));
      }
    } else {
      possible_node_infos.push_back(NodeInfo(possible_device_info, node_id));
    }
  }

  // try access all possible nodes to filter existing nodes
  std::vector< NodeInfo > existing_node_infos;
  BOOST_FOREACH (const NodeInfo &possible_node_info, possible_node_infos) {
    try {
      NodeInfo node_info(possible_node_info);
      NodeHandle node_handle(node_info);
      VCS_NN(GetVersion, node_handle, &node_info.hardware_version, &node_info.software_version,
             &node_info.application_number, &node_info.application_version);
      node_info.serial_number = getSerialNumber(node_handle);
      existing_node_infos.push_back(node_info);
    } catch (const EposException &) {
      // node does not exist
      continue;
    }
  }
  return existing_node_infos;
}

//
// NodeHandle helper functions
//

NodeHandle createNodeHandle(const DeviceInfo &device_info, const unsigned short node_id,
                            const boost::uint64_t serial_number, const unsigned short max_node_id) {
  // get existing node infos
  const std::vector< NodeInfo > node_infos(enumerateNodes(device_info, node_id, max_node_id));

  // identify the node (assuming serial number may be missed)
  if (serial_number == 0) {
    if (node_infos.size() == 1) {
      return NodeHandle(node_infos.front());
    }
  } else {
    BOOST_FOREACH (const NodeInfo &node_info, node_infos) {
      if (node_info.serial_number == serial_number) {
        return NodeHandle(node_info);
      }
    }
  }

  throw EposException("createNodeHandle (Could not identify node)");
}

boost::uint64_t getSerialNumber(const NodeHandle &node_handle) {
  const std::string device_name(getDeviceName(node_handle));
  boost::uint64_t serial_number;
  if (device_name == "EPOS") {
    VCS_OBJ(GetObject, node_handle, 0x2004, 0x00, &serial_number, 8);
  } else if (device_name == "EPOS2") {
    VCS_OBJ(GetObject, node_handle, 0x2004, 0x00, &serial_number, 8);
  } else if (device_name == "EPOS4") {
    VCS_OBJ(GetObject, node_handle, 0x2100, 0x01, &serial_number, 8);
  } else {
    throw EposException("getSerialNumber (Unsupported device name \"" + device_name + "\")");
  }
  return serial_number;
}

} // namespace eposx_hardware
