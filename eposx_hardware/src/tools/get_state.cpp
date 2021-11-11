#include <ios>
#include <iostream>
#include <sstream>
#include <string>

#include <eposx_hardware/utils.h>
#include <eposx_library/Definitions.h>

#include <boost/cstdint.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/program_options/errors.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/value_semantic.hpp>
#include <boost/program_options/variables_map.hpp>

namespace eh = eposx_hardware;
namespace bpo = boost::program_options;

int main(int argc, char *argv[]) {
  eh::DeviceInfo device_info;
  std::string serial_number_str;
  unsigned short node_id, max_node_id;
  try {
    // define available options
    bpo::options_description options;
    bool show_help;
    options.add(
        boost::make_shared< bpo::option_description >("help", bpo::bool_switch(&show_help)));
    options.add(boost::make_shared< bpo::option_description >(
        "device", bpo::value(&device_info.device_name)->default_value("EPOS4")));
    options.add(boost::make_shared< bpo::option_description >(
        "protocol-stack",
        bpo::value(&device_info.protocol_stack_name)->default_value("MAXON SERIAL V2")));
    options.add(boost::make_shared< bpo::option_description >(
        "interface", bpo::value(&device_info.interface_name)->default_value("USB")));
    options.add(boost::make_shared< bpo::option_description >(
        "port", bpo::value(&device_info.port_name)->default_value("USB1")));
    options.add(boost::make_shared< bpo::option_description >(
        "serial-number", bpo::value(&serial_number_str)->default_value("0x602137010479")));
    options.add(boost::make_shared< bpo::option_description >(
        "node-id", bpo::value(&node_id)->default_value(1)));
    options.add(boost::make_shared< bpo::option_description >(
        "max-node-id", bpo::value(&max_node_id)->default_value(8)));
    // parse the command line
    bpo::variables_map args;
    bpo::store(bpo::parse_command_line(argc, argv, options), args);
    bpo::notify(args);
    // show help if requested
    if (show_help) {
      std::cout << "Available options:\n" << options << std::endl;
      return 0;
    }
  } catch (const bpo::error &error) {
    std::cerr << "Error: " << error.what() << std::endl;
    return 1;
  }

  boost::uint64_t serial_number;
  {
    std::istringstream iss(serial_number_str);
    iss >> std::hex >> serial_number;
    if (!iss) {
      std::cerr << "Error: Invalid serial number (" << serial_number_str << ")" << std::endl;
      return 1;
    }
  }

  std::cout << "Identifing a node for\n"
            << "  device: " << device_info.device_name << "\n"
            << "  protocol stack: " << device_info.protocol_stack_name << "\n"
            << "  interface: " << device_info.interface_name << "\n"
            << "  port (ignored if empty): " << device_info.port_name << "\n"
            << "  node id (ignored if 0): " << node_id << "\n"
            << "  serial number (ignored if 0): " << serial_number_str << std::endl;

  try {
    eh::NodeHandle epos_handle(
        eh::createNodeHandle(device_info, node_id, serial_number, max_node_id));

    int position;
    VCS_NN(GetPositionIs, epos_handle, &position);
    std::cout << "Position: " << std::dec << position << std::endl;

    int velocity;
    VCS_NN(GetVelocityIs, epos_handle, &velocity);
    std::cout << "Velocity: " << std::dec << velocity << std::endl;

    short current;
    VCS_NN(GetCurrentIs, epos_handle, &current);
    std::cout << "Current: " << std::dec << current << std::endl;
  } catch (const eh::EposException &error) {
    std::cerr << "Error: " << error.what() << std::endl;
    return 1;
  }

  return 0;
}
