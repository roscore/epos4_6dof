#include <ios>
#include <iostream>

#include <eposx_hardware/utils.h>

#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include <boost/function.hpp>
#include <boost/make_shared.hpp>
#include <boost/program_options/errors.hpp>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/value_semantic.hpp>
#include <boost/program_options/variables_map.hpp>

namespace eh = eposx_hardware;
namespace bpo = boost::program_options;

// execute a given function in a try-catch barriar
template < typename T >
std::vector< T > barriar(const boost::function< std::vector< T >() > &func) {
  try {
    return func();
  } catch (const eh::EposException &error) {
    std::cerr << error.what() << std::endl;
  }
  return std::vector< T >();
}

int main(int argc, char *argv[]) {
  bool list_rs232;
  unsigned short max_node_id;
  try {
    // define available options
    bpo::options_description options;
    bool show_help;
    options.add(
        boost::make_shared< bpo::option_description >("help", bpo::bool_switch(&show_help)));
    options.add(
        boost::make_shared< bpo::option_description >("rs232", bpo::bool_switch(&list_rs232)));
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
    std::cerr << error.what() << std::endl;
    return 1;
  }

  std::cout << "Listing Nodes:" << std::endl;

  const std::vector< std::string > device_names(
      barriar< std::string >(boost::bind(eh::getDeviceNameList)));
  BOOST_FOREACH (const std::string &device_name, device_names) {
    std::cout << device_name << std::endl;

    const std::vector< std::string > protocol_stack_names(
        barriar< std::string >(boost::bind(eh::getProtocolStackNameList, device_name)));
    BOOST_FOREACH (const std::string &protocol_stack_name, protocol_stack_names) {
      std::cout << "\t" << protocol_stack_name << std::endl;

      const std::vector< std::string > interface_names(barriar< std::string >(
          boost::bind(eh::getInterfaceNameList, device_name, protocol_stack_name)));
      BOOST_FOREACH (const std::string &interface_name, interface_names) {
        std::cout << "\t\t" << interface_name << std::endl;
        if (!list_rs232 && interface_name == "RS232") {
          std::cout << "\t\t\tSkipping RS232" << std::endl;
          continue;
        }

        const std::vector< std::string > port_names(barriar< std::string >(
            boost::bind(eh::getPortNameList, device_name, protocol_stack_name, interface_name)));
        BOOST_FOREACH (const std::string &port_name, port_names) {
          std::cout << "\t\t\t" << port_name << std::endl;

          const std::vector< unsigned int > baudrates(barriar< unsigned int >(boost::bind(
              eh::getBaudrateList, device_name, protocol_stack_name, interface_name, port_name)));
          std::cout << "\t\t\t\tBaudrates:" << std::endl;
          BOOST_FOREACH (unsigned int baudrate, baudrates) {
            std::cout << "\t\t\t\t\t" << std::dec << baudrate << std::endl;
          }

          const std::vector< eh::NodeInfo > nodes(barriar< eh::NodeInfo >(boost::bind(
              eh::enumerateNodes,
              eh::DeviceInfo(device_name, protocol_stack_name, interface_name, port_name),
              0 /* no specific node id*/, max_node_id)));
          std::cout << "\t\t\t\tNodes (up to Node Id " << max_node_id << "):" << std::endl;
          BOOST_FOREACH (const eh::NodeInfo &node, nodes) {
            std::cout << "\t\t\t\t\tNode Id: " << std::dec << node.node_id << std::endl;
            std::cout << "\t\t\t\t\t\tSerial Number: 0x" << std::hex << node.serial_number
                      << std::endl;
            std::cout << "\t\t\t\t\t\tHardware Version: 0x" << std::hex << node.hardware_version
                      << std::endl;
            std::cout << "\t\t\t\t\t\tSoftware Version: 0x" << std::hex << node.software_version
                      << std::endl;
            std::cout << "\t\t\t\t\t\tApplication Number: 0x" << std::hex << node.application_number
                      << std::endl;
            std::cout << "\t\t\t\t\t\tApplication Version: 0x" << std::hex
                      << node.application_version << std::endl;
          }
        }
      }
    }
  }

  return 0;
}
