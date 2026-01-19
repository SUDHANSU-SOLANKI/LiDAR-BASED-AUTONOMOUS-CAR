#include "my_robot_hardware/my_robot_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <libserial/SerialPort.h>
#include <sstream>

namespace my_robot_hardware
{

CallbackReturn MyRobotHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Resize vectors based on joints in URDF
  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);

  // OPEN SERIAL PORT
  try {
    serial_port_.Open("/dev/ttyACM0"); 
    serial_port_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("MyRobotHardware"), "Failed to open serial port /dev/ttyACM0");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MyRobotHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MyRobotHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type MyRobotHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (serial_port_.IsDataAvailable()) {
    std::string response;
    serial_port_.ReadLine(response);
    
    // Parse the Arduino output: "left_rad,right_rad"
    std::stringstream ss(response);
    char comma;
    if (ss >> hw_positions_[0] >> comma >> hw_positions_[1]) {
        // Successfully parsed
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyRobotHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  std::string msg = "V " + std::to_string(hw_commands_[0]) + " " + std::to_string(hw_commands_[1]) + "\n";
  
  // ADD THIS LINE TO DEBUG IN YOUR TERMINAL
  RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Sending: %s", msg.c_str());
  RCLCPP_INFO(rclcpp::get_logger("MyRobotHardware"), "Current Pos: L:%f, R:%f", hw_positions_[0], hw_positions_[1]);

  serial_port_.Write(msg);
  return hardware_interface::return_type::OK;
}

} // namespace

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(my_robot_hardware::MyRobotHardware, hardware_interface::SystemInterface)