#include "turtlebot_firmware/turtlebot_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace turtlebot_firmware
{

TurtlebotInterface::TurtlebotInterface() {}

TurtlebotInterface::~TurtlebotInterface() {}

CallbackReturn TurtlebotInterface::on_init(const hardware_interface::HardwareInfo & hw_info)
{
  auto init_res = hardware_interface::SystemInterface::on_init(hw_info);
  if (init_res != CallbackReturn::SUCCESS) {
    return init_res;
  }

  auto j_size = info_.joints.size();
  cmd_vel_.reserve(j_size);
  state_pos_.reserve(j_size);
  state_vel_.reserve(j_size);

  return CallbackReturn::SUCCESS;
}

CallbackReturn TurtlebotInterface::on_activate(const rclcpp_lifecycle::State &)
{
  cmd_vel_.assign(2, 0.0);
  state_pos_.assign(2, 0.0);
  state_vel_.assign(2, 0.0);

  return CallbackReturn::SUCCESS;
}

CallbackReturn TurtlebotInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type TurtlebotInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TurtlebotInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> TurtlebotInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> states;
  for (size_t idx = 0; idx < info_.joints.size(); ++idx) {
    states.emplace_back(hardware_interface::StateInterface(
      info_.joints[idx].name, hardware_interface::HW_IF_POSITION, &state_pos_[idx]));
    states.emplace_back(hardware_interface::StateInterface(
      info_.joints[idx].name, hardware_interface::HW_IF_VELOCITY, &state_vel_[idx]));
  }
  return states;
}

std::vector<hardware_interface::CommandInterface> TurtlebotInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> commands;
  for (size_t idx = 0; idx < info_.joints.size(); ++idx) {
    commands.emplace_back(hardware_interface::CommandInterface(
      info_.joints[idx].name, hardware_interface::HW_IF_VELOCITY, &cmd_vel_[idx]));
  }
  return commands;
}

}

PLUGINLIB_EXPORT_CLASS(turtlebot_firmware::TurtlebotInterface, hardware_interface::SystemInterface)