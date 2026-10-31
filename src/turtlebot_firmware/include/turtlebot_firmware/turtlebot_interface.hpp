#ifndef TURTLEBOT_INTERFACE_HPP
#define TURTLEBOT_INTERFACE_HPP

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <libserial/SerialPort.h>

namespace turtlebot_firmware
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class TurtlebotInterface : public hardware_interface::SystemInterface
{
public:
  TurtlebotInterface();
  virtual ~TurtlebotInterface();

  CallbackReturn on_init(const hardware_interface::HardwareInfo & hw_info) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & current_time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & current_time, const rclcpp::Duration & period) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

private:
  std::vector<double> cmd_vel_;
  std::vector<double> state_pos_;
  std::vector<double> state_vel_;
  
  LibSerial::SerialPort mcu_serial_;
  std::string serial_port_name_;
  rclcpp::Time prev_time_;
};

}

#endif
