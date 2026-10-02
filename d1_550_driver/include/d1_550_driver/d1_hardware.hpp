#ifndef D1_550_DRIVER__D1_HARDWARE_HPP_
#define D1_550_DRIVER__D1_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace d1_550_driver
{

class D1Hardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(D1Hardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Get the logger of the SystemInterface
  //rclcpp::Logger get_logger() const { return *logger_; }

  // Get the logger of the SystemInterface
  //rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

private:
  // Parameters for the RRBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  double hw_slowdown_;

  // Objects for logging 
  //std::shared_ptr<rclcpp::Logger> logger_ = rclcpp::get_logger("d1_hardware");
  rclcpp::Clock::SharedPtr clock_;

  // Store the command for the simulated robot
  std::vector<double> hw_states_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  rclcpp::Node::SharedPtr hw_node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr topic_based_joint_states_subscriber_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr topic_based_joint_commands_publisher_;
  rclcpp::Node::SharedPtr node_;
  sensor_msgs::msg::JointState latest_joint_state_;
};

}  // namespace d1_550_driver
#endif  // D1_550_DRIVER__D1_HARDWARE_HPP_
