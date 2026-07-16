// Copyright 2023 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "d1_550_driver/d1_hardware.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("D1Hardware");

namespace d1_550_driver
{

static constexpr std::size_t POSITION_INTERFACE_INDEX = 0;
static constexpr std::size_t VELOCITY_INTERFACE_INDEX = 1;
// JointState doesn't contain an acceleration field, so right now it's not used
//static constexpr std::size_t EFFORT_INTERFACE_INDEX = 3;

hardware_interface::CallbackReturn D1Hardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  clock_ = std::make_shared<rclcpp::Clock>();



  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  //hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  //hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  //hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        LOGGER, "* Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        LOGGER, "** Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      // dos porque velocidad y posicion
      RCLCPP_FATAL(
        LOGGER, "* Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        LOGGER, "** Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = 0;
    hw_commands_[i] = 0;
    //hw_commands_[i].resize(info_.joints.size(), 0.0);
    //hw_states_[i].resize(info_.joints.size(), 0.0);
  }

  //rclcpp::NodeOptions options;
  //options.arguments({ "--ros-args", "-r", "__node:=topic_based_ros2_control_" + info_.name });
  //node_ = rclcpp::Node::make_shared("_", options);
  node_ = rclcpp::Node::make_shared("d1_hardware_node");

/*   const auto get_hardware_parameter = [this](const std::string& parameter_name, const std::string& default_value) {
    if (auto it = info_.hardware_parameters.find(parameter_name); it != info_.hardware_parameters.end())
    {
      return it->second;
    }
    return default_value;
  }; */

  //topic_based_joint_commands_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
  //    get_hardware_parameter("joint_commands_topic", "/robot_joint_commands"), rclcpp::QoS(1));
  //topic_based_joint_states_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
  //    get_hardware_parameter("joint_states_topic", "/robot_joint_states"), rclcpp::SensorDataQoS(),
  //    [this](const sensor_msgs::msg::JointState::SharedPtr joint_state) { latest_joint_state_ = *joint_state; });
//

  topic_based_joint_commands_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>(
    "/arm_joint_commands", rclcpp::QoS(1));
  topic_based_joint_states_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/arm_joint_states", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr joint_state) { latest_joint_state_ = *joint_state; });
  //topic_based_joint_states_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
  //  "/joint_states", rclcpp::SensorDataQoS(),
  //  [this](const sensor_msgs::msg::JointState::SharedPtr joint_state) { latest_joint_state_ = *joint_state; });

  RCLCPP_INFO(LOGGER, "#### on_init pub = %s", topic_based_joint_commands_publisher_->get_topic_name());
  RCLCPP_INFO(LOGGER, "#### on_init sub = %s", topic_based_joint_states_subscriber_->get_topic_name());

  RCLCPP_INFO(LOGGER, "#### on_init OK — %zu joints", info_.joints.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn D1Hardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(LOGGER, "#### Configuring ...please wait...");

  /* for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(LOGGER, "%.1f seconds left...", hw_start_sec_ - i);
  } */
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // reset values always when configuring hardware
  /* for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_state(name, 0.0);
  }
  for (const auto & [name, descr] : joint_command_interfaces_)
  {
    set_command(name, 0.0);
  } */
  /* for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] = 0;
    hw_commands_[i] = 0;
  } */
  RCLCPP_INFO(LOGGER, "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
D1Hardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
D1Hardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn D1Hardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  //RCLCPP_INFO(LOGGER, "#### Activating ...please wait...");

  /* for (int i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(LOGGER, "#### %.1f seconds left...", hw_start_sec_ - i);
  } */
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  /* for (const auto & [name, descr] : joint_state_interfaces_)
  {
    set_command(name, get_state(name));
  } */
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i] = hw_states_[i];
  }

  RCLCPP_INFO(LOGGER, "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn D1Hardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  //RCLCPP_INFO(LOGGER, "#### Deactivating ...please wait...");

/*   for (int i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(LOGGER, "#### %.1f seconds left...", hw_stop_sec_ - i);
  } */

  RCLCPP_INFO(LOGGER, "#### Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type D1Hardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Reading states:";

  /* for (const auto & [name, descr] : joint_state_interfaces_)
  {
    // Simulate RRBot's movement
    auto new_value = get_state(name) + (get_command(name) - get_state(name)) / hw_slowdown_;
    set_state(name, new_value);
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << get_state(name) << " for joint '" << name << "'";
  } */
  /* for (uint i = 0; i < hw_states_.size(); i++)
  {
    // Simulate RRBot's movement
    hw_states_[i] = hw_states_[i] + (hw_commands_[i] - hw_states_[i]) / hw_slowdown_;

    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << hw_states_[i] << " for joint '" << info_.joints[i].name << "'";
  } */
  /* constexpr double kHwSlowdown = 20.0;  // más alto = más lento el "movimiento" simulado
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] += (hw_commands_[i] - hw_states_[i]) / kHwSlowdown;
  } */
  /* constexpr double kHwSlowdown = 20.0;  // más alto = más lento el "movimiento" simulado
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_states_[i] += (hw_commands_[i] - hw_states_[i]) / kHwSlowdown;
  } */
  if (rclcpp::ok())
  {
    rclcpp::spin_some(node_);
  }

  for (std::size_t i = 0; i < latest_joint_state_.name.size(); ++i)
  {
    const auto& joints = info_.joints;
    auto it = std::find_if(joints.begin(), joints.end(),
                          [&joint_name = std::as_const(latest_joint_state_.name[i])](
                              const hardware_interface::ComponentInfo& info) { return joint_name == info.name; });
    if (it != joints.end())
    {
      auto j = static_cast<std::size_t>(std::distance(joints.begin(), it));
      hw_states_[j] = latest_joint_state_.position[i];
      if (!latest_joint_state_.velocity.empty())
      {
        hw_velocities_[j] = latest_joint_state_.velocity[i];
      }
    }
  }

  for (uint i = 0; i < hw_states_.size(); i++) {
    ss << std::fixed << std::setprecision(3)
       << "\n\t" << info_.joints[i].name << ": " << hw_states_[i];
  }


  RCLCPP_INFO_THROTTLE(LOGGER, *clock_, 5000, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type D1Hardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  std::stringstream ss;
  ss << "Writing commands:";

  /* for (const auto & [name, descr] : joint_command_interfaces_)
  {
    // Simulate sending commands to the hardware
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << get_command(name) << " for joint '" << name << "'";
  } */
  /* for (uint i = 0; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    ss << std::fixed << std::setprecision(2) << std::endl
       << "\t" << hw_commands_[i] << " for joint '" << info_.joints[i].name << "'";
  } */

  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = node_->now();

  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    joint_state.name.push_back(info_.joints[i].name);
    joint_state.position.push_back(hw_commands_[i]);
    
    // only send commands to the interfaces that are defined for this joint
    /* for (std::size_t i = 0; i < info_.joints.size(); ++i)
    {
      joint_state.name.push_back(info_.joints[i].name);
      joint_state.position.push_back(hw_commands_[i]);
    } */
  }

  //topic_based_joint_commands_publisher_->publish(joint_state);

  //ultima parte
  if (rclcpp::ok())
  {
    topic_based_joint_commands_publisher_->publish(joint_state);
  }

  for (uint i = 0; i < hw_commands_.size(); i++) {
    ss << std::fixed << std::setprecision(3)
       << "\n\t *** " << info_.joints[i].name << ": " << hw_commands_[i];
  }

  RCLCPP_INFO_THROTTLE(LOGGER, *clock_, 5000, "%s", ss.str().c_str());
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace d1_550_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  d1_550_driver::D1Hardware, hardware_interface::SystemInterface)
