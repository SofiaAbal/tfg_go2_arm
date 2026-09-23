#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include "msg/ArmString_.hpp"
#include "msg/PubServoInfo_.hpp"

#include <sstream>
#include <cmath>

#include <memory>
#include <vector>
#include <string>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>

using namespace unitree::robot;
using namespace unitree::common;

class UnitreeD1Controller : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

    UnitreeD1Controller() : Node("d1_controller")
    {
        // Initialize Unitree channels
        ChannelFactory::Instance()->Init(0);

        // Initialize publishers and subscribers
        arm_command_pub_ = std::make_unique<ChannelPublisher<unitree_arm::msg::dds_::ArmString_>>("rt/arm_Command");
        arm_command_pub_->InitChannel();

        servo_info_sub_ = std::make_unique<ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>>("current_servo_angle");
        servo_info_sub_->InitChannel([this](const void *msg)
                                     { this->servoInfoCallback(msg); });

        arm_feedback_sub_ = std::make_unique<ChannelSubscriber<unitree_arm::msg::dds_::ArmString_>>("arm_Feedback");
        arm_feedback_sub_->InitChannel([this](const void *msg)
                                       { this->armFeedbackCallback(msg); });

        // Initialize ROS2 components
        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("d1/joint_states", 10);

        load_enable_service_ = this->create_service<std_srvs::srv::SetBool>(
            "d1/enable_load",
            std::bind(&UnitreeD1Controller::enableLoadCallback, this, std::placeholders::_1, std::placeholders::_2));

        trajectory_action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            this,
            "d1/follow_joint_trajectory",
            std::bind(&UnitreeD1Controller::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&UnitreeD1Controller::handleCancel, this, std::placeholders::_1),
            std::bind(&UnitreeD1Controller::handleAccepted, this, std::placeholders::_1));

        // Initialize joint names
        joint_names_ = {
            "d1_link1_joint", "d1_link2_joint", "d1_link3_joint",
            "d1_link4_joint", "d1_link5_joint", "d1_link6_joint",
            "d1_link_l_joint", "d1_link_r_joint"};

        // Initialize joint positions
        current_joint_positions_.resize(8, 0.0);

        // Start publishing timer
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&UnitreeD1Controller::publishJointStates, this));

        RCLCPP_INFO(this->get_logger(), "D1 Controller initialized");
    }

private:
    // Unitree communication
    std::unique_ptr<ChannelPublisher<unitree_arm::msg::dds_::ArmString_>> arm_command_pub_;
    std::unique_ptr<ChannelSubscriber<unitree_arm::msg::dds_::PubServoInfo_>> servo_info_sub_;
    std::unique_ptr<ChannelSubscriber<unitree_arm::msg::dds_::ArmString_>> arm_feedback_sub_;

    // ROS2 components
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr load_enable_service_;
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr trajectory_action_server_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    // Joint state data
    std::vector<std::string> joint_names_;
    std::vector<double> current_joint_positions_;
    std::mutex joint_state_mutex_;
    std::atomic<bool> trajectory_executing_{false};

    void servoInfoCallback(const void *msg)
    {
        const auto *servo_msg = static_cast<const unitree_arm::msg::dds_::PubServoInfo_ *>(msg);

        std::lock_guard<std::mutex> lock(joint_state_mutex_);

        // Convert servo data to joint positions (degrees to radians)
        current_joint_positions_[0] = servo_msg->servo0_data_() * M_PI / 180.0;
        current_joint_positions_[1] = servo_msg->servo1_data_() * M_PI / 180.0;
        current_joint_positions_[2] = servo_msg->servo2_data_() * M_PI / 180.0;
        current_joint_positions_[3] = servo_msg->servo3_data_() * M_PI / 180.0;
        current_joint_positions_[4] = servo_msg->servo4_data_() * M_PI / 180.0;
        current_joint_positions_[5] = servo_msg->servo5_data_() * M_PI / 180.0;

        // Grippers
        current_joint_positions_[6] = servo_msg->servo6_data_();
        current_joint_positions_[7] = servo_msg->servo6_data_();
    }

    void armFeedbackCallback(const void *msg)
    {
        const auto *feedback_msg = static_cast<const unitree_arm::msg::dds_::ArmString_ *>(msg);
        RCLCPP_DEBUG(this->get_logger(), "Arm feedback: %s", feedback_msg->data_().c_str());
    }

    void publishJointStates()
    {
        auto joint_state = sensor_msgs::msg::JointState();
        joint_state.header.stamp = this->now();
        joint_state.header.frame_id = "";

        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        joint_state.name = joint_names_;
        joint_state.position = current_joint_positions_;

        double gripper = linearMap(joint_state.position[6], -10.0, 10.0, 0.03, 0.0);
        joint_state.position[6] = gripper;
        joint_state.position[7] = gripper;

        joint_state.velocity.resize(8, 0.0);
        joint_state.effort.resize(8, 0.0);

        joint_state_pub_->publish(joint_state);
    }

    void enableLoadCallback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        unitree_arm::msg::dds_::ArmString_ msg;

        if (request->data)
        {
            // Enable load (mode 1)
            msg.data_() = "{\"seq\":4,\"address\":1,\"funcode\":5,\"data\":{\"mode\":1}}";
            response->message = "Load enabled";
        }
        else
        {
            // Disable load (mode 0)
            msg.data_() = "{\"seq\":4,\"address\":1,\"funcode\":5,\"data\":{\"mode\":0}}";
            response->message = "Load disabled";
        }

        arm_command_pub_->Write(msg);
        response->success = true;

        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }

    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received trajectory goal with %zu points",
                    goal->trajectory.points.size());

        // Basic validation
        if (goal->trajectory.points.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Empty trajectory received");
            return rclcpp_action::GoalResponse::REJECT;
        }

        if (trajectory_executing_.load())
        {
            RCLCPP_WARN(this->get_logger(), "Another trajectory is already executing");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleFJT> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel trajectory");
        trajectory_executing_ = false;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandleFJT> goal_handle)
    {
        std::thread{std::bind(&UnitreeD1Controller::executeTrajectory, this, goal_handle)}.detach();
    }

    void executeTrajectory(const std::shared_ptr<GoalHandleFJT> goal_handle)
    {
        trajectory_executing_ = true;
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
        auto result = std::make_shared<FollowJointTrajectory::Result>();

        RCLCPP_INFO(this->get_logger(), "Executing trajectory with %zu points",
                    goal->trajectory.points.size());

        try
        {
            for (size_t i = 0; i < goal->trajectory.points.size() && trajectory_executing_; ++i)
            {
                const auto &point = goal->trajectory.points[i];

                // Validate positions
                if (point.positions.size() < 7)
                {
                    RCLCPP_ERROR(this->get_logger(), "Point %zu has insufficient positions", i);
                    result->error_code = FollowJointTrajectory::Result::INVALID_JOINTS;
                    goal_handle->abort(result);
                    trajectory_executing_ = false;
                    return;
                }

                // Send command to arm
                sendJointCommand(point.positions);

                // Update feedback
                feedback->header.stamp = this->now();
                feedback->joint_names = goal->trajectory.joint_names;
                {
                    std::lock_guard<std::mutex> lock(joint_state_mutex_);
                    feedback->actual.positions = current_joint_positions_;
                }
                feedback->desired.positions = point.positions;

                goal_handle->publish_feedback(feedback);

                // Wait for next point timing
                if (i < goal->trajectory.points.size() - 1)
                {
                    const auto &next = goal->trajectory.points[i + 1].time_from_start;
                    const auto &curr = point.time_from_start;

                    // Convert ROS Duration msg -> std::chrono::nanoseconds
                    auto to_ns = [](const builtin_interfaces::msg::Duration &d)
                    {
                        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                            std::chrono::seconds(d.sec) + std::chrono::nanoseconds(d.nanosec));
                    };

                    auto delta_ns = to_ns(next) - to_ns(curr);
                    if (delta_ns.count() > 0)
                    {
                        std::this_thread::sleep_for(delta_ns);
                    }
                }

                if (goal_handle->is_canceling())
                {
                    RCLCPP_INFO(this->get_logger(), "Trajectory execution canceled");
                    result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
                    goal_handle->canceled(result);
                    trajectory_executing_ = false;
                    return;
                }
            }

            // Trajectory completed successfully
            result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Trajectory execution completed successfully");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error during trajectory execution: %s", e.what());
            result->error_code = FollowJointTrajectory::Result::INVALID_GOAL;
            goal_handle->abort(result);
        }

        trajectory_executing_ = false;
    }

    inline double linearMap(double x, double in_min, double in_max, double out_min, double out_max)
    {
        // Clamp
        if (x < in_min)
            x = in_min;
        if (x > in_max)
            x = in_max;

        // Interpolation
        return out_min + ((x - in_min) * (out_max - out_min)) / (in_max - in_min);
    }

    void sendJointCommand(const std::vector<double> &joint_positions)
    {
        std::ostringstream json_stream;
        json_stream << "{\"seq\":4,\"address\":1,\"funcode\":2,\"data\":{\"mode\":1";

        for (size_t i = 0; i < std::min(joint_positions.size(), size_t(7)); ++i)
        {
            double value = 0.0;
            value = joint_positions[i] * 180.0 / M_PI;
            json_stream << ",\"angle" << i << "\":" << value;
        }

        json_stream << "}}";

        unitree_arm::msg::dds_::ArmString_ msg;
        msg.data_() = json_stream.str();
        arm_command_pub_->Write(msg);

        RCLCPP_DEBUG(this->get_logger(), "Sent joint command: %s", msg.data_().c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UnitreeD1Controller>();

    RCLCPP_INFO(node->get_logger(), "Unitree D1 Controller node started");

    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(node->get_logger(), "Exception in main: %s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}