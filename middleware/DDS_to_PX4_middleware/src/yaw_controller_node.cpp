#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono_literals;
using std::placeholders::_1;

class YawControllerNode : public rclcpp::Node {
public:
    YawControllerNode() : Node("yaw_controller_node") {
        rmw_qos_profile_t qos_profile = rmw_get_qos_profile_services_default();
        qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

        publisher_offboard_control_mode_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        publisher_trajectory_setpoint_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        publisher_vehicle_command_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        subscription_ = this->create_subscription<std_msgs::msg::Float64>(
            "/yaw_command", 10, std::bind(&YawControllerNode::yaw_command_callback, this, _1));

        offboard_setpoint_counter_ = 0;

        timer_ = this->create_wall_timer(
            100ms, std::bind(&YawControllerNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "YawControllerNode has been started.");
    }

    void arm();
    void disarm();

private:
    void yaw_command_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void timer_callback();

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr publisher_offboard_control_mode_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr publisher_trajectory_setpoint_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr publisher_vehicle_command_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::atomic<uint64_t> offboard_setpoint_counter_;
    double yaw_ = 0.0;
};

void YawControllerNode::yaw_command_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    yaw_ = msg->data;
}

void YawControllerNode::timer_callback() {
    if (offboard_setpoint_counter_ == 10) {
        this->arm();
    }

    if (offboard_setpoint_counter_ < 11) {
        px4_msgs::msg::OffboardControlMode msg;
        msg.position = false;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = true;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        publisher_offboard_control_mode_->publish(msg);
    }

    px4_msgs::msg::TrajectorySetpoint msg;
    msg.position = {0.0, 0.0, 0.0};
    msg.velocity = {0.0, 0.0, 0.0};
    msg.acceleration = {0.0, 0.0, 0.0};
    msg.yaw = yaw_;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    publisher_trajectory_setpoint_->publish(msg);

    offboard_setpoint_counter_++;
}

void YawControllerNode::arm() {
    px4_msgs::msg::VehicleCommand msg;
    msg.param1 = 1.0;
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    publisher_vehicle_command_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void YawControllerNode::disarm() {
    px4_msgs::msg::VehicleCommand msg;
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    publisher_vehicle_command_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YawControllerNode>());
    rclcpp::shutdown();
    return 0;
}