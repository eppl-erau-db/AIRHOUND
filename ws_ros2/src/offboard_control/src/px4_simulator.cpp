
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <chrono>

using namespace std::chrono_literals;

class Px4Simulator : public rclcpp::Node {
public:
    Px4Simulator() : Node("px4_simulator") {
        rmw_qos_profile_t qos_profile = rmw_get_qos_profile_services_default();
        qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;

        subscription_offboard_control_mode_ = this->create_subscription<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10, std::bind(&Px4Simulator::offboard_control_mode_callback, this, std::placeholders::_1));
        subscription_trajectory_setpoint_ = this->create_subscription<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10, std::bind(&Px4Simulator::trajectory_setpoint_callback, this, std::placeholders::_1));

        publisher_vehicle_status_ = this->create_publisher<px4_msgs::msg::VehicleStatus>("/fmu/out/vehicle_status", 10);
        publisher_vehicle_odometry_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("/fmu/out/vehicle_odometry", 10);

        timer_ = this->create_wall_timer(
            100ms, std::bind(&Px4Simulator::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Px4Simulator has been started.");
    }

private:
    void offboard_control_mode_callback(const px4_msgs::msg::OffboardControlMode::SharedPtr msg) {
        // Not used in this simple simulator
    }

    void trajectory_setpoint_callback(const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
        yaw_ = msg->yaw;
    }

    void timer_callback() {
        px4_msgs::msg::VehicleStatus status_msg;
        status_msg.arming_state = 2; // Armed
        status_msg.nav_state = 14; // Offboard
        status_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        publisher_vehicle_status_->publish(status_msg);

        px4_msgs::msg::VehicleOdometry odom_msg;
        odom_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        odom_msg.q[0] = cos(yaw_ / 2.0);
        odom_msg.q[1] = 0.0;
        odom_msg.q[2] = 0.0;
        odom_msg.q[3] = sin(yaw_ / 2.0);
        publisher_vehicle_odometry_->publish(odom_msg);
    }

    rclcpp::Subscription<px4_msgs::msg::OffboardControlMode>::SharedPtr subscription_offboard_control_mode_;
    rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr subscription_trajectory_setpoint_;
    rclcpp::Publisher<px4_msgs::msg::VehicleStatus>::SharedPtr publisher_vehicle_status_;
    rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr publisher_vehicle_odometry_;
    rclcpp::TimerBase::SharedPtr timer_;

    double yaw_ = 0.0;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Px4Simulator>());
    rclcpp::shutdown();
    return 0;
}
