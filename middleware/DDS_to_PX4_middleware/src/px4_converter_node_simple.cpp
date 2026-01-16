#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class PX4ConverterNodeSimple : public rclcpp::Node {
public:
    PX4ConverterNodeSimple() : Node("px4_converter_node") {
        // Parameters
        this->declare_parameter("auto_arm", true);
        this->declare_parameter("publish_rate", 10.0);
        this->declare_parameter("safety_timeout", 5.0);
        // Note: use_sim_time is automatically declared by ROS2
        
        auto_arm_ = this->get_parameter("auto_arm").as_bool();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        safety_timeout_ = this->get_parameter("safety_timeout").as_double();
        
        // Publishers (using standard ROS messages as proxies)
        setpoint_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/px4_setpoint", 10);
        command_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/px4_command", 10);
        
        // Status publisher for monitoring
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/px4_converter_status", 10);
        
        // Subscriber for demo yaw commands
        yaw_command_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/yaw_command", 10, 
            std::bind(&PX4ConverterNodeSimple::yaw_command_callback, this, std::placeholders::_1));
        
        // Main control timer
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
        control_timer_ = this->create_wall_timer(timer_period, 
            std::bind(&PX4ConverterNodeSimple::control_timer_callback, this));
        
        // Safety timer
        safety_timer_ = this->create_wall_timer(1000ms, 
            std::bind(&PX4ConverterNodeSimple::safety_timer_callback, this));
        
        start_time_ = this->now();
        last_command_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "🎯 PX4 Converter Node (Simple) started");
        RCLCPP_INFO(this->get_logger(), "⚙️  Publishing at %.1f Hz, auto_arm: %s, safety_timeout: %.1fs",
                    publish_rate, auto_arm_ ? "true" : "false", safety_timeout_);
        RCLCPP_INFO(this->get_logger(), "📢 Note: Using standard ROS messages (px4_msgs not available)");
    }

private:
    void yaw_command_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        // Convert demo command to normalized yaw
        target_yaw_ = static_cast<float>(msg->data);
        last_command_time_ = this->now();
        command_count_++;
        
        // Normalize yaw to [-π, π] range for PX4
        while (target_yaw_ > M_PI) target_yaw_ -= 2 * M_PI;
        while (target_yaw_ < -M_PI) target_yaw_ += 2 * M_PI;
        
        double degrees = target_yaw_ * 180.0 / M_PI;
        RCLCPP_INFO(this->get_logger(), 
            "🎯 Converted yaw command %d: %.3f rad (%.1f°) → PX4 setpoint",
            command_count_, target_yaw_, degrees);
        
        // Update status
        has_received_command_ = true;
    }
    
    void control_timer_callback() {
        auto current_time = this->now();
        
        // Publish converted setpoint (using PoseStamped as proxy for trajectory setpoint)
        publish_converted_setpoint();
        
        // Handle arming sequence (simplified)
        if (auto_arm_ && control_counter_ == 10) {
            send_arm_command();
        } else if (auto_arm_ && control_counter_ == 50) {
            send_offboard_command();
        }
        
        control_counter_++;
        
        // Publish status
        if (control_counter_ % 10 == 0) { // Every second at 10Hz
            publish_status_update();
        }
    }
    
    void safety_timer_callback() {
        auto current_time = this->now();
        double time_since_command = (current_time - last_command_time_).seconds();
        
        if (has_received_command_ && time_since_command > safety_timeout_) {
            RCLCPP_WARN(this->get_logger(), 
                "⚠️  Safety timeout: No yaw commands for %.1fs (timeout: %.1fs)",
                time_since_command, safety_timeout_);
        }
    }
    
    void publish_converted_setpoint() {
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        
        // Maintain hover position (NED coordinates simulated)
        msg.pose.position.x = 0.0;  // North
        msg.pose.position.y = 0.0;  // East  
        msg.pose.position.z = -5.0; // Down (5m altitude in NED)
        
        // Convert yaw to quaternion (simplified - only yaw rotation)
        double half_yaw = target_yaw_ / 2.0;
        msg.pose.orientation.x = 0.0;
        msg.pose.orientation.y = 0.0;
        msg.pose.orientation.z = sin(half_yaw);
        msg.pose.orientation.w = cos(half_yaw);
        
        setpoint_pub_->publish(msg);
        setpoint_count_++;
        
        RCLCPP_DEBUG(this->get_logger(), 
            "📡 Published setpoint %d: yaw=%.3f rad, qz=%.3f, qw=%.3f",
            setpoint_count_, target_yaw_, msg.pose.orientation.z, msg.pose.orientation.w);
    }
    
    void send_arm_command() {
        auto msg = std_msgs::msg::String();
        msg.data = "ARM_VEHICLE";
        command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "🔒 ARM command sent (simulated)");
    }
    
    void send_offboard_command() {
        auto msg = std_msgs::msg::String();
        msg.data = "SET_OFFBOARD_MODE";
        command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "🚁 OFFBOARD mode command sent (simulated)");
    }
    
    void publish_status_update() {
        auto current_time = this->now();
        double elapsed_time = (current_time - start_time_).seconds();
        
        auto status_msg = std_msgs::msg::String();
        status_msg.data = "PX4 Converter Status - Commands: " + std::to_string(command_count_) + 
                         ", Setpoints: " + std::to_string(setpoint_count_) + 
                         ", Current yaw: " + std::to_string(target_yaw_) + " rad" +
                         ", Runtime: " + std::to_string(elapsed_time) + "s";
        status_pub_->publish(status_msg);
        
        RCLCPP_DEBUG(this->get_logger(), 
            "📊 Status: %d commands, %d setpoints, %.1fs runtime",
            command_count_, setpoint_count_, elapsed_time);
    }

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr setpoint_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_command_sub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr safety_timer_;
    
    // State variables
    float target_yaw_ = 0.0f;
    bool auto_arm_ = true;
    double safety_timeout_ = 5.0;
    bool has_received_command_ = false;
    
    // Counters and timing
    int control_counter_ = 0;
    int command_count_ = 0;
    int setpoint_count_ = 0;
    rclcpp::Time start_time_;
    rclcpp::Time last_command_time_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PX4ConverterNodeSimple>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}