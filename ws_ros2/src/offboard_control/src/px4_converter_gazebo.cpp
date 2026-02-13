#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class PX4ConverterGazebo : public rclcpp::Node {
public:
    PX4ConverterGazebo() : Node("px4_converter_gazebo") {
        // Parameters
        this->declare_parameter("auto_arm", true);
        this->declare_parameter("publish_rate", 10.0);
        this->declare_parameter("safety_timeout", 5.0);
        
        auto_arm_ = this->get_parameter("auto_arm").as_bool();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        safety_timeout_ = this->get_parameter("safety_timeout").as_double();
        
        // Setup QoS profile for PX4 compatibility
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        
        // Publishers (Real PX4 topics for Gazebo)
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", qos);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", qos);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", qos);
        
        // Status publisher for monitoring
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/px4_converter_status", 10);
        
        // Subscriber for demo yaw commands
        yaw_command_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/yaw_command", 10, 
            std::bind(&PX4ConverterGazebo::yaw_command_callback, this, std::placeholders::_1));
        
        // Subscriber for PX4 vehicle control mode (feedback - more reliable than vehicle_status)
        vehicle_control_mode_sub_ = this->create_subscription<px4_msgs::msg::VehicleControlMode>(
            "/fmu/out/vehicle_control_mode", qos,
            std::bind(&PX4ConverterGazebo::vehicle_control_mode_callback, this, std::placeholders::_1));
        
        // Main control timer
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
        control_timer_ = this->create_wall_timer(timer_period, 
            std::bind(&PX4ConverterGazebo::control_timer_callback, this));
        
        // Safety timer
        safety_timer_ = this->create_wall_timer(1000ms, 
            std::bind(&PX4ConverterGazebo::safety_timer_callback, this));
        
        start_time_ = this->now();
        last_command_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "🚁 PX4 Converter for Gazebo started");
        RCLCPP_INFO(this->get_logger(), "⚙️  Publishing at %.1f Hz, auto_arm: %s, safety_timeout: %.1fs",
                    publish_rate, auto_arm_ ? "true" : "false", safety_timeout_);
        RCLCPP_INFO(this->get_logger(), "🎯 Publishing to real PX4 topics for Gazebo integration");
    }

private:
    void yaw_command_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        // Convert demo command to PX4 format
        target_yaw_ = static_cast<float>(msg->data);
        last_command_time_ = this->now();
        command_count_++;
        
        // Normalize yaw to [-π, π] range for PX4
        while (target_yaw_ > M_PI) target_yaw_ -= 2 * M_PI;
        while (target_yaw_ < -M_PI) target_yaw_ += 2 * M_PI;
        
        double degrees = target_yaw_ * 180.0 / M_PI;
        RCLCPP_INFO(this->get_logger(), 
            "🎯 Converted yaw command %d: %.3f rad (%.1f°) → PX4 Gazebo",
            command_count_, target_yaw_, degrees);
        
        // Update status
        has_received_command_ = true;
    }
    
    void vehicle_control_mode_callback(const px4_msgs::msg::VehicleControlMode::SharedPtr msg) {
        vehicle_armed_ = msg->flag_armed;
        offboard_mode_ = msg->flag_control_offboard_enabled;
        
        static int status_log_counter = 0;
        if (++status_log_counter % 50 == 0) { // Log every ~1 second
            RCLCPP_INFO(this->get_logger(), 
                "📡 Vehicle Mode: Armed=%s, Offboard=%s",
                vehicle_armed_ ? "YES" : "NO", 
                offboard_mode_ ? "YES" : "NO");
        }
    }
    
    void control_timer_callback() {
        auto current_time = this->now();
        uint64_t timestamp = current_time.nanoseconds() / 1000;
        
        // Always publish offboard control mode when we have commands
        if (has_received_command_) {
            publish_offboard_control_mode(timestamp);
            publish_trajectory_setpoint(timestamp);
        }
        
        // Handle arming sequence: PX4 requires setpoint streaming -> offboard -> arm
        // Send commands periodically until armed (handles DDS latency and PX4 readiness)
        if (auto_arm_ && has_received_command_) {
            if (!vehicle_armed_ && control_counter_ > 10 && control_counter_ % 10 == 0) {
                // Send offboard mode command first
                send_offboard_command(timestamp);
                // Then send arm command (PX4 needs offboard active to accept arm in offboard)
                send_arm_command(timestamp);
                RCLCPP_INFO(this->get_logger(), 
                    "🔄 Arming attempt (counter=%d, armed=%s, offboard=%s)",
                    control_counter_, vehicle_armed_ ? "YES" : "NO", 
                    offboard_mode_ ? "YES" : "NO");
            }
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
            
            // Stop publishing setpoints on timeout
            has_received_command_ = false;
        }
    }
    
    void publish_offboard_control_mode(uint64_t timestamp) {
        px4_msgs::msg::OffboardControlMode msg;
        msg.position = true;   // Position hold + yawspeed control
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = timestamp;
        
        offboard_control_mode_pub_->publish(msg);
    }
    
    void publish_trajectory_setpoint(uint64_t timestamp) {
        px4_msgs::msg::TrajectorySetpoint msg;
        
        // Maintain hover position (NED coordinates)
        msg.position = {0.0f, 0.0f, -5.0f}; // 5m altitude in NED
        msg.velocity = {0.0f, 0.0f, 0.0f};
        msg.acceleration = {0.0f, 0.0f, 0.0f};
        
        // Apply yaw command directly as absolute angle
        // tracking_node outputs angular error which oscillates ±, producing side-to-side yaw
        msg.yaw = target_yaw_;
        msg.yawspeed = 0.0f;
        msg.timestamp = timestamp;
        
        trajectory_setpoint_pub_->publish(msg);
        setpoint_count_++;
        
        RCLCPP_DEBUG(this->get_logger(), 
            "📡 Published trajectory setpoint %d: pos=[%.1f,%.1f,%.1f], yaw=%.3f",
            setpoint_count_, msg.position[0], msg.position[1], msg.position[2], msg.yaw);
    }
    
    void send_arm_command(uint64_t timestamp) {
        px4_msgs::msg::VehicleCommand msg;
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        msg.param1 = 1.0f; // 1 to arm, 0 to disarm
        msg.param2 = 0.0f; // Standard arm (PX4 params handle SITL failsafe exceptions)
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = timestamp;
        
        vehicle_command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "🔒 ARM command sent to PX4 Gazebo");
    }
    
    void send_offboard_command(uint64_t timestamp) {
        px4_msgs::msg::VehicleCommand msg;
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        msg.param1 = 1.0f; // main mode
        msg.param2 = 6.0f; // offboard mode
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = timestamp;
        
        vehicle_command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "🚁 OFFBOARD mode command sent to PX4 Gazebo");
    }
    
    void publish_status_update() {
        auto current_time = this->now();
        double elapsed_time = (current_time - start_time_).seconds();
        
        auto status_msg = std_msgs::msg::String();
        status_msg.data = "PX4 Gazebo Status - Commands: " + std::to_string(command_count_) + 
                         ", Setpoints: " + std::to_string(setpoint_count_) + 
                         ", Armed: " + (vehicle_armed_ ? "YES" : "NO") +
                         ", Offboard: " + (offboard_mode_ ? "YES" : "NO") +
                         ", Yaw: " + std::to_string(target_yaw_) + " rad" +
                         ", Runtime: " + std::to_string(elapsed_time) + "s";
        status_pub_->publish(status_msg);
    }

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_command_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr vehicle_control_mode_sub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr safety_timer_;
    
    // State variables
    float target_yaw_ = 0.0f;
    bool auto_arm_ = true;
    double safety_timeout_ = 5.0;
    bool has_received_command_ = false;
    bool vehicle_armed_ = false;
    bool offboard_mode_ = false;
    
    // Counters and timing
    int control_counter_ = 0;
    int command_count_ = 0;
    int setpoint_count_ = 0;
    rclcpp::Time start_time_;
    rclcpp::Time last_command_time_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PX4ConverterGazebo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}