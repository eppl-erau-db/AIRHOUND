#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

// PX4 message types (will work with px4_msgs package)
#ifdef PX4_MSGS_AVAILABLE
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#else
// Mock PX4 messages for testing without px4_msgs package
namespace px4_msgs { namespace msg {
    struct OffboardControlMode {
        bool position = false;
        bool velocity = false;
        bool acceleration = false;
        bool attitude = false;
        bool body_rate = false;
        uint64_t timestamp = 0;
    };
    struct TrajectorySetpoint {
        std::array<float, 3> position = {0.0, 0.0, 0.0};
        std::array<float, 3> velocity = {0.0, 0.0, 0.0};
        std::array<float, 3> acceleration = {0.0, 0.0, 0.0};
        float yaw = 0.0;
        float yawspeed = 0.0;
        uint64_t timestamp = 0;
    };
    struct VehicleCommand {
        static constexpr uint16_t VEHICLE_CMD_COMPONENT_ARM_DISARM = 400;
        static constexpr uint16_t VEHICLE_CMD_DO_SET_MODE = 176;
        uint16_t command = 0;
        float param1 = 0.0;
        float param2 = 0.0;
        uint8_t target_system = 1;
        uint8_t target_component = 1;
        uint8_t source_system = 1;
        uint8_t source_component = 1;
        bool from_external = true;
        uint64_t timestamp = 0;
    };
    struct VehicleStatus {
        uint8_t arming_state = 0;
        uint8_t nav_state = 0;
        bool failsafe = false;
        uint64_t timestamp = 0;
    };
}}
#endif

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class PX4ConverterNode : public rclcpp::Node {
public:
    PX4ConverterNode() : Node("px4_converter_node") {
        // Parameters
        this->declare_parameter("auto_arm", true);
        this->declare_parameter("publish_rate", 10.0);
        this->declare_parameter("safety_timeout", 5.0);
        // Note: use_sim_time is automatically declared by ROS2
        
        auto_arm_ = this->get_parameter("auto_arm").as_bool();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        safety_timeout_ = this->get_parameter("safety_timeout").as_double();
        
        // Setup QoS profile for PX4 compatibility
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        
        // Publishers (PX4 topics)
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", qos);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", qos);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", qos);
        
        // Status publisher for monitoring
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/px4_converter_status", 10);
        
        // Subscriber for demo yaw commands
        yaw_command_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/target_yaw", 10, 
            std::bind(&PX4ConverterNode::yaw_command_callback, this, std::placeholders::_1));
        
        // Main control timer
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
        control_timer_ = this->create_wall_timer(timer_period, 
            std::bind(&PX4ConverterNode::control_timer_callback, this));
        
        // Safety timer
        safety_timer_ = this->create_wall_timer(1000ms, 
            std::bind(&PX4ConverterNode::safety_timer_callback, this));
        
        start_time_ = this->now();
        last_command_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "🎯 PX4 Converter Node started");
        RCLCPP_INFO(this->get_logger(), "⚙️  Publishing at %.1f Hz, auto_arm: %s, safety_timeout: %.1fs",
                    publish_rate, auto_arm_ ? "true" : "false", safety_timeout_);
    }

private:
    void yaw_command_callback(const std_msgs::msg::Float32::SharedPtr msg) {
        // Receive yaw rate command from tracking node
        target_yaw_ = static_cast<float>(msg->data);
        last_command_time_ = this->now();
        command_count_++;
        
        // Clamp yaw rate to reasonable limits (±1 rad/s typical)
        if (target_yaw_ > 1.5f) target_yaw_ = 1.5f;
        if (target_yaw_ < -1.5f) target_yaw_ = -1.5f;
        
        double degrees_per_sec = target_yaw_ * 180.0 / M_PI;
        RCLCPP_INFO(this->get_logger(), 
            "🎯 Received yaw rate cmd %d: %.3f rad/s (%.1f°/s) → PX4 yawspeed",
            command_count_, target_yaw_, degrees_per_sec);
        
        // Update status
        has_received_command_ = true;
    }
    
    void control_timer_callback() {
        auto current_time = this->now();
        uint64_t timestamp = current_time.nanoseconds() / 1000;
        
        // Publish offboard control mode
        publish_offboard_control_mode(timestamp);
        
        // Publish trajectory setpoint with converted yaw
        publish_trajectory_setpoint(timestamp);
        
        // Handle arming sequence: PX4 requires setpoint streaming -> offboard -> arm
        if (auto_arm_ && control_counter_ == 20) {
            send_offboard_command(timestamp);  // Switch to offboard after ~2s of setpoints
        } else if (auto_arm_ && control_counter_ == 30) {
            send_arm_command(timestamp);        // Arm after offboard mode is active
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
            // Could implement emergency landing or failsafe here
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
        
        // Maintain current position (hover)
        msg.position = {0.0f, 0.0f, -5.0f}; // -5m altitude (NED frame)
        msg.velocity = {0.0f, 0.0f, 0.0f};
        msg.acceleration = {0.0f, 0.0f, 0.0f};
        
        // Apply yaw rate command from tracking node
        // tracking.py publishes yaw error/rate (proportional to pixel offset)
        // Use yawspeed for rate control, set yaw to NaN to ignore absolute angle
        msg.yaw = std::nanf(""); // NaN = don't control absolute yaw
        msg.yawspeed = target_yaw_; // Use as yaw rate command (rad/s)
        msg.timestamp = timestamp;
        
        trajectory_setpoint_pub_->publish(msg);
        
        setpoint_count_++;
    }
    
    void send_arm_command(uint64_t timestamp) {
        px4_msgs::msg::VehicleCommand msg;
        msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        msg.param1 = 1.0f; // 1 to arm, 0 to disarm
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        msg.timestamp = timestamp;
        
        vehicle_command_pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "🔒 ARM command sent to PX4");
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
        RCLCPP_INFO(this->get_logger(), "🚁 OFFBOARD mode command sent to PX4");
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
    }

    // Publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr yaw_command_sub_;
    
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
    auto node = std::make_shared<PX4ConverterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}