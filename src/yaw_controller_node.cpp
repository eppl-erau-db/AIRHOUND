#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class YawControllerNode : public rclcpp::Node {
public:
    YawControllerNode() : Node("yaw_controller_node") {
        // Declare parameters
        this->declare_parameter("hover_height", 5.0);
        this->declare_parameter("pre_stream_cycles", 50);
        this->declare_parameter("stream_rate_hz", 10.0);
        this->declare_parameter("target_timeout_sec", 1.0);
        this->declare_parameter("enable_debug", false);
        
        // Get parameters
        hover_height_ = this->get_parameter("hover_height").as_double();
        pre_stream_cycles_ = this->get_parameter("pre_stream_cycles").as_int();
        stream_rate_hz_ = this->get_parameter("stream_rate_hz").as_double();
        target_timeout_sec_ = this->get_parameter("target_timeout_sec").as_double();
        enable_debug_ = this->get_parameter("enable_debug").as_bool();
        
        // Create publishers
        offboard_control_mode_pub_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        vehicle_command_pub_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);
        
        // Create subscribers
        target_yaw_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/target_yaw", 10,
            std::bind(&YawControllerNode::targetYawCallback, this, std::placeholders::_1));
        
        vehicle_status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", 10,
            std::bind(&YawControllerNode::vehicleStatusCallback, this, std::placeholders::_1));
        
        // Create timer for streaming commands
        auto timer_period = std::chrono::duration<double>(1.0 / stream_rate_hz_);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
            std::bind(&YawControllerNode::timerCallback, this));
        
        RCLCPP_INFO(this->get_logger(), 
            "Yaw controller started: height=%.1fm, rate=%.1fHz", 
            hover_height_, stream_rate_hz_);
    }

private:
    // Callback functions
    void targetYawCallback(const std_msgs::msg::Float32::SharedPtr msg) {
        target_yaw_ = msg->data;
        last_target_time_ = this->now();
        
        if (enable_debug_) {
            RCLCPP_DEBUG(this->get_logger(), "Target yaw: %.2f rad", target_yaw_);
        }
    }
    
    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
        vehicle_status_ = *msg;
        
        // Log state changes
        if (msg->arming_state != last_arming_state_) {
            RCLCPP_INFO(this->get_logger(), "Arming state: %s",
                msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED ? 
                "ARMED" : "DISARMED");
            last_arming_state_ = msg->arming_state;
        }
        
        if (msg->nav_state != last_nav_state_) {
            RCLCPP_INFO(this->get_logger(), "Nav state changed to: %d", msg->nav_state);
            last_nav_state_ = msg->nav_state;
        }
    }
    
    void timerCallback() {
        // Always publish control mode and setpoint
        publishOffboardControlMode();
        publishTrajectorySetpoint();
        
        // Handle state machine
        switch (state_) {
            case State::INIT:
                if (++stream_count_ >= pre_stream_cycles_) {
                    RCLCPP_INFO(this->get_logger(), "Pre-streaming complete, switching to OFFBOARD");
                    sendVehicleCommand(
                        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                        1.0,  // Custom mode
                        6.0   // Offboard mode
                    );
                    state_ = State::WAIT_FOR_OFFBOARD;
                }
                break;
                
            case State::WAIT_FOR_OFFBOARD:
                if (vehicle_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                    RCLCPP_INFO(this->get_logger(), "OFFBOARD mode confirmed, arming");
                    sendVehicleCommand(
                        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
                        1.0   // Arm
                    );
                    state_ = State::WAIT_FOR_ARM;
                }
                break;
                
            case State::WAIT_FOR_ARM:
                if (vehicle_status_.arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED) {
                    RCLCPP_INFO(this->get_logger(), "Armed! Starting control");
                    state_ = State::ACTIVE;
                }
                break;
                
            case State::ACTIVE:
                // Normal operation - check for failsafes
                if (vehicle_status_.nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
                    RCLCPP_WARN(this->get_logger(), "Lost OFFBOARD mode!");
                    state_ = State::WAIT_FOR_OFFBOARD;
                }
                break;
        }
    }
    
    void publishOffboardControlMode() {
        px4_msgs::msg::OffboardControlMode msg;
        msg.timestamp = getTimestampUs();
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        
        offboard_control_mode_pub_->publish(msg);
    }
    
    void publishTrajectorySetpoint() {
        px4_msgs::msg::TrajectorySetpoint msg;
        msg.timestamp = getTimestampUs();
        
        // Position control
        msg.position[0] = NAN;  // Hold X
        msg.position[1] = NAN;  // Hold Y  
        msg.position[2] = -hover_height_;  // Z in NED (negative is up)
        
        // Velocity
        msg.velocity[0] = NAN;
        msg.velocity[1] = NAN;
        msg.velocity[2] = NAN;
        
        // Yaw control
        auto time_since_target = (this->now() - last_target_time_).seconds();
        if (time_since_target < target_timeout_sec_) {
            msg.yaw = target_yaw_;  // Use commanded yaw
            msg.yawspeed = NAN;
        } else {
            msg.yaw = NAN;  // Hold current yaw
            msg.yawspeed = 0.0;  // Zero yaw rate
            
            // Warn once
            if (!target_timeout_warned_) {
                RCLCPP_WARN(this->get_logger(), 
                    "Target yaw timeout (%.1fs), holding position", time_since_target);
                target_timeout_warned_ = true;
            }
        }
        
        trajectory_setpoint_pub_->publish(msg);
    }
    
    void sendVehicleCommand(uint16_t command, float param1 = 0.0, float param2 = 0.0) {
        px4_msgs::msg::VehicleCommand msg;
        msg.timestamp = getTimestampUs();
        msg.param1 = param1;
        msg.param2 = param2;
        msg.param3 = 0.0;
        msg.param4 = 0.0;
        msg.param5 = 0.0;
        msg.param6 = 0.0;
        msg.param7 = 0.0;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;
        
        vehicle_command_pub_->publish(msg);
    }
    
    uint64_t getTimestampUs() {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    
    // State machine
    enum class State {
        INIT,
        WAIT_FOR_OFFBOARD,
        WAIT_FOR_ARM,
        ACTIVE
    };
    
    // Publishers
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
    
    // Subscribers  
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr target_yaw_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_sub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Parameters
    double hover_height_;
    int pre_stream_cycles_;
    double stream_rate_hz_;
    double target_timeout_sec_;
    bool enable_debug_;
    
    // State
    State state_ = State::INIT;
    int stream_count_ = 0;
    float target_yaw_ = 0.0;
    bool target_timeout_warned_ = false;
    rclcpp::Time last_target_time_;
    px4_msgs::msg::VehicleStatus vehicle_status_;
    uint8_t last_arming_state_ = 255;
    uint8_t last_nav_state_ = 255;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YawControllerNode>());
    rclcpp::shutdown();
    return 0;
}

