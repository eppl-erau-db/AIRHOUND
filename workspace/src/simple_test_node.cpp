#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>

using namespace std::chrono_literals;

class SimpleTestNode : public rclcpp::Node {
public:
    SimpleTestNode() : Node("simple_test_node") {
        // Publisher for output data
        output_publisher_ = this->create_publisher<std_msgs::msg::String>("/middleware_output", 10);
        
        // Subscriber for input data
        input_subscriber_ = this->create_subscription<std_msgs::msg::Float64>(
            "/yaw_command", 10, std::bind(&SimpleTestNode::input_callback, this, std::placeholders::_1));
        
        // Timer for periodic status updates
        timer_ = this->create_wall_timer(
            1000ms, std::bind(&SimpleTestNode::timer_callback, this));
            
        start_time_ = this->now();
        last_command_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Enhanced SimpleTestNode started - monitoring yaw commands");
    }

private:
    void input_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        auto current_time = this->now();
        double time_since_last = 0.0;
        
        if (command_count_ > 0) {
            time_since_last = (current_time - last_command_time_).seconds();
        }
        
        // Calculate change in yaw
        double yaw_change = msg->data - last_input_;
        double degrees = msg->data * 180.0 / M_PI;
        double change_degrees = yaw_change * 180.0 / M_PI;
        
        // Classify the command
        std::string command_type;
        if (std::abs(yaw_change) < 0.1) command_type = "HOLD";
        else if (yaw_change > 0) command_type = "RIGHT";
        else command_type = "LEFT";
        
        RCLCPP_INFO(this->get_logger(), 
            "Yaw Command [%s]: %.3f rad (%.1f°) | Change: %+.3f rad (%+.1f°) | Dt: %.2fs",
            command_type.c_str(), msg->data, degrees, yaw_change, change_degrees, time_since_last);
        
        // Validate command range
        if (std::abs(msg->data) > 10.0) {
            RCLCPP_WARN(this->get_logger(), "WARNING: Large yaw command detected! %.3f rad", msg->data);
        }
        
        last_input_ = msg->data;
        last_command_time_ = current_time;
        command_count_++;
        
        // Track statistics
        total_rotation_ += std::abs(yaw_change);
    }
    
    void timer_callback() {
        auto current_time = this->now();
        double elapsed_time = (current_time - start_time_).seconds();
        double time_since_last = 0.0;
        
        if (command_count_ > 0) {
            time_since_last = (current_time - last_command_time_).seconds();
        }
        
        auto msg = std_msgs::msg::String();
        msg.data = "Middleware Stats - Commands: " + std::to_string(command_count_) + 
                   ", Total rotation: " + std::to_string(total_rotation_) + " rad, " +
                   "Current yaw: " + std::to_string(last_input_) + " rad";
        output_publisher_->publish(msg);
        
        // Periodic status report
        if (command_count_ > 0) {
            RCLCPP_INFO(this->get_logger(), 
                "Status: %d commands in %.1fs | Avg rate: %.2f Hz | Total rotation: %.2f rad (%.0f°) | Last command: %.2fs ago",
                command_count_, elapsed_time, command_count_/elapsed_time, 
                total_rotation_, total_rotation_ * 180.0/M_PI, time_since_last);
        }
    }
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr output_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr input_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    double last_input_ = 0.0;
    rclcpp::Time start_time_;
    rclcpp::Time last_command_time_;
    int command_count_ = 0;
    double total_rotation_ = 0.0;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleTestNode>());
    rclcpp::shutdown();
    return 0;
}