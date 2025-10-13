
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <chrono>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;

class DummyYawPublisher : public rclcpp::Node {
public:
    DummyYawPublisher() : Node("dummy_yaw_publisher") {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/yaw_command", 10);
        timer_ = this->create_wall_timer(
            2000ms, std::bind(&DummyYawPublisher::timer_callback, this));
        
        // Enhanced test sequence with varied scenarios
        test_sequence_ = {
            0.0,      // 0° - Start position
            1.57,     // 90° - Quarter turn right
            3.14,     // 180° - Half turn
            -1.57,    // -90° - Quarter turn left
            6.28,     // 360° - Full rotation
            0.785,    // 45° - Smaller angle
            -0.785,   // -45° - Negative small angle
            4.71,     // 270° - Three-quarter turn
            2.36,     // 135° - Intermediate angle
            0.0       // Return to start
        };
        
        RCLCPP_INFO(this->get_logger(), "Enhanced DummyYawPublisher started - %zu test positions", test_sequence_.size());
    }

private:
    void timer_callback() {
        auto msg = std_msgs::msg::Float64();
        msg.data = test_sequence_[current_index_];
        
        double degrees = msg.data * 180.0 / M_PI;
        RCLCPP_INFO(this->get_logger(), "Publishing yaw command %zu/%zu: %.3f rad (%.1f°)", 
                    current_index_ + 1, test_sequence_.size(), msg.data, degrees);
        
        publisher_->publish(msg);
        
        current_index_ = (current_index_ + 1) % test_sequence_.size();
        
        if (current_index_ == 0) {
            RCLCPP_INFO(this->get_logger(), "Completed full test sequence, restarting...");
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::vector<double> test_sequence_;
    size_t current_index_ = 0;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DummyYawPublisher>());
    rclcpp::shutdown();
    return 0;
}
