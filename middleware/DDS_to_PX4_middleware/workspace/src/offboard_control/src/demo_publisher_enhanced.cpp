#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <chrono>
#include <vector>
#include <cmath>
#include <random>

using namespace std::chrono_literals;

class DemoPublisherEnhanced : public rclcpp::Node {
public:
    DemoPublisherEnhanced() : Node("demo_publisher_enhanced") {
        // Parameters
        this->declare_parameter("publish_rate", 2.0);
        this->declare_parameter("test_duration", 30);
        this->declare_parameter("enable_cycling", true);
        this->declare_parameter("start_delay", 0.0);
        this->declare_parameter("test_mode", "full");  // "full", "basic", "cardinals"
        // Note: use_sim_time is automatically declared by ROS2

        double publish_rate = this->get_parameter("publish_rate").as_double();
        int test_duration = this->get_parameter("test_duration").as_int();
        enable_cycling_ = this->get_parameter("enable_cycling").as_bool();
        double start_delay = this->get_parameter("start_delay").as_double();
        std::string test_mode = this->get_parameter("test_mode").as_string();
        
        // Publishers
        yaw_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/yaw_command", 10);
        status_publisher_ = this->create_publisher<std_msgs::msg::String>("/demo_status", 10);
        
        // Comprehensive test sequence for PX4 validation
        setupTestSequence(test_mode);

        // Timer for publishing (with optional start delay)
        auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate));
        if (start_delay > 0.0) {
            RCLCPP_INFO(this->get_logger(), "[INFO] Delaying start by %.1f seconds", start_delay);
            start_delay_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(start_delay * 1000.0)),
                [this, timer_period]() {
                    timer_ = this->create_wall_timer(timer_period,
                                                     std::bind(&DemoPublisherEnhanced::timer_callback, this));
                    start_delay_timer_->cancel();
                    RCLCPP_INFO(this->get_logger(), "[INFO] Publishing started");
                });
        } else {
            timer_ = this->create_wall_timer(timer_period, std::bind(&DemoPublisherEnhanced::timer_callback, this));
        }
        
        // Test completion timer
        completion_timer_ = this->create_wall_timer(
            std::chrono::seconds(test_duration),
            std::bind(&DemoPublisherEnhanced::complete_test, this)
        );
        
        start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "[START] Enhanced Demo Publisher started");
        RCLCPP_INFO(this->get_logger(), "[INFO] Test mode: %s", test_mode.c_str());
        RCLCPP_INFO(this->get_logger(), "[INFO] Test sequence: %zu positions, %.1f Hz, %d seconds, cycling: %s",
                    test_sequence_.size(), publish_rate, test_duration,
                    enable_cycling_ ? "enabled" : "disabled");
    }

private:
    void setupTestSequence(const std::string& mode) {
        if (mode == "cardinals") {
            // Basic cardinal directions only
            test_sequence_ = {
                {0.0, "NORTH (0 deg)"},
                {M_PI/2, "EAST (90 deg)"},
                {M_PI, "SOUTH (180 deg)"},
                {-M_PI/2, "WEST (-90 deg)"},
                {0.0, "Return to North (0 deg)"}
            };
        } else if (mode == "basic") {
            // Cardinals + diagonals
            test_sequence_ = {
                {0.0, "NORTH (0 deg)"},
                {M_PI/4, "NE (45 deg)"},
                {M_PI/2, "EAST (90 deg)"},
                {3*M_PI/4, "SE (135 deg)"},
                {M_PI, "SOUTH (180 deg)"},
                {-3*M_PI/4, "SW (-135 deg)"},
                {-M_PI/2, "WEST (-90 deg)"},
                {-M_PI/4, "NW (-45 deg)"},
                {0.0, "Return to North (0 deg)"}
            };
        } else {
            // Full comprehensive test (original)
            test_sequence_ = {
            // Basic cardinal directions
            {0.0, "NORTH (0°)"},
            {M_PI/2, "EAST (90°)"},
            {M_PI, "SOUTH (180°)"},
            {-M_PI/2, "WEST (-90°)"},
            
            // Intermediate angles
            {M_PI/4, "NE (45°)"},
            {3*M_PI/4, "SE (135°)"},
            {-3*M_PI/4, "SW (-135°)"},
            {-M_PI/4, "NW (-45°)"},
            
            // Small precision movements
            {0.1, "Small Right (5.7°)"},
            {-0.1, "Small Left (-5.7°)"},
            {0.05, "Micro Right (2.9°)"},
            {-0.05, "Micro Left (-2.9°)"},
            
            // Large rotations (testing wrap-around)
            {2*M_PI, "Full CW (360°)"},
            {-2*M_PI, "Full CCW (-360°)"},
            {3*M_PI/2, "270° CW"},
            {-3*M_PI/2, "270° CCW"},
            
            // Realistic flight patterns
            {M_PI/6, "Shallow Right (30°)"},
            {M_PI/3, "Medium Right (60°)"},
            {2*M_PI/3, "Sharp Right (120°)"},
            {-M_PI/6, "Shallow Left (-30°)"},
            {-M_PI/3, "Medium Left (-60°)"},
            {-2*M_PI/3, "Sharp Left (-120°)"},
            
            // Return to origin
            {0.0, "Return to North (0 deg)"}
            };
        }
    }
    
    void timer_callback() {
        if (current_index_ >= test_sequence_.size()) {
            return; // Test complete
        }
        
        auto current_time = this->now();
        double elapsed_seconds = (current_time - start_time_).seconds();
        
        // Publish yaw command
        auto yaw_msg = std_msgs::msg::Float64();
        yaw_msg.data = test_sequence_[current_index_].first;
        yaw_publisher_->publish(yaw_msg);
        
        // Publish status
        auto status_msg = std_msgs::msg::String();
        status_msg.data = "Demo Step " + std::to_string(current_index_ + 1) + "/" + 
                         std::to_string(test_sequence_.size()) + ": " + 
                         test_sequence_[current_index_].second;
        status_publisher_->publish(status_msg);
        
        // Log progress
        double degrees = yaw_msg.data * 180.0 / M_PI;
        RCLCPP_INFO(this->get_logger(),
            "[PUBLISH] [%zu/%zu] %.1fs: %.3f rad (%.1f deg) - %s",
            current_index_ + 1, test_sequence_.size(),
            elapsed_seconds, yaw_msg.data, degrees,
            test_sequence_[current_index_].second.c_str());
        
        current_index_++;
        
        if (current_index_ >= test_sequence_.size()) {
            if (enable_cycling_) {
                RCLCPP_INFO(this->get_logger(), "[COMPLETE] Demo sequence complete, cycling...");
                current_index_ = 0; // Reset for cycling
            } else {
                RCLCPP_INFO(this->get_logger(), "[COMPLETE] Demo sequence complete, stopping");
                timer_->cancel(); // Stop publishing
            }
        }
    }
    
    void complete_test() {
        RCLCPP_INFO(this->get_logger(), "[DONE] Test duration completed");

        auto status_msg = std_msgs::msg::String();
        status_msg.data = "TEST_COMPLETE";
        status_publisher_->publish(status_msg);

        // Keep node alive but stop publishing new commands
        completion_timer_->cancel();
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr completion_timer_;
    rclcpp::TimerBase::SharedPtr start_delay_timer_;

    std::vector<std::pair<double, std::string>> test_sequence_;
    size_t current_index_ = 0;
    rclcpp::Time start_time_;
    bool enable_cycling_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DemoPublisherEnhanced>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}