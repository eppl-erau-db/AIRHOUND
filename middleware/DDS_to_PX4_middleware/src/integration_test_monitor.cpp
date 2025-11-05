#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <vector>
#include <unordered_map>
#include <fstream>

using namespace std::chrono_literals;

class IntegrationTestMonitor : public rclcpp::Node {
public:
    IntegrationTestMonitor() : Node("integration_test_monitor") {
        // Parameters
        this->declare_parameter("test_duration", 30);
        this->declare_parameter("monitor_topics", std::vector<std::string>{
            "/yaw_command", "/fmu/in/trajectory_setpoint", "/fmu/out/vehicle_status"
        });
        // Note: use_sim_time is automatically declared by ROS2
        
        int test_duration = this->get_parameter("test_duration").as_int();
        auto monitor_topics = this->get_parameter("monitor_topics").as_string_array();
        
        // Setup monitoring subscriptions
        setup_monitoring_subscriptions();
        
        // Results publisher
        results_pub_ = this->create_publisher<std_msgs::msg::String>("/test_results", 10);
        
        // Monitoring timer
        monitor_timer_ = this->create_wall_timer(1000ms, 
            std::bind(&IntegrationTestMonitor::monitor_callback, this));
        
        // Test completion timer
        completion_timer_ = this->create_wall_timer(
            std::chrono::seconds(test_duration),
            std::bind(&IntegrationTestMonitor::complete_test, this)
        );
        
        start_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "📊 Integration Test Monitor started");
        RCLCPP_INFO(this->get_logger(), "⏱️  Test duration: %d seconds", test_duration);
        RCLCPP_INFO(this->get_logger(), "🎯 Monitoring topics: %zu", monitor_topics.size());
    }

private:
    void setup_monitoring_subscriptions() {
        // Monitor yaw commands from demo publisher
        yaw_command_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/yaw_command", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                track_message("/yaw_command", msg->data);
                last_yaw_command_ = msg->data;
                yaw_command_count_++;
            });
        
        // Monitor demo status
        demo_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/demo_status", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                track_message("/demo_status", msg->data);
            });
        
        // Monitor PX4 converter status
        px4_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/px4_converter_status", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                track_message("/px4_converter_status", msg->data);
                px4_status_count_++;
            });
    }
    
    template<typename T>
    void track_message(const std::string& topic, const T& data) {
        auto current_time = this->now();
        
        // Update topic statistics
        topic_stats_[topic].message_count++;
        topic_stats_[topic].last_received = current_time;
        
        if (topic_stats_[topic].first_received.nanoseconds() == 0) {
            topic_stats_[topic].first_received = current_time;
        }
        
        // Calculate rates
        double elapsed = (current_time - start_time_).seconds();
        if (elapsed > 0) {
            topic_stats_[topic].rate_hz = topic_stats_[topic].message_count / elapsed;
        }
    }
    
    void monitor_callback() {
        auto current_time = this->now();
        double elapsed_time = (current_time - start_time_).seconds();
        
        // Log current statistics
        RCLCPP_INFO(this->get_logger(), 
            "📈 Test Progress [%.1fs]: Yaw commands: %d, PX4 status: %d",
            elapsed_time, yaw_command_count_, px4_status_count_);
        
        // Check for data flow issues
        check_data_flow_health();
        
        // Publish intermediate results
        publish_intermediate_results(elapsed_time);
    }
    
    void check_data_flow_health() {
        auto current_time = this->now();
        
        for (const auto& [topic, stats] : topic_stats_) {
            if (stats.message_count > 0) {
                double time_since_last = (current_time - stats.last_received).seconds();
                
                if (time_since_last > 5.0) { // 5 second timeout
                    RCLCPP_WARN(this->get_logger(), 
                        "⚠️  No messages on %s for %.1fs", topic.c_str(), time_since_last);
                }
            }
        }
        
        // Check conversion efficiency
        if (yaw_command_count_ > 0 && px4_status_count_ > 0) {
            double conversion_ratio = static_cast<double>(px4_status_count_) / yaw_command_count_;
            if (conversion_ratio < 0.5) {
                RCLCPP_WARN(this->get_logger(), 
                    "⚠️  Low conversion efficiency: %.2f (PX4 status / yaw commands)",
                    conversion_ratio);
            }
        }
    }
    
    void publish_intermediate_results(double elapsed_time) {
        auto msg = std_msgs::msg::String();
        msg.data = "INTERMEDIATE_RESULTS: Time=" + std::to_string(elapsed_time) +
                   ", YawCommands=" + std::to_string(yaw_command_count_) +
                   ", PX4Status=" + std::to_string(px4_status_count_) +
                   ", LastYaw=" + std::to_string(last_yaw_command_);
        results_pub_->publish(msg);
    }
    
    void complete_test() {
        auto end_time = this->now();
        double total_test_time = (end_time - start_time_).seconds();
        
        RCLCPP_INFO(this->get_logger(), "🏁 Integration test completed!");
        
        // Generate comprehensive test report
        generate_test_report(total_test_time);
        
        // Publish final results
        publish_final_results(total_test_time);
        
        completion_timer_->cancel();
    }
    
    void generate_test_report(double total_time) {
        RCLCPP_INFO(this->get_logger(), "📋 === INTEGRATION TEST REPORT ===");
        RCLCPP_INFO(this->get_logger(), "⏱️  Total test duration: %.2f seconds", total_time);
        RCLCPP_INFO(this->get_logger(), "📊 Message Statistics:");
        
        for (const auto& [topic, stats] : topic_stats_) {
            RCLCPP_INFO(this->get_logger(), 
                "   %s: %d messages (%.2f Hz)",
                topic.c_str(), stats.message_count, stats.rate_hz);
        }
        
        // Performance analysis
        RCLCPP_INFO(this->get_logger(), "🎯 Performance Analysis:");
        RCLCPP_INFO(this->get_logger(), "   Yaw commands received: %d", yaw_command_count_);
        RCLCPP_INFO(this->get_logger(), "   PX4 status updates: %d", px4_status_count_);
        
        if (yaw_command_count_ > 0) {
            double avg_yaw_rate = yaw_command_count_ / total_time;
            RCLCPP_INFO(this->get_logger(), "   Average yaw command rate: %.2f Hz", avg_yaw_rate);
            
            if (px4_status_count_ > 0) {
                double conversion_efficiency = static_cast<double>(px4_status_count_) / yaw_command_count_;
                RCLCPP_INFO(this->get_logger(), "   Conversion efficiency: %.2f%%", conversion_efficiency * 100);
            }
        }
        
        // Test result summary
        bool test_passed = evaluate_test_success();
        RCLCPP_INFO(this->get_logger(), "✅ Overall test result: %s", 
                    test_passed ? "PASSED" : "FAILED");
        
        if (test_passed) {
            RCLCPP_INFO(this->get_logger(), "🎉 DDS to PX4 conversion is working correctly!");
        } else {
            RCLCPP_WARN(this->get_logger(), "❌ Issues detected in DDS to PX4 conversion");
        }
        
        RCLCPP_INFO(this->get_logger(), "📋 === END OF REPORT ===");
    }
    
    bool evaluate_test_success() {
        // Test success criteria
        bool has_yaw_commands = yaw_command_count_ > 0;
        bool has_px4_status = px4_status_count_ > 0;
        bool good_conversion_rate = (px4_status_count_ > 0 && yaw_command_count_ > 0) ?
            (static_cast<double>(px4_status_count_) / yaw_command_count_) > 0.3 : false;
        
        return has_yaw_commands && has_px4_status && good_conversion_rate;
    }
    
    void publish_final_results(double total_time) {
        auto msg = std_msgs::msg::String();
        msg.data = "FINAL_RESULTS: Duration=" + std::to_string(total_time) +
                   ", YawCommands=" + std::to_string(yaw_command_count_) +
                   ", PX4Status=" + std::to_string(px4_status_count_) +
                   ", TestPassed=" + (evaluate_test_success() ? "true" : "false");
        results_pub_->publish(msg);
    }

    struct TopicStats {
        int message_count = 0;
        double rate_hz = 0.0;
        rclcpp::Time first_received = rclcpp::Time(0);
        rclcpp::Time last_received = rclcpp::Time(0);
    };

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_command_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr demo_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr px4_status_sub_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr results_pub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    rclcpp::TimerBase::SharedPtr completion_timer_;
    
    // Statistics
    std::unordered_map<std::string, TopicStats> topic_stats_;
    int yaw_command_count_ = 0;
    int px4_status_count_ = 0;
    double last_yaw_command_ = 0.0;
    
    rclcpp::Time start_time_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IntegrationTestMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}