#include <chrono>
#include <functional>
#include <iostream>
#include <string>

#include "node.hpp"
#include "rclcpp.hpp"

using namespace std::chrono_literals;

class HelloWorldPublisher : public rclcpp::Node {
   public:
    std::shared_ptr<rclcpp::Publisher<std::string>> publisher_;
    HelloWorldPublisher() : Node("minimal_publisher") {
        publisher_ =
            this->create_publisher<std::string>(
                "pub_topic", 10);

        this->create_wall_timer(500ms, std::bind(&HelloWorldPublisher::timer_callback, this));
    }

    void timer_callback() {
        std::string message{"Hello world"};
        // message.data = "Hello, world! " + std::to_string(this->count_++);
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        std::cout << "sending " << message << std::endl;
        publisher_->publish(message);
    };
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<HelloWorldPublisher>();  // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}

// #include <chrono>
// #include <memory>
// #include <string>

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"

// using namespace std::chrono_literals;

// /* This example creates a subclass of Node and uses a fancy C++11 lambda
//  * function to shorten the callback syntax, at the expense of making the
//  * code somewhat more difficult to understand at first glance. */

// class MinimalPublisher : public rclcpp::Node {
//    public:
//     MinimalPublisher()
//         : Node("minimal_publisher"), count_(0) {
//         publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
//         auto timer_callback =
//             [this]() -> void {
//             auto message = std_msgs::msg::String();
//             message.data = "Hello, world! " + std::to_string(this->count_++);
//             RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
//             this->publisher_->publish(message);
//         };
//         timer_ = this->create_wall_timer(500ms, timer_callback);
//     }

//    private:
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
//     size_t count_;
// };

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<MinimalPublisher>());
//     rclcpp::shutdown();
//     return 0;
// }
