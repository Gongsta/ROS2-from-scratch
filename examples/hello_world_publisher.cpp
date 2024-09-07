#include <chrono>
#include <functional>
#include <iostream>
#include <string>

#include "node.hpp"
#include "rclcpp.hpp"

// Minimally modified from https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

using namespace std::chrono_literals;

class HelloWorldPublisher : public rclcpp::Node {
   public:
    std::shared_ptr<rclcpp::Publisher<std::string>> publisher_;
    int counter = 0;
    HelloWorldPublisher() : Node("minimal_publisher") {
        publisher_ =
            this->create_publisher<std::string>(
                "pub_topic", 10);

        this->create_wall_timer(50ms, std::bind(&HelloWorldPublisher::timer_callback, this));
    }

    void timer_callback() {
        std::string message{"Hello world " + std::to_string(counter)};
        counter++;
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
