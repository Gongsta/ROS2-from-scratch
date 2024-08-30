/*
An example having 2 nodes running in the same component.
*/
#include <chrono>
#include <functional>
#include <iostream>
#include <thread>

#include "node.hpp"
#include "rclcpp.hpp"

using namespace std::chrono_literals;

class HelloWorldSubscriber : public rclcpp::Node {
   public:
    HelloWorldSubscriber() : Node{"minimal_subscriber"} {
        std::cout << "starting subscriber" << std::endl;
        auto subscription = this->create_subscription<std::string>(
            "pub_topic", 10, std::bind(&HelloWorldSubscriber::string_callback, this, std::placeholders::_1));
    }

   private:
    void string_callback(std::string str) {
        std::cout << "Received message: " << str << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
};

class HelloWorldPublisher : public rclcpp::Node {
    int counter = 0;

   public:
    std::shared_ptr<rclcpp::Publisher<std::string>> publisher_;
    HelloWorldPublisher() : Node("minimal_publisher") {
        publisher_ =
            this->create_publisher<std::string>(
                "pub_topic", 10);

        this->create_wall_timer(10ms, std::bind(&HelloWorldPublisher::timer_callback, this));
    }

    void timer_callback() {
        std::string message = "Hello world " + std::to_string(counter);
        counter++;
        // message.data = "Hello, world! " + std::to_string(this->count_++);
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        std::cout << "sending " << message << std::endl;
        publisher_->publish(message);
    };
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<HelloWorldPublisher>();    // initialise node pointer
    auto node2_ptr = std::make_shared<HelloWorldSubscriber>();  // initialise node pointer
    rclcpp::MultithreadedExecutor executor;
    executor.add_node(node_ptr);
    executor.add_node(node2_ptr);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
