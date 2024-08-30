// Stress Test
#include <chrono>
#include <functional>
#include <iostream>
#include <thread>

#include "node.hpp"
#include "rclcpp.hpp"

using namespace std::chrono_literals;

class CustomNode : public rclcpp::Node {
    std::shared_ptr<rclcpp::Publisher<std::string>> publisher_;
    int counter = 0;

   public:
    CustomNode(std::string name, int number) : Node{name} {
        publisher_ = this->create_publisher<std::string>("topic" + std::to_string(number + 1), 10);
        if (number == 0) {
            this->create_wall_timer(10ms, std::bind(&CustomNode::timer_callback, this));
        } else {
            auto subscription = this->create_subscription<std::string>(
                "topic" + std::to_string(number), 10, std::bind(&CustomNode::string_callback, this, std::placeholders::_1));
        }
    }

   private:
    void string_callback(std::string str) {
        std::cout << "Received message: " << str << std::endl;
        publisher_->publish(str);
    }
    void timer_callback() {
        publisher_->publish(std::to_string(counter));
        counter++;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::MultithreadedExecutor executor;
    for (int i = 0; i < 1000; i++) {
        auto node = std::make_shared<CustomNode>("Custom", i);
        executor.add_node(node);
    }
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
