#include <functional>
#include <iostream>

#include "node.hpp"
#include "rclcpp.hpp"

// Minimally modified from https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

class HelloWorldSubscriber : public rclcpp::Node {
   public:
    HelloWorldSubscriber() : Node{"minimal_subscriber"} {
        std::cout << "starting subscriber" << std::endl;
        auto subscription = this->create_subscription<std::string>(
            "pub_topic", 10, std::bind(&HelloWorldSubscriber::string_callback, this, std::placeholders::_1));
    }

   private:
    void string_callback(std::string str) {  // TODO: make this a shared value
        std::cout << "Received message: " << str << std::endl;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<HelloWorldSubscriber>();  // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}
