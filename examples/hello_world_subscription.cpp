#include <functional>
#include <iostream>

#include "node.hpp"
#include "rclcpp.hpp"

class HelloWorldSubscriber : public rclcpp::Node {
   public:
    HelloWorldSubscriber() : Node{"minimal_subscriber"} {
        std::cout << "starting subscriber" << std::endl;
        auto subscription = this->create_subscription<std::string>(
            "sub_topic", 10, std::bind(&HelloWorldSubscriber::string_callback, this, std::placeholders::_1));
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

// #include <memory>

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"

// class MinimalSubscriber : public rclcpp::Node {
//    public:
//     MinimalSubscriber()
//         : Node("minimal_subscriber") {
//         auto topic_callback =
//             [this](std_msgs::msg::String::UniquePtr msg) -> void {
//             RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
//         };
//         subscription_ =
//             this->create_subscription<std_msgs::msg::String>("topic", 10, topic_callback);
//     }

//    private:
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
// };

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<MinimalSubscriber>());
//     rclcpp::shutdown();
//     return 0;
// }
