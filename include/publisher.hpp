#ifndef PUBLISHER_HPP_
#define PUBLISHER_HPP_

#include <string>

#include "rmw.hpp"

namespace rclcpp {

template <typename T>
class Publisher {
    const std::string topic_name;
    double queue_size;

   public:
    Publisher(std::string topic_name, double queue_size) : topic_name{topic_name}, queue_size{queue_size} {}
    ~Publisher() {};
    void publish(T msg) {
        RMW::publish_message<T>(topic_name, msg);
        return;
    }
};

}  // namespace rclcpp

#endif
