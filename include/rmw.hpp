#ifndef RMW_HPP_
#define RMW_HPP_

#include <map>
#include <memory>
#include <string>

#include "subscription.hpp"

// Middleware that you can think of as a lookup table for subscribers

namespace rclcpp {

namespace RMW {
static std::map<std::string, std::vector<std::shared_ptr<SubscriptionBase>>> subscription_table;

template <typename T>
void publish_message(std::string topic_name, T msg) {
    if (subscription_table.count(topic_name)) {
        for (auto sub : subscription_table[topic_name]) {
            auto casted_sub = std::dynamic_pointer_cast<Subscription<T>>(sub);
            if (casted_sub) {
                casted_sub->message_queue->push(msg);
            } else {
                std::cerr << "Error casting" << std::endl;
            }
        }
    } else {
        std::cout << "publishing message with no subscriptions" << std::endl;
    }
}

void register_subscription(std::string topic_name, std::shared_ptr<SubscriptionBase> subscription) {
    subscription_table[topic_name].push_back(subscription);
}

}  // namespace RMW
}  // namespace rclcpp

#endif
