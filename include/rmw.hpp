#ifndef RMW_HPP_
#define RMW_HPP_

#include <map>
#include <memory>
#include <string>

#include "executor_base.hpp"
#include "subscription.hpp"

// Middleware that you can think of as a lookup table for subscribers
namespace rclcpp {
namespace RMW {
static std::map<std::string, std::vector<std::shared_ptr<SubscriptionBase>>> inprocess_subscription_table;  // Table shared amongst threads
static std::map<std::shared_ptr<SubscriptionBase>, std::function<void(std::shared_ptr<SubscriptionBase> sub)>> subscription_executor_table;
// Store events on a node basis

void register_node() {
    // Node ID (Participant ID)
}

// Idea: Use an abstract interface that are events so that we have an even

template <typename T>
void publish_message(std::string topic_name, T msg) {
    // INTRA-PROCESS MESSAGE PASSING
    std::cout << "publishing" << std::endl;
    if (inprocess_subscription_table.count(topic_name)) {
        for (auto sub : inprocess_subscription_table[topic_name]) {
            // Executor associated with subscription is notified of an event.
            auto casted_sub = std::dynamic_pointer_cast<Subscription<T>>(sub);
            if (casted_sub) {
                casted_sub->message_queue->push(msg);
                // notify the executor that a new message is running
                subscription_executor_table[sub](sub);
            } else {
                std::cerr << "Error casting" << std::endl;
            }
        }
    } else {
        std::cout << "not found" << std::endl;
    }
    // INTER-PROCESS MESSAGE PASSING
    std::cout << "done publishing" << std::endl;
}

void register_subscription(std::shared_ptr<SubscriptionBase> subscription, std::function<void(std::shared_ptr<SubscriptionBase>)> notify) {
    std::cout << "registering" << std::endl;
    inprocess_subscription_table[subscription->topic_name].push_back(subscription);
    subscription_executor_table[subscription] = notify;
}

}  // namespace RMW
}  // namespace rclcpp

#endif
