#ifndef NODE_HPP_
#define NODE_HPP_

#include <string>
#include <vector>

#include "publisher.hpp"
#include "subscription.hpp"
#include "timer.hpp"

namespace rclcpp {

class Node {
   private:
    const std::string name;
    std::vector<std::shared_ptr<SubscriptionBase>> subscriptions;
    std::vector<std::shared_ptr<Timer>> timers;

   public:
    Node(const std::string& name) : name{name} {}
    template <typename MessageType>
    std::shared_ptr<Subscription<MessageType>> create_subscription(std::string topic_name, int queue_size, std::function<void(MessageType)> callback) {
        std::shared_ptr<Subscription<MessageType>> s = std::make_shared<Subscription<MessageType>>(topic_name, callback);
        this->subscriptions.push_back(s);
        // In ROS1, there's this concept
        return s;
    }

    template <typename MessageType>
    std::shared_ptr<Publisher<MessageType>> create_publisher(std::string topic_name, int queue_size) {
        std::shared_ptr<Publisher<MessageType>> p = std::make_shared<Publisher<MessageType>>(topic_name, queue_size);
        return p;
    }

    // Main loop that iterates through the subscriptions
    void spin() {
        while (true) {
            // Iterate through subscriptions
            for (auto sub : this->subscriptions) {
                if (sub->has_new_messages()) {  // check queue of messages
                    sub->execute_callback();
                }
            }
            // Iterate through timers
            for (auto timer : this->timers) {
                if (timer->tick()) {
                    timer->callback();
                }
            }
        }
    }

    void create_wall_timer(std::chrono::duration<int64_t, std::milli> interval, std::function<void()> callback) {
        this->timers.push_back(std::make_shared<Timer>(interval, callback));
    }
    ~Node() {}  // Remove all subscriptions here
};
};  // namespace rclcpp

#endif
