#ifndef NODE_HPP_
#define NODE_HPP_

#include <string>
#include <vector>

#include "publisher.hpp"
#include "subscription.hpp"
#include "timer.hpp"

namespace rclcpp {

class Node : public std::enable_shared_from_this<Node> {
   public:
    const std::string name;
    std::vector<std::shared_ptr<SubscriptionBase>> subscriptions;
    std::vector<std::shared_ptr<Timer>> timers;

    Node(const std::string& name) : name{name} {}
    template <typename MessageType>
    std::shared_ptr<Subscription<MessageType>> create_subscription(std::string topic_name, int queue_size, std::function<void(MessageType)> callback) {
        std::shared_ptr<Subscription<MessageType>> s = std::make_shared<Subscription<MessageType>>(topic_name, queue_size, callback);
        this->subscriptions.push_back(s);
        // RMW::register_subscription(topic_name, s); // Registration with RMW done through the executor
        return s;
    }

    template <typename MessageType>
    std::shared_ptr<Publisher<MessageType>> create_publisher(std::string topic_name, int queue_size) {
        std::shared_ptr<Publisher<MessageType>> p = std::make_shared<Publisher<MessageType>>(topic_name, queue_size);
        return p;
    }

    void create_wall_timer(std::chrono::duration<int64_t, std::milli> interval, std::function<void()> callback) {
        this->timers.push_back(std::make_shared<Timer>(interval, callback));
    }
    ~Node() {}  // Remove all subscriptions here
};
};  // namespace rclcpp

#endif
