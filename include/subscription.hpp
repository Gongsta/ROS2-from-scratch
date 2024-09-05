#ifndef SUBSCRIPTION_HPP_
#define SUBSCRIPTION_HPP_

#include <functional>
#include <queue>
#include <string>

#include "message_queue.hpp"

namespace rclcpp {

// pure virtual base class
class SubscriptionBase {
   public:
    // const std::string topic_name;
    std::string topic_name;
    virtual ~SubscriptionBase() = default;
    virtual void execute_callback() = 0;
};

template <typename MessageType>
class Subscription : public SubscriptionBase {
   private:
    std::function<void(MessageType)> callback;

   public:
    std::shared_ptr<MessageQueue<MessageType>> message_queue;  // Populated with data by the RMW (see `rmw.hpp`)

    Subscription(std::string topic_name, int queue_size, std::function<void(MessageType)> callback) : callback{callback}, message_queue{std::make_shared<MessageQueue<MessageType>>(queue_size)} {
        this->topic_name = topic_name;
    };
    ~Subscription() {};
    void execute_callback() {
        auto message = message_queue->pop();  // Executor calls this
        this->callback(message);
    }
};

}  // namespace rclcpp

#endif
