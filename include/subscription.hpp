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
    virtual ~SubscriptionBase() = default;
    virtual void execute_callback() = 0;
};

template <typename MessageType>
class Subscription : public SubscriptionBase {
   private:
    const std::string topic_name;
    std::function<void(MessageType)> callback;

   public:
    std::shared_ptr<MessageQueue<MessageType>> message_queue;
    std::mutex mut;
    Subscription(std::string topic_name, std::function<void(MessageType)> callback) : topic_name{topic_name}, callback{callback}, message_queue{std::make_shared<MessageQueue<MessageType>>()} {};
    ~Subscription() {};
    void execute_callback() {
        auto message = message_queue->pop();  // blocking, using condition variable
        this->callback(message);
    }
};

}  // namespace rclcpp

#endif
