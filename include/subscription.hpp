#ifndef SUBSCRIPTION_HPP_
#define SUBSCRIPTION_HPP_

#include <functional>
#include <queue>
#include <string>

namespace rclcpp {

// pure virtual base class
class SubscriptionBase {
   public:
    virtual ~SubscriptionBase() = default;
    virtual bool has_new_messages() = 0;
    virtual void execute_callback() = 0;
};

template <typename MessageType>
class Subscription : public SubscriptionBase {
   private:
    const std::string topic_name;
    std::function<void(MessageType)> callback;
    std::shared_ptr<std::queue<MessageType>> message_queue;  // TODO: Fix concurrency. Needs to register this

    MessageType get_message() {
        auto message = message_queue->front();
        message_queue->pop();
        return message;
    }

   public:
    Subscription(std::string topic_name, std::function<void(MessageType)> callback) : topic_name{topic_name}, callback{callback} {};
    ~Subscription() {};
    bool has_new_messages() {
        return message_queue->size() != 0;
    }
    void execute_callback() {
        auto message = this->get_message();
        this->callback(message);
    }
};

}  // namespace rclcpp

#endif
