#ifndef EXECUTOR_BASE_HPP_
#define EXECUTOR_BASE_HPP_

#include <thread>

#include "subscription.hpp"

namespace rclcpp {

class ExecutorBase {
   protected:
    std::mutex event_mutex;
    std::queue<std::shared_ptr<SubscriptionBase>> event_queue;
    std::condition_variable cv;

   public:
    virtual void notify(std::shared_ptr<SubscriptionBase> sub) = 0;
    // std::queue<Event> event_queue;
};

}  // namespace rclcpp

#endif
