#ifndef EXECUTOR_BASE_HPP_
#define EXECUTOR_BASE_HPP_

#include <thread>

#include "subscription.hpp"

namespace rclcpp {

class ExecutorBase : public std::enable_shared_from_this<ExecutorBase> {
   protected:
    std::mutex event_mutex;
    std::queue<std::shared_ptr<SubscriptionBase>> event_queue;
    std::vector<std::shared_ptr<SubscriptionBase>> subscriptions;
    std::condition_variable cv;

    std::unordered_map<std::string, std::shared_ptr<SubscriptionBase>> ipc_table;

   public:
    virtual void notify(std::shared_ptr<SubscriptionBase> sub) = 0;

    // A function excuted by a custom thread to periodically check for new data.
    void update_ipc_queues() {
        std::unique_lock lock(event_mutex);
        for (auto x : ipc_table) {
            if (x.second->update_queue_from_ipc()) {  // Data added
                event_queue.push(x.second);
                cv.notify_one();
            }
        }
    }  // std::queue<Event> event_queue;
    virtual ~ExecutorBase() {
        std::cout << "destructor called" << std::endl;
        for (auto sub : subscriptions) {
            sub->unregister_ipc();
        }
    }
};

}  // namespace rclcpp

#endif
