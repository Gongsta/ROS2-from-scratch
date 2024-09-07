
/*
Middleware that you can think of as a message broker / lookup table for subscribers. Works
for both intra-process and intra-process.
*/
#ifndef RMW_HPP_
#define RMW_HPP_

#include <any>
#include <memory>
#include <string>
#include <unordered_map>

#include "shared_memory/helpers.hpp"
#include "shared_memory/message_queue_table.hpp"
#include "shared_memory/shared_message_queue.hpp"
#include "subscription.hpp"

using namespace boost::interprocess;

namespace rclcpp {
namespace RMW {
static std::unordered_map<std::string, std::vector<std::shared_ptr<SubscriptionBase>>> inprocess_subscription_table;  // Table shared amongst threads
static std::unordered_map<std::shared_ptr<SubscriptionBase>, std::function<void(std::shared_ptr<SubscriptionBase> sub)>> subscription_executor_table;
// Store events on a node basis

template <typename T>
void publish_message(std::string topic_name, T msg) {
    /* --- INTRA-PROCESS MESSAGE PASSING --- */
    // Publisher directly sees the subscription's queue
    if (inprocess_subscription_table.count(topic_name)) {
        std::cout << "Found " << inprocess_subscription_table[topic_name].size() << " inprocess subscribers for topic " << topic_name << std::endl;
        for (auto sub : inprocess_subscription_table[topic_name]) {
            // Executor associated with subscription is notified of an event.
            auto casted_sub = std::dynamic_pointer_cast<Subscription<T>>(sub);
            if (casted_sub) {
                casted_sub->message_queue->push(msg);
                // notify the executor that a new message arrived
                subscription_executor_table[sub](sub);
            } else {
                std::cerr << "Error casting" << std::endl;
            }
        }
    }
    /* --- INTER-PROCESS MESSAGE PASSING --- */
    // Each subscription will be associated with a shared memory queue. The publisher
    // Merely publishes to the shared memory queue. Since the publisher cannot see the
    // the subscription, the subscriber will need to be notified. Executor's job again
    managed_shared_memory segment(open_or_create, "shared_message_queue_table", 65536);
    MessageQueueTable mq_table(segment);
    scoped_lock<interprocess_mutex> lock(mq_table.mutex);
    const auto *retrieved_memory_names = mq_table.get_memory_region(topic_name, segment);

    if (retrieved_memory_names) {
        std::cout << "Found " << retrieved_memory_names->size() << " outprocess subscribers for topic " << topic_name << std::endl;
        for (const auto &name : *retrieved_memory_names) {
            managed_shared_memory queue_segment(open_only, name.c_str());
            SharedMessageQueue<T> *buffer = queue_segment.find<SharedMessageQueue<T>>("Buffer").first;

            scoped_lock<interprocess_mutex> lock(buffer->mutex);
            if (buffer->queue.size() >= buffer->buffer_size) {
                std::cout << "Queue size exceeded, dropping message" << std::endl;
            } else {
                buffer->queue.push_back(msg);
                buffer->cond_read.notify_one();
            }
        }
    } else {
        std::cout << "no outprocess subscribers found." << std::endl;
    }
}

std::string register_subscription(std::shared_ptr<SubscriptionBase> subscription, std::function<void(std::shared_ptr<SubscriptionBase>)> notify) {
    // --- Inprocess ---
    inprocess_subscription_table[subscription->topic_name].push_back(subscription);
    subscription_executor_table[subscription] = notify;

    // --- Outprocess ---
    std::string region_name = subscription->register_ipc();
    return region_name;
}

}  // namespace RMW
}  // namespace rclcpp

#endif
