#ifndef SUBSCRIPTION_HPP_
#define SUBSCRIPTION_HPP_

#include <functional>
#include <queue>
#include <string>

#include "message_queue.hpp"
#include "shared_memory/helpers.hpp"
#include "shared_memory/message_queue_table.hpp"
#include "shared_memory/shared_message_queue.hpp"

namespace rclcpp {

// pure virtual base class
class SubscriptionBase {
   public:
    // const std::string topic_name;
    std::string topic_name;
    virtual ~SubscriptionBase() {}

    virtual void execute_callback() = 0;
    virtual std::string register_ipc() = 0;
    virtual void unregister_ipc() = 0;
    virtual bool update_queue_from_ipc() = 0;
};

template <typename MessageType>
class Subscription : public SubscriptionBase {
   private:
    std::function<void(MessageType)> callback;
    std::string memory_region;

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

    // I want to put this code in RMW, but I can't because of dynamic typing...
    std::string register_ipc() {
        managed_shared_memory segment(open_or_create, "shared_message_queue_table", 65536);
        MessageQueueTable mq_table(segment);
        scoped_lock<interprocess_mutex> lock(mq_table.mutex);

        // Allocate a new queue
        std::string region_name = generate_unique_shared_memory_segment();
        managed_shared_memory queue_segment(create_only, region_name.c_str(), 65536);
        memory_region = region_name;

        boost::interprocess::allocator<MessageType, segment_manager_t> alloc_inst(queue_segment.get_segment_manager());

        // Construct the shared memory buffer in shared memory
        int buffer_size = 10;  // Maximum size of the queue
        queue_segment.construct<SharedMessageQueue<MessageType>>("Buffer")(alloc_inst, buffer_size);
        std::vector<std::string> memory_region_names;
        const auto* retrieved_memory_names = mq_table.get_memory_region(this->topic_name, segment);
        if (retrieved_memory_names) {  // Append existing names
            for (const auto& shmem_str : *retrieved_memory_names) {
                memory_region_names.push_back(std::string(shmem_str.c_str()));  // Convert shared memory strings to std::string
            }
        }
        memory_region_names.push_back(region_name);
        // Insert or update a topic and its associated vector of memory region names into the table
        mq_table.insert_or_update(this->topic_name, memory_region_names, segment);
        return region_name;
    }

    bool update_queue_from_ipc() {
        // Function to check queues on shared memory and fill locally if needed
        if (memory_region.empty()) {  // need to register with IPC first
            return false;
        }
        managed_shared_memory queue_segment(open_only, memory_region.c_str());
        SharedMessageQueue<MessageType>* buffer = queue_segment.find<SharedMessageQueue<MessageType>>("Buffer").first;
        scoped_lock<interprocess_mutex> lock(buffer->mutex);
        if (buffer->queue.size() > 0) {
            MessageType data = buffer->queue.front();
            buffer->queue.pop_front();
            message_queue->push(data);
            return true;
        }
        return false;
    }
    void unregister_ipc() {
        // Delete queue
        shared_memory_object::remove(memory_region.c_str());

        // Delete from table
        managed_shared_memory segment(open_or_create, "shared_message_queue_table", 65536);
        MessageQueueTable mq_table(segment);
        scoped_lock<interprocess_mutex> lock(mq_table.mutex);
        // Allocate a new queue
        std::vector<std::string> memory_region_names;
        const auto* retrieved_memory_names = mq_table.get_memory_region(this->topic_name, segment);
        if (retrieved_memory_names) {
            for (const auto& shmem_str : *retrieved_memory_names) {
                if (shmem_str.c_str() != memory_region.c_str()) {
                    memory_region_names.push_back(std::string(shmem_str.c_str()));
                }
            }
        }
        mq_table.insert_or_update(this->topic_name, memory_region_names, segment);
    }
};

}  // namespace rclcpp

#endif
