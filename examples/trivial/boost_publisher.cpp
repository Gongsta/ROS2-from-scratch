#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "shared_memory/shared_message_queue.hpp"

using namespace boost::interprocess;

int main() {
    // Remove shared memory on exit
    struct shm_remove {
        shm_remove() { shared_memory_object::remove("SharedMemory"); }
        ~shm_remove() { shared_memory_object::remove("SharedMemory"); }
    } remover;

    // Create a new shared memory segment
    managed_shared_memory segment(create_only, "SharedMemory", 65536);

    // Create an allocator for shared memory
    boost::interprocess::allocator<int, segment_manager_t> alloc_inst(segment.get_segment_manager());

    // Construct the shared memory buffer in shared memory
    int buffer_size = 10;  // Maximum size of the queue
    SharedMessageQueue<int>* buffer = segment.construct<SharedMessageQueue<int>>("Buffer")(alloc_inst, buffer_size);

    // Continuously publish messages
    int message_count = 0;
    while (true) {
        scoped_lock<interprocess_mutex> lock(buffer->mutex);
        while (buffer->queue.size() == buffer->buffer_size) {
            // Wait for space to become available
            buffer->cond_write.wait(lock);
        }

        // Add a message to the buffer
        buffer->queue.push_back(message_count);

        // Notify consumers
        buffer->cond_read.notify_one();
        std::cout << "Published " << message_count << std::endl;
        message_count++;
    }

    return 0;
}
