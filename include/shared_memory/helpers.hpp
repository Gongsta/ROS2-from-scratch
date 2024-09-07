#ifndef SM_HELPERS_HPP_
#define SM_HELPERS_HPP_
#include <boost/interprocess/managed_shared_memory.hpp>
#include <string>

#include "shared_memory/shared_message_queue.hpp"

std::string gen_random(const int len) {
    static const char alphanum[] =
        "0123456789"
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz";
    std::string tmp_s;
    tmp_s.reserve(len);

    for (int i = 0; i < len; ++i) {
        tmp_s += alphanum[rand() % (sizeof(alphanum) - 1)];
    }

    return tmp_s;
}

std::string generate_unique_shared_memory_segment() {
    while (true) {
        std::string name = gen_random(12);
        try {
            // Attempt to open the shared memory segment
            // This will not create the segment, only open it if it exists
            boost::interprocess::managed_shared_memory segment(boost::interprocess::open_only, name.c_str());
        } catch (boost::interprocess::interprocess_exception& e) {
            // If an exception is thrown, it means the shared memory segment does not exist
            return name;
        }
    }
}

void create_lookup_table() {
    // Create a new shared memory segment
    boost::interprocess::managed_shared_memory segment(boost::interprocess::create_only, "NodeLookup", 65536);

    // Create an allocator for shared memory
    boost::interprocess::allocator<int, segment_manager_t> alloc_inst(segment.get_segment_manager());

    // Construct the shared memory buffer in shared memory
    int buffer_size = 10;  // Maximum size of the queue
    SharedMessageQueue<int>* buffer = segment.construct<SharedMessageQueue<int>>("Buffer")(alloc_inst, buffer_size);

    // Continuously publish messages
    int message_count = 0;
    while (true) {
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(buffer->mutex);
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
}

#endif
