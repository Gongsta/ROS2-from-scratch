#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <cstring>
#include <iostream>
#include <thread>

#include "shared_memory/shared_message_queue.hpp"

using namespace boost::interprocess;

int main() {
    try {
        // Open the existing shared memory segment
        managed_shared_memory segment(open_only, "SharedMemory");

        SharedMessageQueue<int> *buffer = segment.find<SharedMessageQueue<int>>("Buffer").first;

        // Continuously receive messages
        while (true) {
            scoped_lock<interprocess_mutex> lock(buffer->mutex);
            while (buffer->queue.empty()) {
                // Wait for a message to be available
                buffer->cond_read.wait(lock);
            }

            // Read a message from the buffer
            int message = buffer->queue[0];
            buffer->queue.pop_front();

            std::cout << "Received message: " << message << std::endl;

            // Notify producers that space is available
            buffer->cond_write.notify_one();
        }
    } catch (interprocess_exception &ex) {
        std::cout << "Error: " << ex.what() << std::endl;
        return 1;
    }

    return 0;
}
