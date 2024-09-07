#include <boost/interprocess/containers/map.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <functional>
#include <iostream>
#include <utility>

#include "shared_memory/message_queue_table.hpp"

int main() {
    // Create or open shared memory
    managed_shared_memory segment(open_or_create, "shared_message_queue_table", 65536);
    MessageQueueTable mq_table(segment);

    while (true) {
        std::string topic_name;
        std::cout << "Enter topic name: ";
        std::cin >> topic_name;

        std::vector<std::string> memory_region_names;
        std::string region_name;
        std::cout << "Enter memory region names (enter 'done' to finish):" << std::endl;
        while (std::cin >> region_name && region_name != "done") {
            memory_region_names.push_back(region_name);
        }

        // Insert or update a topic and its associated vector of memory region names into the table
        mq_table.insert_or_update(topic_name, memory_region_names, segment);

        const auto *retrieved_memory_names = mq_table.get_memory_region(topic_name, segment);

        if (retrieved_memory_names) {
            std::cout << "Memory regions for topic '" << topic_name << "' found: ";
            for (const auto &name : *retrieved_memory_names) {
                std::cout << name << " ";
            }
            std::cout << std::endl;
        } else {
            std::cout << "Memory regions for topic '" << topic_name << "' not found." << std::endl;
        }
    }

    return 0;
}
