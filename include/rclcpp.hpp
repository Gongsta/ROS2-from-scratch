#ifndef RCLCPP_HPP_
#define RCLCPP_HPP_

#include <map>
#include <queue>

#include "executor.hpp"
#include "shared_memory/helpers.hpp"

namespace rclcpp {

void init(int argc, char** argv) {
    // Initialize some sort // Brocast the node over the next
    // If there is another active subscriber, don't delete
    shared_memory_object::remove("shared_message_queue_table");
}

void spin(std::shared_ptr<Node> node) {
    SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
}

void shutdown() {
    return;
}
}  // namespace rclcpp

#endif
