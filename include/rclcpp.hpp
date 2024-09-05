#ifndef RCLCPP_HPP_
#define RCLCPP_HPP_

#include <map>
#include <queue>

#include "executor.hpp"

namespace rclcpp {

void init(int argc, char** argv) {
    // Initialize some sort // Brocast the node over the next
    return;
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
