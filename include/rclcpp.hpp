#ifndef RCLCPP_HPP_
#define RCLCPP_HPP_

#include <map>
#include <queue>

#include "executor.hpp"

namespace rclcpp {

void init(int argc, char** argv) {
    // We need some sort of ROS master, so that we know how to synchronize between threads
    // Need to do multiprocessing, or should i just do intraprocess for now?
    // zerorpc
    return;
}

void spin(std::shared_ptr<Node> node) {
    MultithreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
}

void shutdown() {
    return;
}
}  // namespace rclcpp

#endif
