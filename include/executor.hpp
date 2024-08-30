#ifndef EXECUTOR_HPP_
#define EXECUTOR_HPP_

#include <thread>

#include "node.hpp"

namespace rclcpp {

class SingleThreadedExecutor {
   public:
    void add_node(std::shared_ptr<Node> node) {
        for (auto sub : node->subscriptions) {
        }
        for (auto timer : node->timers) {
        }
    }
    void spin() {
    }
};

class MultithreadedExecutor {
    // https://docs.ros.org/en/foxy/Concepts/About-Executors.html
    std::vector<std::thread> threads;
    std::vector<std::shared_ptr<Node>> nodes;

   public:
    void add_node(std::shared_ptr<Node> node) {
        nodes.push_back(node);

        for (auto sub : node->subscriptions) {
            threads.push_back(std::thread{
                [sub]() {
                    while (true) {
                        sub->execute_callback();
                    } }});
        }

        for (auto timer : node->timers) {
            threads.push_back(std::thread{[timer]() {
                while (true) {
                    // std::cout << "busy waiting" << std::endl;
                    timer->execute_callback();
                }
            }});
        }
    }

    void spin() {
        for (auto& thread : threads) {
            thread.join();
        }
    }
};

}  // namespace rclcpp

#endif
