#ifndef EXECUTOR_HPP_
#define EXECUTOR_HPP_

#include <any>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "executor_base.hpp"
#include "node.hpp"

namespace rclcpp {
/*
Each executor maintains an event queue that stores pointers to subscriptions that have new data coming in.
*/

class SingleThreadedExecutor : public ExecutorBase {
    std::thread main_thread;
    std::vector<std::shared_ptr<Timer>> timers;

   public:
    SingleThreadedExecutor() {
        main_thread = std::thread{
            [&]() {
                while (true) {
                    // Iterate over timers, make sure to call the ones that need to be called
                    auto now = std::chrono::steady_clock::now();
                    for (auto timer : timers) {
                        if (timer->next_wake_up < now) {
                            timer->execute_callback_async();
                            timer->next_wake_up = now + timer->interval;
                        }
                    }

                    auto next_wake_up = now + std::chrono::hours(24);
                    for (auto timer : timers) {
                        next_wake_up = min(next_wake_up, timer->next_wake_up);
                    }

                    std::unique_lock lock(event_mutex);
                    cv.wait_until(lock, next_wake_up, [&] { return !event_queue.empty(); });

                    while (!event_queue.empty()) {
                        auto subscription = event_queue.front();
                        event_queue.pop();
                        subscription->execute_callback();
                    }
                }
            }};
    }
    void add_node(std::shared_ptr<Node> node) {
        // register the nodes subscriptions with the RMW
        for (auto sub : node->subscriptions) {
            std::string memory_map = RMW::register_subscription(sub, std::bind(&SingleThreadedExecutor::notify, this, std::placeholders::_1));
            ipc_table[memory_map] = sub;
            this->subscriptions.push_back(sub);
        }

        for (auto timer : node->timers) {
            timers.push_back(timer);
        }
    }

    void notify(std::shared_ptr<SubscriptionBase> sub) {
        std::unique_lock lock(event_mutex);
        event_queue.push(sub);
        cv.notify_one();
    }

    void spin() {
        std::thread ipc{[this]() {
            while (true) {
                update_ipc_queues();
            }
        }};
        ipc.join();
        main_thread.join();
    }
};

class MultithreadedExecutor : public ExecutorBase {
    // https://docs.ros.org/en/foxy/Concepts/About-Executors.html
    std::vector<std::thread> threads;

   public:
    void notify(std::shared_ptr<SubscriptionBase> sub) {
        // In the multithreaded case, the event queue is not used
        return;
    }

    void add_node(std::shared_ptr<Node> node) {
        for (auto sub : node->subscriptions) {
            RMW::register_subscription(sub, std::bind(&MultithreadedExecutor::notify, this, std::placeholders::_1));
            this->subscriptions.push_back(sub);
        }

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
                    timer->execute_callback();
                }
            }});
        }
        // Busy thread that checks shared memory. TODO: Change to event-driven
        threads.push_back(std::thread{[this]() {
            while (true) {
                update_ipc_queues();
            }
        }});
    }
    void spin() {
        for (auto& thread : threads) {
            thread.join();
        }
    }
};

}  // namespace rclcpp

#endif
