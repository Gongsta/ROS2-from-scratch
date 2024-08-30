
#ifndef TIMER_HPP_
#define TIMER_HPP_

#include <chrono>
#include <functional>
#include <thread>

namespace rclcpp {

class Timer {
   public:
    using Milliseconds = std::chrono::duration<int64_t, std::milli>;

    Milliseconds interval;
    std::function<void()> callback;
    Timer(std::chrono::duration<int64_t, std::milli> interval, std::function<void()> callback) : interval{interval}, callback{callback} {}
    void execute_callback() {
        std::this_thread::sleep_for(interval);
        callback();
    }
};

}  // namespace rclcpp

#endif
