
#ifndef TIMER_HPP_
#define TIMER_HPP_

#include <chrono>
#include <functional>
#include <thread>

namespace rclcpp {

class Timer {
   private:
    std::function<void()> callback_;

   public:
    using Milliseconds = std::chrono::duration<int64_t, std::milli>;

    Milliseconds interval;
    std::chrono::time_point<std::chrono::steady_clock> next_wake_up = std::chrono::steady_clock::now();  // Set a far future time initially
    Timer(std::chrono::duration<int64_t, std::milli> interval, std::function<void()> callback) : interval{interval}, callback_{callback} {}
    void execute_callback() {
        std::this_thread::sleep_for(interval);
        callback_();
    }
    void execute_callback_async() {
        callback_();
    }
};

}  // namespace rclcpp

#endif
