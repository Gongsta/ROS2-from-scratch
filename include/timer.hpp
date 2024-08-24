
#ifndef TIMER_HPP_
#define TIMER_HPP_

#include <chrono>
#include <functional>

namespace rclcpp {

class Timer {
   public:
    using Milliseconds = std::chrono::duration<int64_t, std::milli>;
    using TimePoint = std::chrono::time_point<std::chrono::system_clock>;

    Milliseconds interval;
    TimePoint start_;
    std::function<void()> callback;
    Timer(std::chrono::duration<int64_t, std::milli> interval, std::function<void()> callback) : interval{interval}, start_{std::chrono::system_clock::now()}, callback{callback} {}
    bool tick() {
        TimePoint now = std::chrono::system_clock::now();

        // Calculate the elapsed time in milliseconds
        auto elapsed = std::chrono::duration_cast<Milliseconds>(now - start_);

        if (elapsed >= interval) {
            start_ = now;
            return true;
        }
        return false;
    }
};

}  // namespace rclcpp

#endif
