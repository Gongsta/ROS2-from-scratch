/*
Example to show that we can achieve these same ROS functionalities by implementing 2 simple threads.
*/
#include <condition_variable>
#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

using namespace std;

queue<string> q;
mutex m_mutex;
condition_variable cv;
int counter = 0;

void publisher_loop() {
    while (true) {
        std::unique_lock lock(m_mutex);
        q.push("hello world" + to_string(counter));
        counter++;
        cv.notify_one();
    }
}

void subscription_loop() {
    while (true) {
        std::unique_lock lock(m_mutex);
        cv.wait(lock, []() { return !q.empty(); });
        std::string data = q.front();
        q.pop();
        cout << "Received " << data << endl;
    }
}

int main() {
    thread t1(publisher_loop);
    thread t2(subscription_loop);
    t1.join();
    t2.join();
}
