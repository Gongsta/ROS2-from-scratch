#include <condition_variable>
#include <iostream>
#include <mutex>
#include <thread>

int buffer[10];
int idx = 0;
std::mutex m;
std::condition_variable cv;

void producer() {
  while (true) {
    std::unique_lock<std::mutex> lock(m);
    cv.wait(lock, [] { return idx < 10; }); // Wait until buffer isn't full
    buffer[idx] = rand() % 100;
    std::cout << "Produced: " << buffer[idx] << std::endl;
    idx++;
    lock.unlock();
    cv.notify_one(); // Notify consumer
  }
}

void consumer() {
  while (true) {
    std::unique_lock<std::mutex> lock(m);
    cv.wait(lock, [] { return idx > 0; }); // Wait until buffer isn't empty
    idx--;
    std::cout << "Consumed: " << buffer[idx] << std::endl;
    lock.unlock();
    cv.notify_one(); // Notify producer
  }
}

int main() {
  std::thread t1(producer);
  std::thread t2(consumer);
  t1.join();
  t2.join();
}
