# ROS2-from-scratch
A header-only implementation of [Robot Operating System (ROS2)](https://docs.ros.org/en/rolling/index.html) in C++ to illustrate the key underworkings of `rclcpp` under the hood. No `colcon`, no software bloat, just very basics to get ROS nodes communicating with each other in a pub-sub architecture, built with CMake.

**NOTE that this is an EXTREMELY simplified version of ROS2.** It does not call the `rcl` api and has a very basic `rmw` implementation.

**What this repository implements:**
- A base `rclcpp::Node` class that enables *pub-sub communication* via `this->create_publisher` and `this->create_subscription` methods
- Event-driven architecture using `std::condition_variable`s on thread-safe queues (`MessageQueue`) to enable efficient execution of callbacks in a multithreaded environment
    - Flexible executors (`SingleThreadedExecutor` and `MultithreadedExecutor`) that can schedule multiple nodes in the same process
- Flexible inter-process (multiprocessing) and intra-process (multithreading) capabilities for execution of `rclcpp::Node`s
- Inter-process communication between nodes via shared memory using `Boost.Interprocess`
    - Note that because we are using shared memory, I don't IPC between different machines (no socket programming)
    - However, we get the benefit of really good performance thanks to zero-copy data transfer / no serialization


Concepts that are covered
- Process vs. thread (intra-process vs. inter-process comms)
    - Choosing to multithread by default as opposed to multiprocess to minimize overhead
- Queue sizes
- Callbacks, deadlocks
- Thread-safe queues for sending messages
- Because nodes are all in the same process, serialization is not implemented. Message IDL (serialization) (Why do we need to use ROSIDL?? Can't we just define a class and send it over?).

What isn't implemented:
- Even more efficient intra-process comms through zero-copy data transfer (see (https://docs.ros.org/en/eloquent/Tutorials/Intra-Process-Communication.html)[here])
- Quality of Service
- Callback groups
- Type Adaptation (bring up experience using this NVIDIA)
- ROS Services
- rest of ROS 2 features

Todo:
- Overloaded callbacks to enable **zero-copy** data transfer between nodes via `std::unique_ptr` with `std::move` semantics


### Usage
You need Boost installed
On mac
```
brew install boost
```

On linux
```
sudo apt-get install libboost-all-dev
```

Then, build the packages
```
mkdir build
cd build
cmake ..
make
```

### Examples

#### Two nodes that communicate with each other in the same process

```
./hello_world_intraprocess
```

By default, a single thread (`SingleThreadedExecutor`) is used to execute the callbacks of each node. To use multiple threads, uncomment the line in `hello_world_intraprocess.cpp` that creates a `MultithreadedExecutor` object.

#### Two nodes that communicate with each other in different processes
```
./hello_world_publisher
```
In a separate terminal, run
```
./hello_world_subscription
```

#### Stress testing
```
./1000_node_component
```
Spawn 1000 nodes that communicate with each other in the same process. This is to test the efficiency of the executor in scheduling callbacks.


### High-Level Overview of system
- `Node` create `Publisher` and `Subscription` shared pointers every time `create_publisher` and `create_subscription` is called
- The `Subscription` objects are registered with the "`rmw`" layer in a lookup table mapping topic names to `Subscription` objects
- Each `Subscription` owns a queue of messages (thread-safe) that is populated by the `rmw` layer when a new message is received
- At runtime, the `Executor` is responsible for scheduling the callbacks of each node (event-driven)
- When a new message is published, the `RMW` triggers the executor through a condition variable

On executors:
- For `SinglethreadedExecutor`, all subscriptions are registered on an event_queue. When a new message is published, the executor is notified through this event queue

Excerpt From `SinglethreadedExecutor`:
```cpp
// Subscription registration
void add_node(std::shared_ptr<Node> node) {
    for (auto sub : node->subscriptions) {
        RMW::register_subscription(sub, std::bind(&SingleThreadedExecutor::notify, this, std::placeholders::_1));
    }
    for (auto timer : node->timers) {
        timers.push_back(timer);
    }
}
    // "notify" will be called by the RMW
    void notify(std::shared_ptr<SubscriptionBase> sub) {
    std::unique_lock lock(event_mutex);
    event_queue.push(sub);
    cv.notify_one();
}
```

In `RMW`, this is what a registration looks like:
```cpp
// The executor is registered alongside the subscription
void register_subscription(std::shared_ptr<SubscriptionBase> subscription, std::function<void(std::shared_ptr<SubscriptionBase>)> notify) {
    inprocess_subscription_table[subscription->topic_name].push_back(subscription);
    subscription_executor_table[subscription] = notify;
}
```
- In ROS, there are more advanced mechanisms for scheduling callbacks, such as callback groups, but this is a simple implementation

- For `MultithreadedExecutor`, each subscription is associated with a thread, each thread blocked on a condition variable

node -> publisher -> rmw -> executor -> subscriber -> node

### IPC Mechanism through Shared Memory
Background:
- When all nodes are in the same process, they can simply share a lookup table (`unordered_map`) that maps topic names to pointers to subscriptions. When a message is published, the RMW can simply look up the subscription in this table and populate the queue of messages owned by each subscription
- When nodes are in different processes, we need to use shared memory to share this lookup table (`MessageQueueTable`). Queues are also shared.
