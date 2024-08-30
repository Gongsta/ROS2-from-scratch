# ROS2-from-scratch
A *header-only* implementation of [Robot Operating System (ROS2)](https://docs.ros.org/en/rolling/index.html) in C++ to illustrate the key underworkings of `rclcpp` under the hood. No `colcon`, no software bloat, just very basics to get ROS nodes communicating with each other in a pub-sub architecture.

**NOTE that this is an EXTREMELY simplified version of ROS2.** It does not call the `rcl` api and has a very basic `rmw` implementation.

**What this repository implements:**
- A base `rclcpp::Node` class that enables *pub-sub communication* via `this->create_publisher` and `this->create_subscription` methods
- Event-driven architecture using `std::condition_variable`s on thread-safe queues (`MessageQueue`) to enable efficient execution of callbacks in a multithreaded environment
    - Flexible executors (`SingleThreadedExecutor` and `MultithreadedExecutor`) that can schedule multiple nodes in the same process
- Flexible inter-process (multiprocessing) and intra-process (multithreading) capabilities for execution of `rclcpp::Node`s
- Message serialization using Protobuf for inter-process communication between nodes


Concepts that are covered
- Process vs. thread (intra-process vs. inter-process comms)
    - Choosing to multithread by default as opposed to multiprocess to minimize overhead
- Queue size
- Callbacks, handling multiple callbacks, deadlocks
- Thread-safe queues for sending messages

Because nodes are all in the same process, serialization is not implemented. Message IDL (serialization) (Why do we need to use ROSIDL?? Can't we just define a class and send it over?)

What isn't implemented:
- Quality of Service
- Callback groups
- Type Adaptation (bring up experience using this NVIDIA)
- ROS Services
- rest of ROS 2 features


### Usage
```
mkdir build
cd build
cmake ..
make
```

### Examples

#### Two nodes that communicate with each other in the same process (multithreaded)
```
./hello_world_mt
```

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
- These `Publisher` and `Subscriber` objects are registered with the "`rmw`" layer in a lookup table mapped to topic names
- Each `Subscriber` owns a queue of messages that is filled by the publisher
- At runtime, `Executor` is responsible for scheduling the callbacks of each node
