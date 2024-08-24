# ROS2-from-scratch
A header-only implementation of ROS in C++ to illustrate the key underworkings of `rclcpp` under the hood. No `colcon`,
no software bloat, just very basics to get ROS nodes communicating with each other.


Concepts that are covered
- Process vs. thread (intra-process vs. inter-process comms)
- Queue size
- Message IDL (serialization) (Why do we need to use ROSIDL?? Can't we just define a class and send it over?)
- Callbacks, handling multiple callbacks, deadlocks
- Thread-safe queues for sending messages

More advanced (maybe I cover):
- Quality of Service
- Callback groups
- Performance
- Type Adaptation (bring up experience using this NVIDIA)

API is made to be the same, with minimum complexity.

We're also going to see how this applies for other languages like Python.

Removing colcon as a build system. You should have CMake installed.

Usage

```
mkdir build
cd build
cmake ..
make
```

And you can

Questions:
- Getting to work out of the box over the network
