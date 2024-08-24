cmake_minimum_required(VERSION 3.5.1)
project(ros2)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g")

# /include folder
include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(${PROJECT_NAME} INTERFACE)
# add_library(${PROJECT_NAME} SHARED
    # src/node.cpp
# )

add_executable(sub examples/hello_world_subscription.cpp)
add_executable(pub examples/hello_world_publisher.cpp)

target_link_libraries(sub ${PROJECT_NAME})
target_link_libraries(pub ${PROJECT_NAME})
