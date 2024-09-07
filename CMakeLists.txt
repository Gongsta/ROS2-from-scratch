cmake_minimum_required(VERSION 3.5.1)
project(ros2)

include(FetchContent)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g")

# Include Boost
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# /include folder
include_directories(${PROJECT_SOURCE_DIR}/include)
add_library(${PROJECT_NAME} INTERFACE)

# Trivial Examples
add_executable(basic_mt examples/trivial/basic_multithreading.cpp)
add_executable(publisher examples/trivial/boost_publisher.cpp)
add_executable(subscription examples/trivial/boost_subscription.cpp)
add_executable(map examples/trivial/boost_map_creator.cpp)
add_executable(client examples/trivial/client.cpp)
add_executable(server examples/trivial/server.cpp)

#Intra-process
add_executable(hello_world_intraprocess examples/hello_world_intraprocess.cpp)
add_executable(1000_component examples/1000_node_component.cpp)
# Inter-process
add_executable(hello_world_subscription examples/hello_world_subscription.cpp)
add_executable(hello_world_publisher examples/hello_world_publisher.cpp)

target_link_libraries(${PROJECT_NAME} INTERFACE ${Boost_LIBRARIES})
target_link_libraries(basic_mt ${PROJECT_NAME})
target_link_libraries(publisher ${PROJECT_NAME})
target_link_libraries(subscription ${PROJECT_NAME})
target_link_libraries(map ${PROJECT_NAME})
target_link_libraries(client ${PROJECT_NAME})
target_link_libraries(server ${PROJECT_NAME})
target_link_libraries(hello_world_intraprocess ${PROJECT_NAME})
target_link_libraries(1000_component ${PROJECT_NAME})
target_link_libraries(hello_world_subscription ${PROJECT_NAME})
target_link_libraries(hello_world_publisher ${PROJECT_NAME})
