#ifndef SHARED_MESSAGE_QUEUE_HPP_
#define SHARED_MESSAGE_QUEUE_HPP_

#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/deque.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

typedef boost::interprocess::managed_shared_memory::segment_manager segment_manager_t;

template <typename MessageType>
struct SharedMessageQueue {
    typedef boost::interprocess::allocator<MessageType, segment_manager_t> ShmemAllocator;
    typedef boost::interprocess::deque<MessageType, ShmemAllocator> SharedQueue;

    int buffer_size;
    boost::interprocess::interprocess_mutex mutex;           // Mutex for synchronization
    boost::interprocess::interprocess_condition cond_read;   // Condition to signal data available
    boost::interprocess::interprocess_condition cond_write;  // Condition to signal space available
    SharedQueue queue;                                       // Dynamic buffer in shared memory

    // Constructor to initialize buffer state
    SharedMessageQueue(const ShmemAllocator& alloc, int buffer_size) : buffer_size{buffer_size}, queue{alloc} {}
};

#endif
