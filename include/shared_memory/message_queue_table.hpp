#ifndef MESSAGE_QUEUE_TABLE_HPP_
#define MESSAGE_QUEUE_TABLE_HPP_
// Table that stores topic_names to vectors of memory regions
// This memory region is designated for a particular subscription

// https://stackoverflow.com/questions/12413034/shared-map-with-boostinterprocess
#include <boost/interprocess/containers/map.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <utility>

using namespace boost::interprocess;

struct MessageQueueTable {
    typedef managed_shared_memory::segment_manager SegmentManager;
    typedef allocator<void, SegmentManager> VoidAllocator;
    // Shared memory allocators
    typedef basic_string<char, std::char_traits<char>, allocator<char, SegmentManager>> ShmemString;
    typedef vector<ShmemString, allocator<ShmemString, SegmentManager>> ShmemStringVector;
    // The map will have ShmemString as the key and ShmemStringVector as the value
    typedef std::pair<const ShmemString, ShmemStringVector> ValueType;
    // Allocator for the map
    typedef allocator<ValueType, SegmentManager> ShmemAllocator;
    // STL-compatible map using shared memory
    typedef map<ShmemString, ShmemStringVector, std::less<ShmemString>, ShmemAllocator> MySHMMap;

    // Map pointer stored in shared memory
    offset_ptr<MySHMMap> m_pmap;
    interprocess_mutex mutex;  // to prevent multiple reads

    // Constructor to initialize the shared memory allocator and map
    MessageQueueTable(managed_shared_memory &segment) {
        // Get the allocator instance
        ShmemAllocator alloc_inst(segment.get_segment_manager());
        m_pmap = segment.find_or_construct<MySHMMap>("message_queue_table")(std::less<ShmemString>(), alloc_inst);
    }

    // Method to insert or update a topic_name and its associated vector of memory region names
    void insert_or_update(const std::string &topic_name, const std::vector<std::string> &memory_region_names, managed_shared_memory &segment) {
        // Create shared memory string using the allocator
        ShmemString shm_topic_name(topic_name.c_str(), segment.get_segment_manager());

        // Create a shared memory vector of strings
        allocator<ShmemString, SegmentManager> string_alloc(segment.get_segment_manager());
        ShmemStringVector shm_memory_region_vector(string_alloc);

        // Populate the vector with the provided memory region names
        for (const auto &name : memory_region_names) {
            shm_memory_region_vector.emplace_back(name.c_str(), segment.get_segment_manager());
        }

        // Try to find the existing entry
        auto it = m_pmap->find(shm_topic_name);

        // If the topic_name already exists, update the vector; otherwise, insert a new pair
        if (it != m_pmap->end()) {  // Update value
            it->second = shm_memory_region_vector;
        } else {  // Insert value
            m_pmap->emplace(std::make_pair(std::move(shm_topic_name), std::move(shm_memory_region_vector)));
        }
    }

    const ShmemStringVector *get_memory_region(const std::string &topic_name, managed_shared_memory &segment) {
        ShmemString shm_topic_name(topic_name.c_str(), segment.get_segment_manager());
        auto it = m_pmap->find(shm_topic_name);

        if (it != m_pmap->end()) {
            return &it->second;
        }

        return nullptr;
    }
};

#endif
