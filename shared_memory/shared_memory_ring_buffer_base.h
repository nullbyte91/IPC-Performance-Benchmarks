#ifndef SHARED_MEMORY_RING_BUFFER_BASE_H 
#define SHARED_MEMORY_RING_BUFFER_BASE_H 

#include <vector>

#include <boost/lockfree/spsc_queue.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/containers/string.hpp>

#include <opencv2/opencv.hpp>

namespace bip = boost::interprocess;
namespace bc = boost::container;
namespace shm
{
    using Mem = bip::managed_shared_memory;
    using Segment = Mem::segment_manager;

    template <typename T> using Alloc = bip::allocator<T, Segment>;
    template <typename T, int cap> using Queue = boost::lockfree::spsc_queue<T, boost::lockfree::capacity<cap>  >;
    using Buffer = std::vector<uchar, Alloc<uchar> >;
    using BufferQueue = Queue<Buffer, 1024>;

    typedef struct MatHeader_ {
        int cols;
        int rows;
        size_t step;
        size_t elemSize;
        size_t elemType;

        MatHeader_(int cols, int rows, size_t step, size_t elemSize, size_t elemType): 
            cols(cols), rows(rows), step(step), elemSize(elemSize), elemType(elemType)
        {}

        static size_t size() {
            return sizeof(MatHeader_);
        }

        size_t dataSize() {
            return rows * step;
        }

        size_t totalSize() {
            return dataSize() + size();
        }
    } MatHeader;
}

#endif // SHARED_MEMORY_RING_BUFFER_BASE_H 