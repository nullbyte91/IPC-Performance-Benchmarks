#include "shared_memory_ring_buffer_base.h"

#include <iostream>
#include <boost/thread/thread.hpp>
#include <boost/random.hpp>
#include <opencv2/opencv.hpp>

int main() {
    int global_count{0};

    // Initialize random number generator
    boost::mt19937 generator;
    boost::uniform_int<> range( 50, 100 );
    boost::variate_generator< boost::mt19937, boost::uniform_int<> > nextRandomValue(generator, range);

    // Create a managed shared memory segment
    bip::managed_shared_memory segment(bip::open_or_create, "MySharedMemory", 65536);

    // Find or construct a BufferQueue in shared memory segment
    auto& queue = *segment.find_or_construct<shm::BufferQueue>("queue")();

    while (true) {
        // Create a buffer object in shared memory segment
        shm::Buffer v(segment.get_segment_manager());

        // Pop the buffer from the queue if available
        if (queue.pop(v)) {
            size_t offset = 0;
            
            // Get the MatHeader from the buffer data
            shm::MatHeader& header = *((shm::MatHeader*) v.data()); offset += shm::MatHeader::size();

            // Create a cv::Mat object using the header information and data
            cv::Mat mat(header.rows, header.cols, header.elemType, v.data() + offset, header.step);
        
            // Print the size of the consumed image
            std::cout << "[⚓ Received image: " << header.rows << " * " << header.cols << ": " << global_count++ << "] =>"<< std::endl;
        }
    }
    return EXIT_SUCCESS; 
}