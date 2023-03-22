#include "shared_memory_ring_buffer_base.h"

#include <boost/thread/thread.hpp>
#include <boost/random.hpp>

int main() {
    // Create example OpenCV matrices (images)
    cv::Mat frame_1 = cv::Mat::eye(480, 640, CV_8UC3);
    cv::Mat frame_2 = cv::Mat::eye(1280, 960, CV_8UC3);
    cv::Mat frame_3 = cv::Mat::eye(1800, 1200, CV_8UC3);
    cv::Mat frame_4 = cv::Mat::eye(2100, 1500, CV_8UC3);

    // Initialize random number generator
    boost::mt19937 generator;
    boost::uniform_int<> range( 50, 100 );
    boost::variate_generator< boost::mt19937, boost::uniform_int<> > nextRandomValue(generator, range);

    // Remove the shared memory object to avoid conflicts
    bip::shared_memory_object::remove("MySharedMemory");

    // Create a managed shared memory segment with double the size
    bip::managed_shared_memory segment(bip::open_or_create, "MySharedMemory", 65536665 * 5);

    // Find or construct a BufferQueue in shared memory segment
    auto& queue = *segment.find_or_construct<shm::BufferQueue>("queue")();

    int count{0}, global_count{0};
    cv::Mat mat;
    while (true) {
        // Assign different images based on count value
        if (count == 0) {
            mat = frame_1;
            count += 1;
        } else if (count == 1) {
            mat = frame_2;
            count += 1;
        } else if (count == 2) {
            mat = frame_3;
            count += 1;
        } else {
            mat = frame_4;
            count = 0;
        }

        // Create a header for the image to be sent
        shm::MatHeader header(mat.cols, mat.rows, mat.step[0], mat.elemSize(), mat.type());

        // Create a buffer in shared memory to store the image
        auto buf = shm::Buffer(header.totalSize(), segment.get_segment_manager());

        // Copy the header and image data to the buffer
        size_t offset = 0;
        memcpy(buf.data(), (uchar*) &header, shm::MatHeader::size()); offset += shm::MatHeader::size();
        memcpy(buf.data() + offset, mat.data, header.dataSize());

        // Push the buffer to the queue in shared memory
        queue.push(buf);
        std::cout << "[⚓ Publish image: " << mat.rows << " * " << mat.cols << ": " << global_count++ << "] =>"<< std::endl;
        
        // Exit the loop after 2000 iterations
        if (global_count == 2000){
            break;
        }
    }
    
    bip::shared_memory_object::remove("MySharedMemory");
    return EXIT_SUCCESS;
}