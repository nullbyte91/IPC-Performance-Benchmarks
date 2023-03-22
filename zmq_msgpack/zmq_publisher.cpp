#include "image.hpp"
#include <chrono>   
#include <opencv2/opencv.hpp>

int main()
{
    zmq::context_t ctx{1};
    zmq::socket_t server{ctx, zmq::socket_type::rep};
    server.bind("tcp://127.0.0.1:5555");

    // Create example OpenCV matrices (images)
    cv::Mat frame_1 = cv::Mat::eye(480, 640, CV_8UC3);
    cv::Mat frame_2 = cv::Mat::eye(1280, 960, CV_8UC3);
    cv::Mat frame_3 = cv::Mat::eye(1800, 1200, CV_8UC3);
    cv::Mat frame_4 = cv::Mat::eye(2100, 1500, CV_8UC3);
    
    int count{0}, global_count{0};
    cv::Mat frame;
    while (true) {
        // Assign the appropriate frame depending on the value of count
        if (count == 0) {
            frame = frame_1;
            count += 1;
        } else if (count == 1) {
            frame = frame_2;
            count += 1;
        } else if (count == 2) {
            frame = frame_3;
            count += 1;
        } else {
            frame = frame_4;
            count = 0;
        }
        // open for requests 
        std::cout << "open for request..." << std::endl;
        zmq::message_t req;
        server.recv(req, zmq::recv_flags::none);
        std::cout << "[⚡ got request] => " << req.to_string() << std::endl;

        // prepare data
        Image img_data;
        img_data.matrix = std::vector<uchar>(frame.data, frame.data + (frame.rows * frame.cols * frame.channels()));
        img_data.rows = frame.rows;
        img_data.cols = frame.cols;
        img_data.type = frame.type();

        // packed/serialize the data using msgpack
        msgpack::sbuffer serialized_img;
        msgpack::pack(&serialized_img, img_data);
        std::cout << "[⚓ Publish image: " << frame.rows << " * " << frame.cols << ": " << global_count++ << "] =>"<< std::endl;
        // send the serialized data as the response to client
        zmq::message_t packed_msg(serialized_img.size());
        std::memcpy(packed_msg.data(), serialized_img.data(), serialized_img.size());
        server.send(packed_msg, zmq::send_flags::none);
        
        // Terminate the loop after 2000 iterations
        if (global_count == 2000) {
            exit(1);
        }
    }

    return EXIT_SUCCESS;
}