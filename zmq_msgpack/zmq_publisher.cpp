#include <chrono>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <ctime>
#include <iostream>
#include <msgpack.hpp>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <zmq.hpp>
#include <zmq_addon.hpp>

#include "utils.hpp"

#define MSG_SIZE_TEST 10000

struct Image {
    std::vector<uchar> matrix;
    int rows{0};
    int cols{0};
    int type{0};
    std::int64_t timestamp_ns{0};
    MSGPACK_DEFINE(matrix, rows, cols, type, timestamp_ns);
};

class ImagePublisher {
public:
    ImagePublisher(const std::string& address)
        : ctx_(1), server_(ctx_, zmq::socket_type::rep) {
        server_.bind(address);
    }

    void run() {
        int count{0}, global_count{0};
        cv::Mat frame;
        while (true) {
            // Assign the appropriate frame depending on the value of count
            if (count == 0) {
                frame = frame_1_;
                count += 1;
            } else if (count == 1) {
                frame = frame_2_;
                count += 1;
            } else if (count == 2) {
                frame = frame_3_;
                count += 1;
            } else {
                frame = frame_4_;
                count = 0;
            }

            // Open for requests
            std::cout << "open for request..." << std::endl;
            zmq::message_t req;
            server_.recv(req, zmq::recv_flags::none);
            std::cout << "[⚡ got request] => " << req.to_string() << std::endl;

            zmq::message_t packed_msg = timeMeasurement_.measure_time<zmq::message_t>(
                std::bind(&ImagePublisher::serialize_image_frame, this, std::ref(frame)));

            std::cout << "[⚓ Publish image: " << frame.rows << " * " << frame.cols << ": " << global_count++ << "] =>" << std::endl;

            server_.send(packed_msg, zmq::send_flags::none);

            if (global_count == MSG_SIZE_TEST) {
                double ser_time = timeMeasurement_.mean_time(MSG_SIZE_TEST - 1);
                std::cout << "Serialization Mean time: " << NS_TO_MS(ser_time) << " ms" << std::endl;
                exit(1);
            }
        }
    }

private:
    zmq::message_t serialize_image_frame(const cv::Mat& frame) {
        // Prepare data
        Image img_data;

        img_data.matrix = std::vector<uchar>(frame.data, frame.data + (frame.rows * frame.cols * frame.channels()));
        img_data.rows = frame.rows;
        img_data.cols = frame.cols;
        img_data.type = frame.type();
        auto timestamp = std::chrono::high_resolution_clock::now();
        img_data.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(timestamp.time_since_epoch()).count();

        // Packed/serialize the data using msgpack
        msgpack::sbuffer serialized_img;
        msgpack::pack(&serialized_img, img_data);

        // Send the serialized data as the response to client
        zmq::message_t packed_msg(serialized_img.size());
        std::memcpy(packed_msg.data(), serialized_img.data(), serialized_img.size());

        return packed_msg;
    }

    zmq::context_t ctx_;
    zmq::socket_t server_;
    TimeMeasurement timeMeasurement_;

    cv::Mat frame_1_ = cv::Mat::eye(480, 640, CV_8UC3);
    cv::Mat frame_2_ = cv::Mat::eye(1280, 960, CV_8UC3);
    cv::Mat frame_3_ = cv::Mat::eye(1800, 1200, CV_8UC3);
    cv::Mat frame_4_ = cv::Mat::eye(2100, 1500, CV_8UC3);
};

int main() {
    ImagePublisher publisher("tcp://127.0.0.1:5555");
    publisher.run();

    return EXIT_SUCCESS;
}
