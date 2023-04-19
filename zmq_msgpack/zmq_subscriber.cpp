#include <chrono>
#include <iostream>
#include <msgpack.hpp>
#include <opencv2/opencv.hpp>
#include <string>
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

class ImageSubscriber {
public:
    ImageSubscriber(const std::string& server_address)
        : ctx_(1), client_(ctx_, zmq::socket_type::req) {
        client_.connect(server_address);
    }

    cv::Mat deserialize_image(zmq::message_t& response_data) {
        // Deserialize the response data
        msgpack::unpacked unpacked_response_data;
        msgpack::unpack(unpacked_response_data,
                        static_cast<const char*>(response_data.data()),
                        response_data.size());

        auto unpacked_response = unpacked_response_data.get().as<Image>();
        chrono_time_ = std::chrono::high_resolution_clock::time_point(
            std::chrono::nanoseconds(unpacked_response.timestamp_ns));

        return cv::Mat(unpacked_response.rows, unpacked_response.cols,
                       unpacked_response.type, unpacked_response.matrix.data());
    }

    void run() {
        TimeMeasurement timeMeasurement_;
        int global_count{0};
        std::chrono::nanoseconds::rep time_diff_ns_{0};
        const std::string req_data{"asking for cv::Mat"};

        while (true) {
            // Send a request
            std::cout << "sending request... " << std::endl;
            client_.send(zmq::buffer(req_data), zmq::send_flags::none);

            // Get the response data
            zmq::message_t response_data;
            client_.recv(response_data, zmq::recv_flags::none);
            auto current_chrono_time =
                std::chrono::high_resolution_clock::now();
            cv::Mat frame = timeMeasurement_.measure_time<cv::Mat>(
                std::bind(&ImageSubscriber::deserialize_image, this,
                          std::ref(response_data)));

            auto time_diff = current_chrono_time - chrono_time_;
            time_diff_ns_ +=
                std::chrono::duration_cast<std::chrono::nanoseconds>(time_diff)
                    .count();

            std::cout << "[⚓ Received image: " << frame.rows << " * "
                      << frame.cols << ": " << global_count++ << "] =>"
                      << std::endl;

            if (global_count == MSG_SIZE_TEST) {
                double latency =
                    static_cast<double>(time_diff_ns_) / global_count;
                std::cout << "Latency: " << NS_TO_MS(latency) << " ms"
                          << std::endl;
                double deser_time = timeMeasurement_.mean_time(MSG_SIZE_TEST);
                std::cout << "Deserialization Mean time: "
                          << NS_TO_MS(deser_time) << " ms" << std::endl;
            }
        }
    }

private:
    zmq::context_t ctx_;
    zmq::socket_t client_;
    std::chrono::high_resolution_clock::time_point chrono_time_;
};

int main() {
    std::string server_address = "tcp://127.0.0.1:5555";
    ImageSubscriber subscriber(server_address);
    subscriber.run();
    return EXIT_SUCCESS;
}
