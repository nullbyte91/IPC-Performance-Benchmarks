#include <iostream>
#include <opencv2/opencv.hpp>
#include "httplib.h"
#include "json.hpp"
#include "utils.hpp"

#define MSG_SIZE_TEST 9998

using namespace httplib;
using json = nlohmann::json;

class ImageReceiver {
public:
    ImageReceiver(const std::string &host, int port)
        : host_(host), port_(port), server_(std::make_shared<Server>()) {
        server_->Post("/send_image", [this](const Request &req, Response &res) {
            this->process_image(req, res);
        });
    }

    void start() {
        server_->listen(host_.c_str(), port_);
    }

private:
    std::string host_;
    int port_;
    std::shared_ptr<Server> server_;
    TimeMeasurement timeMeasurement_;
    std::chrono::nanoseconds::rep time_diff_ns_{0};
    std::chrono::high_resolution_clock::time_point chrono_time;
    size_t global_count{0};

    cv::Mat deserialize_image(const Request &req) {
        auto json_data = req.body;
        json image_json = json::parse(json_data);
        chrono_time = std::chrono::system_clock::time_point(std::chrono::nanoseconds(image_json["timestamp"]));
        std::vector<uchar> decoded_data = image_json["encoded_data"];
        int rows = image_json["rows"];
        int cols = image_json["cols"];
        int type = image_json["type"];

        return cv::Mat(rows, cols, type, decoded_data.data());
    }

    void process_image(const Request &req, Response &res) {
        auto current_chrono_time = std::chrono::high_resolution_clock::now();
        cv::Mat frame = timeMeasurement_.measure_time<cv::Mat>(std::bind(&ImageReceiver::deserialize_image, this, std::ref(req)));

        auto time_diff = current_chrono_time - chrono_time;
        time_diff_ns_ += std::chrono::duration_cast<std::chrono::nanoseconds>(time_diff).count();

        // Process the received image as needed
        std::cout << "Received image: " << frame.rows << " * " << frame.cols << ": " << global_count++ << std::endl;

        if (global_count == MSG_SIZE_TEST) {
            double latency = static_cast<double>(time_diff_ns_) / global_count;
            std::cout << "Latency: " << NS_TO_MS(latency) << " ms" << std::endl;
            double deser_time = timeMeasurement_.mean_time(MSG_SIZE_TEST);
            std::cout << "Deserialization Mean time: " << NS_TO_MS(deser_time) << " ms" << std::endl;
        }

        res.set_content("Image received", "text/plain");
    }
};

int main() {
    ImageReceiver receiver("localhost", 8080);
    receiver.start();
    return 0;
}
