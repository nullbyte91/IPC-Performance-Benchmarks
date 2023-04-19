#include <iostream>
#include <opencv2/opencv.hpp>
#include "httplib.h"
#include "json.hpp"
#include "utils.hpp"

#define MSG_SIZE_TEST 10000

using namespace httplib;
using json = nlohmann::json;

class TimeMeasurement; // Forward declaration for TimeMeasurement class

class ImageSender {
public:
    ImageSender(const std::string& host, int port)
        : host_(host), port_(port), client_(host + ":" + std::to_string(port)), timeMeasurement_() {}

    void send_image(const cv::Mat& image) {
        json image_json;
        timeMeasurement_.measure_time<void>([&]() { serialize_image_frame(image_json, image); });
        auto response = client_.Post("/send_image", image_json.dump(), "application/json");
        if (response && response->status == 200) {
            // pass
        } else {
            std::cerr << "Failed to connect to the server." << std::endl;
        }
    }

    double mean_serialization_time(int num_messages) {
        return timeMeasurement_.mean_time(num_messages);
    }

private:
    void serialize_image_frame(json& image_json, const cv::Mat& image) {
        std::vector<uchar> encoded_image;
        cv::imencode(".jpg", image, encoded_image);

        image_json["encoded_data"] = encoded_image;
        image_json["rows"] = image.rows;
        image_json["cols"] = image.cols;
        image_json["type"] = image.type();
        image_json["timestamp"] = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
    }

    std::string host_;
    int port_;
    Client client_;
    TimeMeasurement timeMeasurement_;
};

int main() {
    ImageSender sender("http://localhost", 8080);

    // Create example OpenCV matrices (images)
    cv::Mat frame_1 = cv::Mat::eye(480, 640, CV_8UC3);
    cv::Mat frame_2 = cv::Mat::eye(1280, 960, CV_8UC3);
    cv::Mat frame_3 = cv::Mat::eye(1800, 1200, CV_8UC3);
    cv::Mat frame_4 = cv::Mat::eye(2100, 1500, CV_8UC3);

    int count{0}, global_count{0};

    cv::Mat image;

    while (true) {
        // Assign the appropriate frame depending on the value of count
        if (count == 0) {
            image = frame_1;
            count += 1;
        } else if (count == 1) {
            image = frame_2;
            count += 1;
        } else if (count == 2) {
            image = frame_3;
            count += 1;
        } else {
            image = frame_4;
            count = 0;
        }

        sender.send_image(image);

        std::cout << "Publish Image : " << image.rows << " * " << image.cols << ": " << global_count++ << std::endl;

        // Terminate the loop after 2000 iterations
        if (global_count == MSG_SIZE_TEST) {
            double ser_time = sender.mean_serialization_time(MSG_SIZE_TEST - 1);
            std::cout << "Serialization Mean time: " << NS_TO_MS(ser_time) << " ms" << std::endl;
            exit(1);
        }
    }
    return 0;
}
