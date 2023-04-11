#include <iostream>
#include <hiredis/hiredis.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "image.pb.h"
#include "utils.hpp"

#define MSG_SIZE_TEST 10000

class RedisImagePublisher {
public:
    RedisImagePublisher(const std::string& host, int port)
        : host_(host), port_(port), context_(nullptr) {
        connect();
    }

    ~RedisImagePublisher() {
        if (context_) {
            redisFree(context_);
        }
    }

    void connect() {
        context_ = redisConnect(host_.c_str(), port_);
        if (context_ == nullptr || context_->err) {
            if (context_) {
                std::cerr << "Error connecting to Redis: " << context_->errstr << std::endl;
                redisFree(context_);
            } else {
                std::cerr << "Can't allocate Redis context" << std::endl;
            }
            throw std::runtime_error("Cannot connect to Redis");
        }
    }

    void serialize_image_frame(std::string &serialized, const cv::Mat& frame) {
        trodes::proto::Image img_data;
        img_data.set_rows(frame.rows);
        img_data.set_cols(frame.cols);
        img_data.set_elt_type(frame.type());
        img_data.set_elt_size((int)frame.elemSize());
        size_t dataSize = frame.rows * frame.cols * frame.elemSize();
        img_data.set_mat_data(frame.data, dataSize);
        img_data.set_timestamp(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count());
        if (!img_data.SerializeToString(&serialized)) {
            throw std::runtime_error("Error serializing message");
        }
    }

    void publish_image(const cv::Mat& frame, int global_count) {
        std::string serialized;
        timeMeasurement_.measure_time<void>(std::bind(&RedisImagePublisher::serialize_image_frame, this, std::ref(serialized), std::ref(frame)));

        std::cout << "Publish Image : " << frame.rows << " * " << frame.cols << ": " << global_count << std::endl;
        redisReply *reply = (redisReply *)redisCommand(context_, "PUBLISH image_channel %b", serialized.data(), serialized.size());
        if (reply == nullptr) {
            std::cerr << "Error publishing message to Redis: " << context_->errstr << std::endl;
            throw std::runtime_error("Cannot publish image to Redis");
        }
        freeReplyObject(reply);
    }

    double mean_serialization_time(int num_messages) {
        return timeMeasurement_.mean_time(num_messages);
    }

private:
    std::string host_;
    int port_;
    redisContext* context_;
    TimeMeasurement timeMeasurement_;
};

int main() {

    // Create a RedisImagePublisher instance and connect to the Redis server
    RedisImagePublisher publisher("127.0.0.1", 6379);

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

        try {
            publisher.publish_image(frame, global_count);
        } catch (std::runtime_error& e) {
            std::cerr << "Error: " << e.what() << std::endl;
            break;
        }

        global_count++;

        // Terminate the loop after 2000 iterations
        if (global_count == MSG_SIZE_TEST) {
            double ser_time = publisher.mean_serialization_time(MSG_SIZE_TEST - 1);
            std::cout << "Serialization Mean time: " << NS_TO_MS(ser_time) << " ms" << std::endl;
            exit(1);
        }
    }

    return EXIT_SUCCESS;
}

