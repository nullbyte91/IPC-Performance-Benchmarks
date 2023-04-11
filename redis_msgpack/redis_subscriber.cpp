#include <iostream>
#include <msgpack.hpp>
#include <hiredis/hiredis.h>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "utils.hpp"

#define MSG_SIZE_TEST 9000

std::chrono::high_resolution_clock::time_point chrono_time;

struct Image {
    std::vector<uchar> matrix;
    int rows{0};
    int cols{0};
    int type{0};
    std::chrono::high_resolution_clock::time_point timestamp;
    MSGPACK_DEFINE(matrix, rows, cols, type, timestamp);
};

class RedisImageSubscriber {
public:
    RedisImageSubscriber(const std::string &ip_address, int port)
        : ip_address_(ip_address), port_(port) {
        connect_to_redis();
    }

    ~RedisImageSubscriber() {
        if (redis_context_) {
            redisFree(redis_context_);
        }
    }

    void connect_to_redis() {
        redis_context_ = redisConnect(ip_address_.c_str(), port_);

        if (redis_context_ == NULL || redis_context_->err) {
            if (redis_context_) {
                std::cerr << "Error connecting to Redis: " << redis_context_->errstr << std::endl;
                redisFree(redis_context_);
            } else {
                std::cerr << "Can't allocate Redis context" << std::endl;
            }
            throw std::runtime_error("Failed to connect to Redis server");
        }
    }

    void subscribe(const std::string &channel_name) {
        redisReply *reply = (redisReply *)redisCommand(redis_context_, "SUBSCRIBE %s", channel_name.c_str());
        if (reply == NULL || reply->type == REDIS_REPLY_ERROR) {
            std::cerr << "Error subscribing to channel: " << redis_context_->errstr << std::endl;
            throw std::runtime_error("Failed to subscribe to Redis channel");
        }
        freeReplyObject(reply);
    }

    redisReply *get_message() {
        redisReply *reply;
        if (redisGetReply(redis_context_, (void **)&reply) == REDIS_OK) {
            return reply;
        } else {
            return nullptr;
        }
    }

    cv::Mat deserialize_image(redisReply* reply) {
        msgpack::unpacked unpacked_data;
        msgpack::unpack(unpacked_data, reply->element[2]->str, reply->element[2]->len);
        Image img_data;

        try {
            img_data = unpacked_data.get().as<Image>();
            chrono_time = img_data.timestamp;
        } catch (const std::exception &e) {
            std::cerr << "Error deserializing message: " << e.what() << std::endl;
        }

        return cv::Mat(img_data.rows, img_data.cols, img_data.type, img_data.matrix.data());
    }

private:
    std::string ip_address_;
    int port_;
    redisContext *redis_context_{nullptr};
};

int main() {
    TimeMeasurement timeMeasurement_;
    std::atomic<bool> exit_flag_{false};
    size_t global_count{0};
    std::chrono::nanoseconds::rep time_diff_ns_{0};

    // Create a RedisImageSubscriber instance and connect to the Redis server
    RedisImageSubscriber subscriber("127.0.0.1", 6379);
    subscriber.subscribe("image_channel");

    redisReply *reply;
    while ((reply = subscriber.get_message()) != nullptr) {
        if (reply->type == REDIS_REPLY_ERROR) {
            std::cerr << "Error receiving message from Redis: " << reply->str << std::endl;
            break;
        }

        if (exit_flag_) {
            double latency = static_cast<double>(time_diff_ns_) / global_count;
            std::cout << "Latency: " << NS_TO_MS(latency) << " ms" << std::endl;
            double deser_time = timeMeasurement_.mean_time(MSG_SIZE_TEST);
            std::cout << "Deserialization Mean time: " << NS_TO_MS(deser_time) << " ms" << std::endl;
            break;
        }

        auto current_chrono_time = std::chrono::high_resolution_clock::now();

        if (reply->type == REDIS_REPLY_ARRAY && reply->elements == 3 &&
            std::string(reply->element[0]->str) == "message") {
            auto time_diff = current_chrono_time - chrono_time;
            time_diff_ns_ += std::chrono::duration_cast<std::chrono::nanoseconds>(time_diff).count();

            cv::Mat frame = timeMeasurement_.measure_time<cv::Mat>(std::bind(&RedisImageSubscriber::deserialize_image, &subscriber, std::ref(reply)));

            std::cout << "Received image: " << frame.rows << " * " << frame.cols << ": " << global_count++ << std::endl;

            if (global_count == MSG_SIZE_TEST) {
                exit_flag_ = true;
            }
        }

        freeReplyObject(reply);
    }

    return EXIT_SUCCESS;
}

