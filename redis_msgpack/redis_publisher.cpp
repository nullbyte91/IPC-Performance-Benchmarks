#include <iostream>
#include <msgpack.hpp>
#include <hiredis/hiredis.h>
#include <opencv2/opencv.hpp>

struct Image {
    std::vector<uchar> matrix;
    int rows = 0;
    int cols = 0;
    int type = 0;
    MSGPACK_DEFINE(matrix, rows, cols, type);
};

int main() {
    // Connect to the Redis server
    redisContext *c = redisConnect("127.0.0.1", 6379);

    // Create example OpenCV matrices (images)
    cv::Mat frame_1 = cv::Mat::eye(480, 640, CV_8UC3);
    cv::Mat frame_2 = cv::Mat::eye(1280, 960, CV_8UC3);
    cv::Mat frame_3 = cv::Mat::eye(1800, 1200, CV_8UC3);
    cv::Mat frame_4 = cv::Mat::eye(2100, 1500, CV_8UC3);

    // Check if the connection to the Redis server is successful
    if (c == NULL || c->err) {
        if (c) {
            std::cerr << "Error connecting to Redis: " << c->errstr << std::endl;
            redisFree(c);
        } else {
            std::cerr << "Can't allocate Redis context" << std::endl;
        }
        return EXIT_FAILURE;
    }

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

        // Create an Image struct and fill it with the data from the current frame
        Image img_data;
        img_data.matrix = std::vector<uchar>(frame.data, frame.data + (frame.rows * frame.cols * frame.channels()));
        img_data.rows = frame.rows;
        img_data.cols = frame.cols;
        img_data.type = frame.type();

        // Serialize the Image struct using msgpack
        msgpack::sbuffer sbuf;
        msgpack::pack(sbuf, img_data);

        // Publish the serialized image to the Redis channel
        global_count += 1;
        std::cout << "Publish Image : " << frame.rows << " * " << frame.cols << ": " <<  global_count << std::endl;
        redisReply *reply = (redisReply *)redisCommand(c, "PUBLISH image_channel %b", sbuf.data(), sbuf.size());
        if (reply == NULL) {
            std::cerr << "Error publishing message to Redis: " << c->errstr << std::endl;
            break;
        }

        // Terminate the loop after 2000 iterations
        if (global_count == 2000) {
            exit(1);
        }
        freeReplyObject(reply);
    }

    // Free the Redis connection and exit successfully
    redisFree(c);
    return EXIT_SUCCESS;
}
