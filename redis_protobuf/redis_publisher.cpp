#include <iostream>
#include <hiredis/hiredis.h>
#include <opencv2/opencv.hpp>

#include "image.pb.h"

int main() {

    // Create example OpenCV matrices (images)
    cv::Mat frame_1 = cv::Mat::eye(480, 640, CV_8UC3);
    cv::Mat frame_2 = cv::Mat::eye(1280, 960, CV_8UC3);
    cv::Mat frame_3 = cv::Mat::eye(1800, 1200, CV_8UC3);
    cv::Mat frame_4 = cv::Mat::eye(2100, 1500, CV_8UC3);

    redisContext *c = redisConnect("127.0.0.1", 6379);
    
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

        // Create an Image protobuf message
        trodes::proto::Image serializableMat;

        //set the trivial fields
        serializableMat.set_rows(frame.rows);
        serializableMat.set_cols(frame.cols);
        serializableMat.set_elt_type(frame.type());
        serializableMat.set_elt_size((int)frame.elemSize());

        //set the matrix's raw data
        size_t dataSize = frame.rows * frame.cols * frame.elemSize();
        serializableMat.set_mat_data(frame.data, dataSize);
        // Serialize the message using Protocol Buffers
        std::string serialized;
        if (!serializableMat.SerializeToString(&serialized)) {
            std::cerr << "Error serializing message" << std::endl;
            break;
        }

        global_count += 1;
        std::cout << "Publish Image : " << frame.rows << " * " << frame.cols << ": " <<  global_count << std::endl;
        // Publish the message to the Redis channel
        redisReply *reply = (redisReply *)redisCommand(c, "PUBLISH image_channel %b", serialized.data(), serialized.size());
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

    redisFree(c);
    return EXIT_SUCCESS;
}
