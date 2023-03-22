#include <iostream>
#include <hiredis/hiredis.h>
#include <opencv2/opencv.hpp>

#include "image.pb.h"

int main() {
    int global_count{0};
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

    redisReply *reply = (redisReply *)redisCommand(c, "SUBSCRIBE image_channel");
    freeReplyObject(reply);

    while (redisGetReply(c, (void **)&reply) == REDIS_OK) {
        if (reply == NULL || reply->type == REDIS_REPLY_ERROR) {
            std::cerr << "Error receiving message from Redis: " << c->errstr << std::endl;
            break;
        }

        if (reply->type == REDIS_REPLY_ARRAY && reply->elements == 3 &&
            std::string(reply->element[0]->str) == "message") {
            // Deserialize the message using protobuf
            trodes::proto::Image serializableMat;
            serializableMat.ParseFromArray(reply->element[2]->str, reply->element[2]->len);

            try {
                // Convert the serialized matrix to an OpenCV Mat object
                cv::Mat img(serializableMat.rows(), serializableMat.cols(), serializableMat.elt_type());
                size_t dataSize = serializableMat.rows() * serializableMat.cols() * serializableMat.elt_size();
                memcpy(img.data, serializableMat.mat_data().data(), dataSize);
                global_count += 1;
                std::cout << "Received image: " << img.rows << " * " << img.cols << ": " << global_count << std::endl;
                // Display the image
                // cv::imshow("Image", img);
                // cv::waitKey(1);
            } catch (const std::exception &e) {
                std::cerr << "Error deserializing message: " << e.what() << std::endl;
            }
        }

        freeReplyObject(reply);
    }

    redisFree(c);
    return EXIT_SUCCESS;
}
