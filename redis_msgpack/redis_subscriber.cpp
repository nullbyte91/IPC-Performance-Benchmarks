#include <iostream>
#include <msgpack.hpp>
#include <hiredis/hiredis.h>
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/all_layers.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

struct Image {
    std::vector<uchar> matrix;
    int rows = 0;
    int cols = 0;
    int type = 0;
    MSGPACK_DEFINE(matrix, rows, cols, type);
};

int main() {
    int global_count{0};

    // Connect to the Redis server
    redisContext *c = redisConnect("127.0.0.1", 6379);

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

    // Subscribe to the Redis channel
    redisReply *reply = (redisReply *)redisCommand(c, "SUBSCRIBE image_channel");
    freeReplyObject(reply);

    // Listen for messages from the Redis channel
    while (redisGetReply(c, (void **)&reply) == REDIS_OK) {
        if (reply == NULL || reply->type == REDIS_REPLY_ERROR) {
            std::cerr << "Error receiving message from Redis: " << c->errstr << std::endl;
            break;
        }

        if (reply->type == REDIS_REPLY_ARRAY && reply->elements == 3 &&
            std::string(reply->element[0]->str) == "message") {

            // Deserialize the message using msgpack
            msgpack::unpacked unpacked_data;
            msgpack::unpack(unpacked_data, reply->element[2]->str, reply->element[2]->len);

            try {
                Image img_data = unpacked_data.get().as<Image>();

                // Process the received image data
                global_count += 1;
                std::cout << "Received image: " << img_data.rows << " * " << img_data.cols << ": " << global_count << std::endl;
                // cv::imshow(
                //     "Result",
                //     cv::Mat(img_data.rows, img_data.cols,
                //             img_data.type, img_data.matrix.data()));
                // cv::waitKey(1);
            } catch (const std::exception& e) {
                std::cerr << "Error deserializing message: " << e.what() << std::endl;
            }
        }

        freeReplyObject(reply);
    }

    // Free the Redis connection and exit successfully
    redisFree(c);
    return EXIT_SUCCESS;
}
