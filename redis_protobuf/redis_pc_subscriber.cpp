#include <iostream>
#include <hiredis/hiredis.h>
#include <vector>
#include "pointcloud.pb.h"

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
    redisReply *reply = (redisReply *)redisCommand(c, "SUBSCRIBE pointcloud_channel");
    freeReplyObject(reply);

    // Listen for incoming messages
    while (redisGetReply(c, (void **)&reply) == REDIS_OK) {
        // Check for errors in receiving messages
        if (reply == NULL || reply->type == REDIS_REPLY_ERROR) {
            std::cerr << "Error receiving message from Redis: " << c->errstr << std::endl;
            break;
        }

        // Check if the reply contains a message
        if (reply->type == REDIS_REPLY_ARRAY && reply->elements == 3 &&
            std::string(reply->element[0]->str) == "message") {
            // Deserialize the message using protobuf
            trodes::proto::PointCloud pointcloud_data;
            pointcloud_data.ParseFromArray(reply->element[2]->str, reply->element[2]->len);

            try {
                // Access the deserialized point cloud data
                uint32_t width = pointcloud_data.width();
                uint32_t height = pointcloud_data.height();
                bool is_dense = pointcloud_data.is_dense();
                std::vector<float> points(pointcloud_data.points().begin(), pointcloud_data.points().end());

                global_count += 1;
                std::cout << "Received point cloud: " << width << " points: " << global_count << std::endl;

                // Process the received point cloud (e.g., display, analyze, or store)

            } catch (const std::exception &e) {
                std::cerr << "Error deserializing message: " << e.what() << std::endl;
            }
        }

        // Free the reply object
        freeReplyObject(reply);
    }

    // Free the Redis connection and exit successfully
    redisFree(c);
    return EXIT_SUCCESS;
}
