#include <iostream>
#include <random>
#include <vector>
#include <hiredis/hiredis.h>
#include "pointcloud.pb.h"

int main() {
    // Create example point cloud data
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(-1.0, 1.0);

    int count{0}, global_count{0};
    uint32_t width = 16000;
    uint32_t height = 1;
    bool is_dense = false;
    std::vector<float> points(width * height * 3);

    for (size_t i = 0; i < width * height * 3; i++) {
        points[i] = distribution(generator);
    }

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

    // Publish point clouds in a loop
    while (true) {
        // Create a PointCloudData protobuf message
        trodes::proto::PointCloud pointcloud_data;

        // Set the point cloud data fields
        pointcloud_data.set_width(width);
        pointcloud_data.set_height(height);
        pointcloud_data.set_is_dense(is_dense);
        for (const auto &point : points) {
            pointcloud_data.add_points(point);
        }

        // Serialize the message using Protocol Buffers
        std::string serialized;
        if (!pointcloud_data.SerializeToString(&serialized)) {
            std::cerr << "Error serializing message" << std::endl;
            break;
        }

        global_count += 1;
        std::cout << "Publish Point Cloud: " << width << " points: " << global_count << std::endl;

        // Publish the message to the Redis channel
        redisReply *reply = (redisReply *)redisCommand(c, "PUBLISH pointcloud_channel %b", serialized.data(), serialized.size());
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
