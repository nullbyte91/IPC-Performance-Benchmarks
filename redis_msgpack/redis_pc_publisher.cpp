#include <iostream>
#include <msgpack.hpp>
#include <hiredis/hiredis.h>
#include <random>
#include <vector>

struct PointCloudData {
  std::vector<float> points;
  uint32_t width;
  uint32_t height;
  bool is_dense;
  MSGPACK_DEFINE(points, width, height, is_dense);
};

int main() {
  // Connect to the Redis server
  redisContext* c = redisConnect("127.0.0.1", 6379);

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

  std::default_random_engine generator;
  std::uniform_real_distribution<float> distribution(-1.0, 1.0);

  int global_count{0};
  while (true) {
    // Generate a random point cloud
    uint32_t width = 16000;
    uint32_t height = 1;
    bool is_dense = false;
    std::vector<float> points(width * height * 3);

    for (size_t i = 0; i < width * height * 3; i++) {
      points[i] = distribution(generator);
    }

    // Create a PointCloudData struct and fill it with the generated point cloud data
    PointCloudData pointcloud_data;
    pointcloud_data.points = points;
    pointcloud_data.width = width;
    pointcloud_data.height = height;
    pointcloud_data.is_dense = is_dense;

    // Serialize the PointCloudData struct using msgpack
    msgpack::sbuffer sbuf;
    msgpack::pack(sbuf, pointcloud_data);

    // Publish the serialized point cloud to the Redis channel
    global_count += 1;
    std::cout << "Publish Point Cloud with " << width << " points: " << global_count << std::endl;
    redisReply* reply = (redisReply*)redisCommand(c, "PUBLISH pointcloud_channel %b", sbuf.data(), sbuf.size());
    if (reply == NULL) {
      std::cerr << "Error publishing message to Redis: " << c->errstr << std::endl;
      break;
    }

    // Terminate the loop after 2000 iterations
    if (global_count == 2000) {
        break;
    }
    freeReplyObject(reply);
  }

  // Free the Redis connection and exit successfully
  redisFree(c);
  return EXIT_SUCCESS;
}
