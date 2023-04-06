#include <iostream>
#include <msgpack.hpp>
#include <hiredis/hiredis.h>
#include <vector>

struct PointCloudData {
  std::vector<float> points;
  uint32_t width;
  uint32_t height;
  bool is_dense;
  MSGPACK_DEFINE(points, width, height, is_dense);
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
  redisReply *reply = (redisReply *)redisCommand(c, "SUBSCRIBE pointcloud_channel");
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
        PointCloudData pointcloud_data = unpacked_data.get().as<PointCloudData>();
        // Process the received point cloud data
        global_count += 1;
        std::cout << "Received point cloud with " << pointcloud_data.width << " points: " << global_count << std::endl;
        // Add point cloud processing code here
      } catch (const std::exception &e) {
        std::cerr << "Error deserializing message: " << e.what() << std::endl;
      }
    }
    freeReplyObject(reply);
  }
  // Free the Redis connection and exit successfully
  redisFree(c);
  return EXIT_SUCCESS;
}
