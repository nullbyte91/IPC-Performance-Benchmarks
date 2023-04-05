#include <chrono>
#include <random>
#include <vector>
#include <iostream>
#include <cstring>
#include <zmq.hpp>
#include <msgpack.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

struct PointCloudData {
  std::vector<float> points;
  uint32_t width;
  uint32_t height;
  bool is_dense;
};

int main() {
  zmq::context_t ctx{1};
  zmq::socket_t server{ctx, zmq::socket_type::rep};
  server.bind("tcp://127.0.0.1:5555");

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

  while (true) {
    // Generate a random point cloud


    // Wait for requests
    std::cout << "open for request..." << std::endl;
    zmq::message_t req;
    server.recv(req, zmq::recv_flags::none);
    std::cout << "[⚡ got request] => " << req.to_string() << std::endl;

    // Prepare data
    PointCloudData pointcloud_data;
    pointcloud_data.points = points;
    pointcloud_data.width = width;
    pointcloud_data.height = height;
    pointcloud_data.is_dense = is_dense;

    // Pack/serialize the data using msgpack
    msgpack::sbuffer serialized_pointcloud;
    msgpack::pack(&serialized_pointcloud, pointcloud_data);
    std::cout << "[⚓ Publish point cloud with " << width << " points: " << global_count++ << "] =>" << std::endl;

    // Send the serialized data as the response to the client
    zmq::message_t packed_msg(serialized_pointcloud.size());
    std::memcpy(packed_msg.data(), serialized_pointcloud.data(), serialized_pointcloud.size());
    server.send(packed_msg, zmq::send_flags::none);

    // Terminate the loop after 2000 iterations
    if (global_count == 20000) {
       break;
    }
  }

  return EXIT_SUCCESS;
}
