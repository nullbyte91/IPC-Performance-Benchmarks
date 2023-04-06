#include <iostream>
#include <vector>
#include <zmq.hpp>
#include <msgpack.hpp>

#include "pointcloud_pack.hpp"

int main() {
  int global_count{0};
  zmq::context_t ctx{1};
  zmq::socket_t client{ctx, zmq::socket_type::req};
  client.connect("tcp://127.0.0.1:5555");

  const std::string req_data{"asking for point cloud"};

  while (true) {
    // Send a request
    std::cout << "sending request... " << std::endl;
    client.send(zmq::buffer(req_data), zmq::send_flags::none);

    // Get the response data
    zmq::message_t response_data;
    client.recv(response_data, zmq::recv_flags::none);

    // Deserialize the response data
    msgpack::unpacked unpacked_response_data;
    msgpack::unpack(unpacked_response_data,
                    static_cast<const char*>(response_data.data()),
                    response_data.size());

    auto unpacked_response = unpacked_response_data.get().as<PointCloudData>();

    std::cout << "[⚓ Received point cloud with " << unpacked_response.width << " points: " << global_count++ << "] =>" << std::endl;

    // Process the received point cloud data
    // In this example, we simply print the first 5 points
    for (size_t i = 0; i < 5 && i < unpacked_response.width; ++i) {
      std::cout << "Point " << i << ": x=" << unpacked_response.points[i * 3]
                << ", y=" << unpacked_response.points[i * 3 + 1]
                << ", z=" << unpacked_response.points[i * 3 + 2] << std::endl;
    }
  }

  return EXIT_SUCCESS;
}
