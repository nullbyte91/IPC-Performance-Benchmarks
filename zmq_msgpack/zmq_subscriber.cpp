
#include "image.hpp"

int main()
{
  int global_count{0};
  zmq::context_t ctx{1};
  zmq::socket_t client{ctx, zmq::socket_type::req};
  client.connect("tcp://127.0.0.1:5555");

  const std::string req_data{"asking for cv::Mat"};

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

    auto unpacked_response = unpacked_response_data.get().as<Image>();

    cv::Mat img =  cv::Mat(unpacked_response.rows, unpacked_response.cols,
                unpacked_response.type, unpacked_response.matrix.data());

    std::cout << "[⚓ Received image: " << unpacked_response.rows << " * " << unpacked_response.cols << ": " << global_count++ << "] =>"<< std::endl;

    // Return back the vector<uchar> to cv::Mat image and show it 
    // cv::imshow(
    //     "Result",
    //     cv::Mat(unpacked_response.rows, unpacked_response.cols,
    //             unpacked_response.type, unpacked_response.matrix.data()));
    // cv::waitKey(1);                
  }
  
  return EXIT_SUCCESS;
}