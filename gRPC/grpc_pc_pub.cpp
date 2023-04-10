#include <iostream>
#include <random>
#include <vector>
#include <grpcpp/grpcpp.h>
#include "point_cloud.grpc.pb.h"

#define MSG_SIZE_TEST 10000

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using point_cloud_transfer::PointCloud;
using point_cloud_transfer::PointCloudTransfer;
using point_cloud_transfer::Response;

class PointCloudTransferClient {
public:
    int global_count{0};
    PointCloudTransferClient(std::shared_ptr<Channel> channel) : stub_(PointCloudTransfer::NewStub(channel)) 
    { 
    }

    std::string SendPointCloud(uint32_t width, uint32_t height, 
                    bool is_dense, std::vector<float> pointCloudPoints) {
        PointCloud request;
        // Set point cloud data in the request
        for (float point : pointCloudPoints) {
            request.add_points(point);
        }

        request.set_width(width);
        request.set_height(height);
        request.set_is_dense(is_dense);


        Response response;
        ClientContext context;

        // Call the RPC and store its response and status
        std::cout << "Sending point cloud data to the server..." << std::endl;
        Status status = stub_->SendPointCloud(&context, request, &response);
        std::cout << "Received response from the server." << std::endl;

        // If the status is OK, return the response message; otherwise, return an error message
        if (status.ok()) {
            std::cout << "Publish Pointcloud : " <<  global_count++ << std::endl;
            return response.message();
        } else {
            std::cout << "Error: " << status.error_code() << ": " << status.error_message() << std::endl;
            return "Error";
        }
    }

private:
    std::unique_ptr<PointCloudTransfer::Stub> stub_;
};

int main(int argc, char** argv) {
    int global_count{0};
    uint32_t width = 16000;
    uint32_t height = 1;
    bool is_dense = false;
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(-1.0, 1.0);

    std::string server_address("0.0.0.0:50051");

    // Create a client object with the specified server address
    std::cout << "Connecting to server at: " << server_address << std::endl;
    PointCloudTransferClient client(grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials()));

    std::vector<float> pointCloudPoints_(width * height * 3);
    for (size_t i = 0; i < width * height * 3; i++) 
        pointCloudPoints_[i] = distribution(generator);   

    while (true)
    {
        // Send the PointCloudData to the server and receive the response
        std::string response = client.SendPointCloud(width, height, is_dense, pointCloudPoints_);
        std::cout << "Server response: " << response << std::endl;
        
        if (client.global_count == MSG_SIZE_TEST)
        {
            return EXIT_SUCCESS;
        }
    }
    return EXIT_SUCCESS;
}
