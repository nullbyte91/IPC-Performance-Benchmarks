#include <grpcpp/grpcpp.h>
#include "point_cloud_transfer.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using point_cloud_transfer::PointCloud;
using point_cloud_transfer::PointCloudTransfer;
using point_cloud_transfer::Response;

class PointCloudTransferClient {
public:
    PointCloudTransferClient(std::shared_ptr<Channel> channel) : stub_(PointCloudTransfer::NewStub(channel)) {}

    std::string SendPointCloud(const PointCloudData& point_cloud_data) {
        PointCloud request;
        // Set point cloud data in the request
        for (float point : point_cloud_data.points) {
            request.add_points(point);
        }
        request.set_width(point_cloud_data.width);
        request.set_height(point_cloud_data.height);
        request.set_is_dense(point_cloud_data.is_dense);

        Response response;
        ClientContext context;

        // Call the RPC and store its response and status
        Status status = stub_->SendPointCloud(&context, request, &response);

        // If the status is OK, return the response message; otherwise, return an error message
        if (status.ok()) {
            return response.message();
        } else {
            std::cout << "Error: " << status.error_code() << ": " << status.error_message() << std::endl;
            return "Error";
        }
    }

private:
    std::unique_ptr<PointCloudTransfer::Stub> stub_;
};

struct PointCloudData {
    std::vector<float> points;
    uint32_t width;
    uint32_t height;
    bool is_dense;
};

int main(int argc, char** argv) {
    std::string server_address("0.0.0.0:50051");

    // Create a client object with the specified server address
    PointCloudTransferClient client(grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials()));

    // Create example PointCloudData
    PointCloudData point_cloud_data;
    point_cloud_data.points = {1.0, 2.0, 3.0, 4.0, 5.0}; // Fill in with actual data
    point_cloud_data.width = 16000;
    point_cloud_data.height = 1;
    point_cloud_data.is_dense = true;

    // Send the PointCloudData to the server and receive the response
    std::string response = client.SendPointCloud(point_cloud_data);

    std::cout << "Server response: " << response << std::endl;

    return 0;
}
