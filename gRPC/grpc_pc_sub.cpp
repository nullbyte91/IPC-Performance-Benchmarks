#include <grpcpp/grpcpp.h>
#include "point_cloud.grpc.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using point_cloud_transfer::PointCloud;
using point_cloud_transfer::PointCloudTransfer;
using point_cloud_transfer::Response;

class PointCloudTransferServiceImpl final : public PointCloudTransfer::Service {
    Status SendPointCloud(ServerContext* context, const PointCloud* request, Response* response) override {
        // Process the PointCloud data
        std::cout << "Received PointCloud: " << request->width() << " * " << request->height() << " : " << global_count++ << std::endl;

        // Set the response message
        response->set_message("PointCloud received successfully.");

        // Return an OK status
        return Status::OK;
    }
    private:
        int global_count{0};
};

void RunServer() {
    std::string server_address("0.0.0.0:50051");
    PointCloudTransferServiceImpl service;

    ServerBuilder builder;
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    builder.RegisterService(&service);
    std::unique_ptr<Server> server(builder.BuildAndStart());
    std::cout << "Server listening on " << server_address << std::endl;

    server->Wait();
}

int main(int argc, char** argv) {
    RunServer();
    return 0;
}
