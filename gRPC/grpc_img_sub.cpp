#include <grpcpp/grpcpp.h>
#include <grpcpp/impl/codegen/status_code_enum.h>
#include "image_transfer.grpc.pb.h"
#include <opencv2/opencv.hpp>

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using image_transfer::ImageTransfer;
using image_transfer::ImageRequest;
using image_transfer::ImageResponse;

#define MSG_SIZE 1073741824
#define SERVER_ADDRESS "0.0.0.0:50051"

int global_count{0};

class ImageTransferServiceImpl final : public ImageTransfer::Service {
    Status SendImage(ServerContext* context, const ImageRequest* request, ImageResponse* response) override {
        // Convert image data in the request to an OpenCV Mat object
        cv::Mat img(request->height(), request->width(), request->type(), (void*)request->image_data().data());

        // Process the image, e.g., display or save it
        std::cout << "Received image: " << img.rows << " * " << img.cols << ": " << global_count++ << std::endl;

        // Set the response message
        response->set_message("Image received successfully.");
        // Return an OK status
        return Status::OK;
    }
};

void RunServer() {
    std::string server_address(SERVER_ADDRESS);
    ImageTransferServiceImpl service;

    ServerBuilder builder;
    // Add a listening port with insecure server credentials
    builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
    // Register the service implementation
    builder.RegisterService(&service);
    // Set the maximum send and receive message sizes
    builder.SetMaxSendMessageSize(MSG_SIZE);
    builder.SetMaxReceiveMessageSize(MSG_SIZE);
    // Build and start the server
    std::unique_ptr<Server> server(builder.BuildAndStart());
    std::cout << "Server listening on " << server_address << std::endl;

    // Wait for the server to shutdown
    server->Wait();
}

int main(int argc, char** argv) {
    RunServer();
    return 0;
}
