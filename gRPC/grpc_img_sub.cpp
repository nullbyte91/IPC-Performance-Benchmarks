#include <grpcpp/grpcpp.h>
#include <grpcpp/impl/codegen/status_code_enum.h>
#include "image_transfer.grpc.pb.h"
#include <opencv2/opencv.hpp>

#include "utils.hpp"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using image_transfer::ImageTransfer;
using image_transfer::ImageRequest;
using image_transfer::ImageResponse;

#define MSG_SIZE 1073741824
#define SERVER_ADDRESS "0.0.0.0:50051"

#define MSG_SIZE_TEST 9999

int global_count{0};
std::chrono::nanoseconds::rep time_diff_ns_{0};
std::chrono::high_resolution_clock::time_point chrono_time;

TimeMeasurement timeMeasurement_;

class ImageTransferServiceImpl final : public ImageTransfer::Service {
    // Implement the SendImage RPC function
    Status SendImage(ServerContext* context, const ImageRequest* request, ImageResponse* response) override {
        auto current_chrono_time = std::chrono::high_resolution_clock::now();
        
        // Deserialize the image and measure the time taken
        cv::Mat img = timeMeasurement_.measure_time<cv::Mat>(std::bind(&ImageTransferServiceImpl::deserialize_image, this, std::cref(*request)));

        // Calculate the time difference since the last image
        auto time_diff = current_chrono_time - chrono_time;

        time_diff_ns_ += std::chrono::duration_cast<std::chrono::nanoseconds>(time_diff).count();
        std::cout << "time_diff_ns_" << time_diff_ns_ << std::endl;

        // Process the image, e.g., display or save it
        std::cout << "Received image: " << img.rows << " * " << img.cols << ": " << global_count++ << std::endl;

        // Set the response message
        response->set_message("Image received successfully.");

        // Calculate and print the latency and deserialization time
        if (global_count == MSG_SIZE_TEST)
        {
            double latency = static_cast<double>(time_diff_ns_) / global_count;
            std::cout << "Latency: " << NS_TO_MS(latency) << " ms" << std::endl;
            double deser_time = timeMeasurement_.mean_time(MSG_SIZE_TEST);
            std::cout << "Deserialization Mean time: " << NS_TO_MS(deser_time) << " ms" << std::endl;
            sleep(2);
            exit(1);
        }

        // Return an OK status
        return Status::OK;
    }

    // Deserialize the image from the ImageRequest object
    cv::Mat deserialize_image(const ImageRequest& request) const {
        chrono_time = std::chrono::system_clock::time_point(std::chrono::nanoseconds(request.timestamp()));
        cv::Mat img;
        img = cv::Mat(request.height(), request.width(), 
                    request.type(), (void*)request.image_data().data());
        return img;
    }
};

// Function to run the gRPC server
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
