#include <grpcpp/grpcpp.h>
#include <grpcpp/impl/codegen/status_code_enum.h>
#include "image_transfer.grpc.pb.h"
#include <opencv2/opencv.hpp>
#include <fstream>

#include "utils.hpp"

using grpc::Channel;
using grpc::ClientContext;
using grpc::Status;
using image_transfer::ImageTransfer;
using image_transfer::ImageRequest;
using image_transfer::ImageResponse;

#define SERVER_ADDRESS "0.0.0.0:50051"
#define MSG_SIZE_TEST 11000

class ImageTransferClient {
public:
    int global_count{0};
    ImageTransferClient(std::shared_ptr<Channel> channel) : stub_(ImageTransfer::NewStub(channel)) {}

    // SendImage function sends the cv::Mat image to the server
    std::string SendImage(const cv::Mat& img) {
        ImageRequest request;
        ImageResponse response;
        ClientContext context;

        // Measure the time taken to serialize the image frame
        timeMeasurement_.measure_time<void>(std::bind(&ImageTransferClient::serialize_image_frame, this, std::ref(request), std::ref(img)));

        // Call the RPC and store its response and status
        Status status = stub_->SendImage(&context, request, &response);
        
        // If the status is OK, return the response message; otherwise, return an error message
        if (status.ok()) {
            std::cout << "Publish Image : " << img.rows << " * " << img.cols << ": " <<  global_count++ << std::endl;
            return response.message();
        } else {
            std::cout << "Error: " << status.error_code() << ": " << status.error_message() << std::endl;
            return "Error";
        }
    }

    // Calculate and return the mean serialization time
    double mean_serialization_time(int num_messages) {
        return timeMeasurement_.mean_time(num_messages);
    }

private:
    std::unique_ptr<ImageTransfer::Stub> stub_;
    TimeMeasurement timeMeasurement_;

    // Serialize the image frame and populate the ImageRequest object
    void serialize_image_frame(image_transfer::ImageRequest &request, cv::Mat img) {
        // Set image data, width, height, and type in the request
        request.set_image_data(img.data, img.total() * img.elemSize());
        request.set_width(img.cols);
        request.set_height(img.rows);
        request.set_type(img.type());
        request.set_timestamp(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count());       
    }
};

int main(int argc, char** argv) {
    int count{0};

    // Create example OpenCV matrices (images)
    cv::Mat frame_1 = cv::Mat::eye(480, 640, CV_8UC3);
    cv::Mat frame_2 = cv::Mat::eye(1280, 960, CV_8UC3);
    cv::Mat frame_3 = cv::Mat::eye(1800, 1200, CV_8UC3);
    cv::Mat frame_4 = cv::Mat::eye(2100, 1500, CV_8UC3);
    cv::Mat frame;
    std::string server_address(SERVER_ADDRESS);

    // Create a client object with the specified server address
    ImageTransferClient client(grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials()));
    
    while (true)
    {
        // Cycle through the example images
        if (count == 0)
        {
            frame = frame_1;
            count += 1;
        } else if (count == 1)
        {
            frame = frame_2;
            count += 1;
        } else if (count == 2)
        {
            frame = frame_3;
            count += 1;
        } else {
            frame = frame_4;
            count = 0;
        }

        // Send the current image to the server and receive the response
        std::string response = client.SendImage(frame); 

        if (client.global_count == MSG_SIZE_TEST)
        {
            std::cout << "* help ** " << std::endl;
            double ser_time = client.mean_serialization_time(MSG_SIZE_TEST - 1);
            std::cout << "Serialization Mean time: " << NS_TO_MS(ser_time) << " ms" << std::endl;
            sleep(10);
        }
        // To Do
        // Convert the string response to StatusCode
    }
    return EXIT_SUCCESS;
}
