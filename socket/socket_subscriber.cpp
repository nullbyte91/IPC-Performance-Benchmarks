#include <opencv2/opencv.hpp>
#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

#define PORT 8080

int main(int argc, char const *argv[]) {
    int sock = 0, valread;
    struct sockaddr_in serv_addr, client_addr;
    char buffer[1024] = {0};
    cv::Mat image;

    // Create socket
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        std::cout << "\n Socket creation error \n";
        return -1;
    }

    // Set socket options
    int opt = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        std::cout << "setsockopt failed" << std::endl;
        return -1;
    }

    // Configure server address
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    serv_addr.sin_port = htons(PORT);

    // Bind socket to the port
    if (bind(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        std::cout << "bind failed" << std::endl;
        return -1;
    }

    // Listen for incoming connections
    if (listen(sock, 3) < 0) {
        std::cout << "listen failed" << std::endl;
        return -1;
    }

    std::cout << "Listening on port " << PORT << std::endl;

    // Accept incoming connection
    socklen_t client_addr_size = sizeof(client_addr);
    int new_socket;
    if ((new_socket = accept(sock, (struct sockaddr *)&client_addr, &client_addr_size)) < 0) {
        std::cout << "accept failed" << std::endl;
        return -1;
    }

    std::cout << "Client connected" << std::endl;

    // Receive data from the publisher
    while (true) {
        std::cout << "debug 1" << std::endl;
        memset(buffer, 0, sizeof(buffer));
        valread = read(new_socket, buffer, 1024);

        std::cout << "debug 2" << std::endl;

        // Convert received data to cv::Mat
        cv::Mat received_image = cv::imdecode(cv::Mat(1, valread, CV_8UC1, buffer), cv::IMREAD_COLOR);

        std::cout << "[⚓ Received image: " << received_image.rows << " * " << received_image.cols << "] =>"<< std::endl;
        // Check for successful decoding
        if (received_image.empty()) {
            std::cout << "Could not decode received image" << std::endl;
        }
        else {
            image = received_image.clone();
            cv::imshow("Received image", image);
            cv::waitKey(1);
        }
    }

    return 0;
}
