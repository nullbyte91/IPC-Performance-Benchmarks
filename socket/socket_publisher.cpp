#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#define PORT 8080

int main(int argc, char const *argv[])
{
    // OpenCV video capture
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera" << std::endl;
        return -1;
    }

    // Create socket
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock == -1) {
        std::cerr << "Failed to create socket" << std::endl;
        return -1;
    }

    // Prepare the address structure for the socket
    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr("127.0.0.1");
    address.sin_port = htons(PORT);

    // Connect to the server
    int connect_result = connect(sock, (struct sockaddr *)&address, sizeof(address));
    if (connect_result == -1) {
        std::cerr << "Failed to connect to the server" << std::endl;
        return -1;
    }

    // Publish images from camera
    while (true) {
        std::cout << "debug 1" << std::endl;
        cv::Mat frame;
        cap >> frame;

        // Serialize the image to a byte buffer
        std::vector<uchar> buffer;
        cv::imencode(".png", frame, buffer);

        std::cout << "debug 2" << std::endl;
        // Send the size of the image
        int size = buffer.size();
        if (send(sock, &size, sizeof(size), 0) == -1) {
            std::cerr << "Failed to send size" << std::endl;
            break;
        }

        std::cout << "debug 3" << std::endl;
        // Send the image data
        if (send(sock, buffer.data(), buffer.size(), 0) == -1) {
            std::cerr << "Failed to send data" << std::endl;
            break;
        }

        usleep(1000);
    }

    // Close the socket
    close(sock);

    return 0;
}
