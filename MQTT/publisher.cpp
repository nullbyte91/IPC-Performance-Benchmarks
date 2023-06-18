#include <iostream>
#include <opencv2/opencv.hpp>
#include "mqtt/async_client.h"

class MQTTPublisher {
private:
    std::string ADDRESS;
    std::string CLIENTID;
    std::string TOPIC;

    mqtt::async_client client;
    const int MSG_SIZE_TEST = 20000;

public:
    MQTTPublisher(std::string address, std::string clientID, std::string topic)
        : ADDRESS(address), CLIENTID(clientID), TOPIC(topic), client(ADDRESS, CLIENTID) {}

    void publishImages() {
        // Create example OpenCV matrices (images)
        cv::Mat frame_1 = cv::Mat::eye(480, 640, CV_8UC3);
        cv::Mat frame_2 = cv::Mat::eye(1280, 960, CV_8UC3);
        cv::Mat frame_3 = cv::Mat::eye(1800, 1200, CV_8UC3);
        cv::Mat frame_4 = cv::Mat::eye(2100, 1500, CV_8UC3);

        int count{0}, global_count{0};
        cv::Mat frame;

        try {
            client.connect()->wait();  // Connect client before the loop.

            while (true) {
                if (count == 0) {
                    frame = frame_1;
                    count += 1;
                } else if (count == 1) {
                    frame = frame_2;
                    count += 1;
                } else if (count == 2) {
                    frame = frame_3;
                    count += 1;
                } else {
                    frame = frame_4;
                    count = 0;
                }

                std::vector<uchar> buffer;
                cv::imencode(".jpg", frame, buffer);
                auto byte_ptr = reinterpret_cast<const char*>(buffer.data());

                mqtt::message_ptr pubmsg = mqtt::make_message(TOPIC, byte_ptr, buffer.size());
                pubmsg->set_qos(2);
                client.publish(pubmsg)->wait_for(std::chrono::milliseconds(1));

                global_count++;

                std::cout << "Image published successfully: " << global_count << std::endl;

                // Terminate the loop after 2000 iterations
                if (global_count == MSG_SIZE_TEST) {
                    break;
                }
            }

            client.disconnect()->wait();  // Disconnect client after the loop.
        }
        catch (const mqtt::exception& exc) {
            std::cerr << exc.what() << std::endl;
        }
    }
};

int main(int argc, char* argv[]) {
    // Create an MQTTPublisher object
    MQTTPublisher publisher("tcp://localhost:1883", "MQTTPublisher", "MQTTIMAGE");

    // Publish images
    publisher.publishImages();

    return 0;
}