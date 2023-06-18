#include <iostream>
#include <opencv2/opencv.hpp>
#include "mqtt/async_client.h"

const std::string ADDRESS("tcp://localhost:1883");
const std::string CLIENTID("MQTTSubscriber");
const std::string TOPIC("MQTTIMAGE");

int global_count{0};

class callback : public virtual mqtt::callback
{
public:
    void message_arrived(mqtt::const_message_ptr msg) override {


        // Construct vector from the message payload
        std::vector<uchar> buffer(msg->get_payload().begin(), msg->get_payload().end());

        cv::Mat img = cv::imdecode(buffer, cv::IMREAD_COLOR);
        if (img.empty()) {
            std::cout << "Could not decode image" << std::endl;
            return;
        }
        global_count++;
        std::cout << "Image received: " << global_count << " Topic Name: " << msg->get_topic() << std::endl;
    }
};

int main(int argc, char* argv[])
{
    mqtt::async_client client(ADDRESS, CLIENTID);
    
    callback cb;
    client.set_callback(cb);
    
    try {
        client.connect()->wait();
        client.start_consuming();
        client.subscribe(TOPIC, 1)->wait();
        
        // Wait indefinitely until you manually terminate application
        while (true);
    }
    catch (const mqtt::exception& exc) {
        std::cerr << exc.what() << std::endl;
        return 1;
    }

    return 0;
}
