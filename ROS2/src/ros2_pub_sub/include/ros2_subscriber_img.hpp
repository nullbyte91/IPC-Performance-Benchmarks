#ifndef ROS2_SUBSCRIBER_IMG_HPP
#define ROS2_SUBSCRIBER_IMG_HPP

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

using String = std_msgs::msg::String;
using Int32 = std_msgs::msg::Int32;
using Float32 = std_msgs::msg::Float32;
using Image = sensor_msgs::msg::Image;

inline int
encoding2mat_type(const std::string & encoding)
{
  if (encoding == "mono8") {
    return CV_8UC1;
  } else if (encoding == "bgr8") {
    return CV_8UC3;
  } else if (encoding == "mono16") {
    return CV_16SC1;
  } else if (encoding == "rgba8") {
    return CV_8UC4;
  }
  throw std::runtime_error("Unsupported mat type");
}
template <typename T>
class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode(const std::string& topic)
    : Node("SubscriberNode")
  {
    subscription_ = this->create_subscription<T>(topic, 1000,
        [this](const typename T::SharedPtr msg) {
          if (std::is_same<T, Image>::value) {
            cv::Mat frame(
            msg->height, msg->width,
            encoding2mat_type(msg->encoding),
            msg->data.data());
            RCLCPP_INFO(this->get_logger(), "%d : Received Image %d %d", count_++, frame.rows,  frame.cols);
          }
        });
  }
private:
  typename rclcpp::Subscription<T>::SharedPtr subscription_;
  size_t count_ = 0;
};

#endif //ROS2_SUBSCRIBER_IMG_HPP