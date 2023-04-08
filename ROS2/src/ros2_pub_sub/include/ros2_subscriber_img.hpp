#ifndef ROS2_SUBSCRIBER_IMG_HPP
#define ROS2_SUBSCRIBER_IMG_HPP

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

#include "policy_map.hpp"

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

    auto history = name_to_history_policy_map.find("keep_last");
    history_policy_ = history->second;
    auto reliability = name_to_reliability_policy_map.find("reliable");
    reliability_policy_ = reliability->second;
    auto qos = rclcpp::QoS(
    rclcpp::QoSInitialization(
    // Depth represents how many messages to store in history when the
    // history policy is KEEP_LAST.
    history_policy_,
    depth_
    ));
    qos.reliability(reliability_policy_);

    subscription_ = this->create_subscription<T>(topic, qos,
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
  // ROS Parameter
  rmw_qos_reliability_policy_t reliability_policy_;
  rmw_qos_history_policy_t history_policy_;
  size_t depth_ = 1000;
  typename rclcpp::Subscription<T>::SharedPtr subscription_;
  size_t count_ = 0;
};

#endif //ROS2_SUBSCRIBER_IMG_HPP