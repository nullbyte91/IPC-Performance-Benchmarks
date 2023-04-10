#ifndef ROS2_SUBSCRIBER_HPP
#define ROS2_SUBSCRIBER_HPP

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

using String = std_msgs::msg::String;
using Int32 = std_msgs::msg::Int32;
using Float32 = std_msgs::msg::Float32;

template <typename T>
class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode(const std::string& topic)
    : Node("SubscriberNode")
  {
    subscription_ = this->create_subscription<T>(topic, 10,
        [this](const typename T::SharedPtr msg) {
          if (std::is_same<T, String>::value) {
            RCLCPP_INFO(this->get_logger(), "Received string");
          }
          else if (std::is_same<T, Int32>::value) {
            RCLCPP_INFO(this->get_logger(), "Received integer");
          }
          else if (std::is_same<T, Float32>::value) {
            RCLCPP_INFO(this->get_logger(), "Received float");
          }
        });
  }
private:
  typename rclcpp::Subscription<T>::SharedPtr subscription_;
};

#endif //ROS2_SUBSCRIBER_HPP