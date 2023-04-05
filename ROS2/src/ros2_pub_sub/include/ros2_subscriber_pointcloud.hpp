#ifndef ROS2_SUBSCRIBER_IMG_HPP
#define ROS2_SUBSCRIBER_IMG_HPP

#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/image.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using String = std_msgs::msg::String;
using Int32 = std_msgs::msg::Int32;
using Float32 = std_msgs::msg::Float32;
using Image = sensor_msgs::msg::Image;
using PointCloud = sensor_msgs::msg::PointCloud2;

template <typename T>
class SubscriberNode : public rclcpp::Node
{
public:
  SubscriberNode(const std::string& topic)
    : Node("SubscriberNode")
  {
    subscription_ = this->create_subscription<T>(topic, 1000,
        [this](const typename T::SharedPtr msg) {
          if (std::is_same<T, PointCloud>::value) {
            RCLCPP_INFO(this->get_logger(), "Received point cloud with %d points", msg->width);
          }
        });
  }
private:
  typename rclcpp::Subscription<T>::SharedPtr subscription_;
  size_t count_ = 0;
};

#endif //ROS2_SUBSCRIBER_IMG_HPP