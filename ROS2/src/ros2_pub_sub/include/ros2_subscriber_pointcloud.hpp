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
using PointCloud = custom_msg::msg::PointCloud;

#define MSG_SIZE_TEST 9000

inline 
std::chrono::high_resolution_clock::time_point to_chrono_time(const rclcpp::Time& rclcpp_time) {
    auto nanoseconds = rclcpp_time.nanoseconds();
    auto duration = std::chrono::nanoseconds(nanoseconds);
    return std::chrono::high_resolution_clock::time_point(duration);
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

          if (exit_flag_) {
            RCLCPP_INFO(this->get_logger(), "Latency: %lf ns", NS_TO_MS(computeMeanLatency()));
            double deser_time = timeMeasurement_.mean_time(MSG_SIZE_TEST);
            RCLCPP_INFO(this->get_logger(), "Deseralization Mean time: %lf in ns", NS_TO_MS(deser_time));
            return;
          }

          // Convert the rclcpp::Time to std::chrono::high_resolution_clock::time_point.
          auto chrono_time = to_chrono_time(msg->time.stamp);

          // Get the current time as std::chrono::high_resolution_clock::time_point.
          auto current_chrono_time = std::chrono::high_resolution_clock::now();

          // Compute the difference between the two time points.
          auto time_diff = current_chrono_time - chrono_time;
      
          // Convert the time difference to milliseconds.
          time_diff_ns_ += std::chrono::duration_cast<std::chrono::nanoseconds>(time_diff).count();

          timeMeasurement_.measure_time<void>(std::bind(&SubscriberNode<T>::deserialize_pcl, this, msg));

          RCLCPP_INFO(this->get_logger(), "%d : PointCloud Image %d %d", count_++, msg->pointcloud.width, msg->pointcloud.height);
          if (count_ == MSG_SIZE_TEST)
          {
            exit_flag_ = true;
          }
        });
  }

  void deserialize_pcl(const typename T::SharedPtr &msg) 
  {
      // Yet to define
  }

  double computeMeanLatency() const
  {
      if (time_diff_ns_ == 0)
      {
          return 0.0;
      }
      return static_cast<double>(time_diff_ns_) / count_;
  }

private:
  typename rclcpp::Subscription<T>::SharedPtr subscription_;
  size_t count_ = 0;
  std::atomic<bool> exit_flag_{false};
  double total_time_;
  std::chrono::nanoseconds::rep time_diff_ns_;
  TimeMeasurement timeMeasurement_;
};

#endif //ROS2_SUBSCRIBER_IMG_HPP