#ifndef ROS2_SUBSCRIBER_IMG_HPP
#define ROS2_SUBSCRIBER_IMG_HPP

#include <rclcpp/rclcpp.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <atomic>

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

#include "custom_msg/msg/image.hpp"

#include "policy_map.hpp"
#include "utils.hpp"

#define MSG_SIZE_TEST 9000

using Image = custom_msg::msg::Image;

inline 
std::chrono::high_resolution_clock::time_point to_chrono_time(const rclcpp::Time& rclcpp_time) {
    auto nanoseconds = rclcpp_time.nanoseconds();
    auto duration = std::chrono::nanoseconds(nanoseconds);
    return std::chrono::high_resolution_clock::time_point(duration);
}

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

    auto history = name_to_history_policy_map.find("Keep all");
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
          
          // Measure the time taken for the deserialization operation
          frame_ = timeMeasurement_.measure_time<cv::Mat>(std::bind(&SubscriberNode<T>::deserialize_image, this, msg));

          RCLCPP_INFO(this->get_logger(), "%d : Received Image %d %d", count_++, frame_.rows,  frame_.cols);
          if (count_ == MSG_SIZE_TEST)
          {
            exit_flag_ = true;
          }
        });    
  }
  double computeMeanLatency() const
  {
      if (time_diff_ns_ == 0)
      {
          return 0.0;
      }
      return static_cast<double>(time_diff_ns_) / count_;
  }

  cv::Mat deserialize_image(const typename T::SharedPtr &msg) {
      cv::Mat frame(
          msg->img.height, msg->img.width,
          encoding2mat_type(msg->img.encoding),
          msg->img.data.data());
      return frame;
  }
  ~SubscriberNode()
  {
    std::cout << "destructor called" << std::endl;
  }
private:
  // ROS Parameter
  rmw_qos_reliability_policy_t reliability_policy_;
  rmw_qos_history_policy_t history_policy_;
  size_t depth_ = 1000;
  typename rclcpp::Subscription<T>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_ = 0;
  size_t msg_count_;
  std::atomic<bool> exit_flag_{false};
  double total_time_;
  std::chrono::nanoseconds::rep time_diff_ns_;
  cv::Mat frame_;
  TimeMeasurement timeMeasurement_;
};

#endif //ROS2_SUBSCRIBER_IMG_HPP