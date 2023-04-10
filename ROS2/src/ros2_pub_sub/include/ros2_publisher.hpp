#ifndef ROS2_PUBLISHER_HPP
#define ROS2_PUBLISHER_HPP

#include <chrono>
#include <memory>
#include <typeinfo>
#include <random>

#include <rclcpp/rclcpp.hpp>

#include "opencv2/highgui/highgui.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "custom_msg/msg/image.hpp"
#include "custom_msg/msg/point_cloud.hpp"

#include "policy_map.hpp"
#include "utils.hpp"

#define MSG_SIZE_TEST 10000
using namespace std::chrono_literals;

// ROS2 std_msg types
using String = std_msgs::msg::String;
using Int32 = std_msgs::msg::Int32;
using Float32 = std_msgs::msg::Float32;
using Image = custom_msg::msg::Image;
using PointCloud = custom_msg::msg::PointCloud;

inline rclcpp::Time to_rclcpp_time(const std::chrono::high_resolution_clock::time_point& time_point) {
    auto duration = time_point.time_since_epoch();
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count();
    return rclcpp::Time(nanoseconds);
}

inline std::string mat_type2encoding(int mat_type)
{
    switch (mat_type) {
        case CV_8UC1:
        return "mono8";
        case CV_8UC3:
        return "bgr8";
        case CV_16SC1:
        return "mono16";
        case CV_8UC4:
        return "rgba8";
        default:
        throw std::runtime_error("Unsupported encoding type");
    }
}

template <typename T>
class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode(const std::string& topic)
        : Node("PublisherNode")
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
        publisher_ = this->create_publisher<T>(topic, qos);
        timer_ = this->create_wall_timer(50ns, [this]() {
            publishMessage();
        });
    }

private:
    // ROS Parameter
    rmw_qos_reliability_policy_t reliability_policy_;
    rmw_qos_history_policy_t history_policy_;
    size_t depth_ = 2000;
    size_t count_ = 0;
    typename rclcpp::Publisher<T>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::atomic<bool> exit_flag_{false};
    TimeMeasurement timeMeasurement_;
    cv::Mat frame_0 = cv::Mat::eye(480, 640, CV_8UC3);
    cv::Mat frame_1 = cv::Mat::eye(1280, 960, CV_8UC3);
    cv::Mat frame_2 = cv::Mat::eye(1800, 1200, CV_8UC3);
    cv::Mat frame_3 = cv::Mat::eye(2100, 1500, CV_8UC3);

    template <typename U = T, std::enable_if_t<std::is_same<U, String>::value, int> = 0>
    void publishMessage()
    {
        RCLCPP_INFO(this->get_logger(), "Publishing string:");
        auto message = T();
        message.data = "Hello world";
        publisher_->publish(message);
        
    }

    template <typename U = T, std::enable_if_t<std::is_same<U, Int32>::value, int> = 0>
    void publishMessage()
    {
        RCLCPP_INFO(this->get_logger(), "Publishing int: ");
        auto message = T();
        message.data = 60000;
        publisher_->publish(message);
    }

    template <typename U = T, std::enable_if_t<std::is_same<U, Float32>::value, int> = 0>
    void publishMessage()
    {
        RCLCPP_INFO(this->get_logger(), "Publishing int: ");
        auto message = T();
        message.data = 160000;
        publisher_->publish(message);
    }

    template <typename U = T, std::enable_if_t<std::is_same<U, Image>::value, int> = 0>
    void publishMessage()
    {
        RCLCPP_INFO(this->get_logger(), "%d : Publishing Image: 480 * 640", count_++);
        auto msg0 = Image();
        timeMeasurement_.measure_time<void>(std::bind(&PublisherNode<T>::serialize_image_frame, this, std::ref(msg0), frame_0));
        publisher_->publish(std::move(msg0));
        
        RCLCPP_INFO(this->get_logger(), "%d : Publishing Image: 1280 * 960", count_++);
        auto msg1 = Image();
        timeMeasurement_.measure_time<void>(std::bind(&PublisherNode<T>::serialize_image_frame, this, std::ref(msg1), frame_1));
        publisher_->publish(std::move(msg1));

        RCLCPP_INFO(this->get_logger(), "%d : Publishing Image: 1800 * 1200", count_++);
        auto msg2 = Image();
        timeMeasurement_.measure_time<void>(std::bind(&PublisherNode<T>::serialize_image_frame, this, std::ref(msg2), frame_2));
        publisher_->publish(std::move(msg2));

        RCLCPP_INFO(this->get_logger(), "%d : Publishing Image: 2100 * 1500", count_++);
        auto msg3 = Image();
        timeMeasurement_.measure_time<void>(std::bind(&PublisherNode<T>::serialize_image_frame, this, std::ref(msg3), frame_3));
        publisher_->publish(std::move(msg3));
        if (count_ == MSG_SIZE_TEST)
        {
            double ser_time = timeMeasurement_.mean_time(MSG_SIZE_TEST-1);
            RCLCPP_INFO(this->get_logger(), "seralization Mean time: %lf ns", NS_TO_MS(ser_time));
            timer_->cancel();
        }
    }

    template <typename U = T, std::enable_if_t<std::is_same<U, PointCloud>::value, int> = 0>
    void publishMessage()
    {
        auto pcl = PointCloud();
        timeMeasurement_.measure_time<void>(std::bind(&PublisherNode<T>::serialize_image_pcl, this, std::ref(pcl)));
        publisher_->publish(std::move(pcl));        
        RCLCPP_INFO(this->get_logger(), "%d : Publishing point cloud with %d points", count_++, pcl.pointcloud.width);
        if (count_ == MSG_SIZE_TEST)
        {
            double ser_time = timeMeasurement_.mean_time(MSG_SIZE_TEST-1);
            RCLCPP_INFO(this->get_logger(), "seralization Mean time: %lf ns", NS_TO_MS(ser_time));
            timer_->cancel(); 
        }
    }

    void serialize_image_pcl(custom_msg::msg::PointCloud &pcl) {
        pcl.pointcloud.header.stamp = this->now();
        pcl.pointcloud.header.frame_id = "pointcloud_frame";
        pcl.pointcloud.height = 1;
        pcl.pointcloud.width = 16000;
        pcl.pointcloud.is_dense = false;
        pcl.pointcloud.is_bigendian = false;
        sensor_msgs::PointCloud2Modifier modifier(pcl.pointcloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");

        std::default_random_engine generator;
        std::uniform_real_distribution<float> distribution(-1.0, 1.0);

        sensor_msgs::PointCloud2Iterator<float> iter_x(pcl.pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(pcl.pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(pcl.pointcloud, "z");
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            *iter_x = distribution(generator);
            *iter_y = distribution(generator);
            *iter_z = distribution(generator);
        }
        pcl.time.stamp = this->now();
    }

    void serialize_image_frame(custom_msg::msg::Image &msg, cv::Mat frame) {
        msg.img.header.frame_id = "camera_frame";
        msg.img.height = frame.rows;
        msg.img.width = frame.cols;
        msg.img.encoding = mat_type2encoding(frame.type());
        msg.img.is_bigendian = false;
        msg.img.step = static_cast<decltype(msg.img.step)>(frame.step);
        msg.img.data.assign(frame.datastart, frame.dataend);
        msg.time.stamp = to_rclcpp_time(std::chrono::high_resolution_clock::now());
  }
};

#endif //ROS2_PUBLISHER_HPP

