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
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "policy_map.hpp"

using namespace std::chrono_literals;

// ROS2 std_msg types
using String = std_msgs::msg::String;
using Int32 = std_msgs::msg::Int32;
using Float32 = std_msgs::msg::Float32;
using Image = sensor_msgs::msg::Image;
using PointCloud = sensor_msgs::msg::PointCloud2;


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


        publisher_ = this->create_publisher<T>(topic, qos);
        timer_ = this->create_wall_timer(30ns, [this]() {
            publishMessage();
        });
    }

private:
    // ROS Parameter
    rmw_qos_reliability_policy_t reliability_policy_;
    rmw_qos_history_policy_t history_policy_;
    size_t depth_ = 1000;
    size_t count_ = 0;
    typename rclcpp::Publisher<T>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::Mat frame_1 = cv::Mat::eye(480, 640, CV_8UC3);
    cv::Mat frame_2 = cv::Mat::eye(1280, 960, CV_8UC3);
    cv::Mat frame_3 = cv::Mat::eye(1800, 1200, CV_8UC3);
    cv::Mat frame_4 = cv::Mat::eye(2100, 1500, CV_8UC3);

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
        sensor_msgs::msg::Image::UniquePtr msg(new sensor_msgs::msg::Image());
        // Pack the OpenCV image into the ROS image.
        msg->header.frame_id = "camera_frame";
        msg->height = frame_1.rows;
        msg->width = frame_1.cols;
        msg->encoding = mat_type2encoding(frame_1.type());
        msg->is_bigendian = false;
        msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_1.step);
        msg->data.assign(frame_1.datastart, frame_1.dataend);
        publisher_->publish(std::move(msg));
        
        RCLCPP_INFO(this->get_logger(), "%d : Publishing Image: 1280 * 960", count_++);
        sensor_msgs::msg::Image::UniquePtr msg1(new sensor_msgs::msg::Image());
        msg1->header.frame_id = "camera_frame";
        msg1->height = frame_2.rows;
        msg1->width = frame_2.cols;
        msg1->encoding = mat_type2encoding(frame_2.type());
        msg1->is_bigendian = false;
        msg1->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_2.step);
        msg1->data.assign(frame_2.datastart, frame_2.dataend);
        publisher_->publish(std::move(msg1));

        RCLCPP_INFO(this->get_logger(), "%d : Publishing Image: 1800 * 1200", count_++);
        sensor_msgs::msg::Image::UniquePtr msg3(new sensor_msgs::msg::Image());
        msg3->header.frame_id = "camera_frame";
        msg3->height = frame_3.rows;
        msg3->width = frame_3.cols;
        msg3->encoding = mat_type2encoding(frame_3.type());
        msg3->is_bigendian = false;
        msg3->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_3.step);
        msg3->data.assign(frame_3.datastart, frame_3.dataend);
        publisher_->publish(std::move(msg3));

        RCLCPP_INFO(this->get_logger(), "%d : Publishing Image: 2100 * 1500", count_++);
        sensor_msgs::msg::Image::UniquePtr msg4(new sensor_msgs::msg::Image());
        msg4->header.frame_id = "camera_frame";
        msg4->height = frame_4.rows;
        msg4->width = frame_4.cols;
        msg4->encoding = mat_type2encoding(frame_4.type());
        msg4->is_bigendian = false;
        msg4->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_4.step);
        msg4->data.assign(frame_4.datastart, frame_4.dataend);
        publisher_->publish(std::move(msg4));
        if (count_ == 5000)
        {
            std::cout << "Stopping timer" << std::endl;
            timer_->cancel();
        }
    }

    template <typename U = T, std::enable_if_t<std::is_same<U, PointCloud>::value, int> = 0>
    void publishMessage()
    {
        sensor_msgs::msg::PointCloud2 pointcloud;
        pointcloud.header.stamp = this->now();
        pointcloud.header.frame_id = "pointcloud_frame";
        pointcloud.height = 1;
        pointcloud.width = 16000;
        pointcloud.is_dense = false;
        pointcloud.is_bigendian = false;
        sensor_msgs::PointCloud2Modifier modifier(pointcloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");

        std::default_random_engine generator;
        std::uniform_real_distribution<float> distribution(-1.0, 1.0);

        sensor_msgs::PointCloud2Iterator<float> iter_x(pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(pointcloud, "z");
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        *iter_x = distribution(generator);
        *iter_y = distribution(generator);
        *iter_z = distribution(generator);
        }

        RCLCPP_INFO(this->get_logger(), "Publishing point cloud with %d points", pointcloud.width);
        publisher_->publish(std::move(pointcloud));        
        count_ += 1;
        if (count_ == 5000)
        {
            std::cout << "Stopping timer" << std::endl;
            timer_->cancel();            
        }
    }
};

#endif //ROS2_PUBLISHER_HPP

