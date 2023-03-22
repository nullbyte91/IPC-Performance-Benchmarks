#include "rclcpp/rclcpp.hpp"
#include "ros2_publisher.hpp"
#include "ros2_subscriber_img.hpp"

int main(int argc, char * argv[])
{
  // Initialize the ROS2 communication
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  auto subscriber = std::make_shared<SubscriberNode<Image>>("image_topic");
  executor.add_node(subscriber);
  executor.spin();

  // Shutdown the ROS2 communication
  rclcpp::shutdown();

  return 0;
}
