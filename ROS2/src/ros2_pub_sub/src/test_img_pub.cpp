#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "ros2_publisher.hpp"

int main(int argc, char * argv[])
{
  // Initialize the ROS2 communication
  rclcpp::init(argc, argv);

  sleep(10);

  rclcpp::executors::SingleThreadedExecutor executor;
  auto publisher = std::make_shared<PublisherNode<Image>>("image_topic");
  executor.add_node(publisher);
  executor.spin();

  // Shutdown the ROS2 communication
  rclcpp::shutdown();

  return 0;
}