#include "rclcpp/rclcpp.hpp"
#include "ros2_publisher.hpp"
#include "ros2_subscriber.hpp"

int main(int argc, char * argv[])
{
  // Initialize the ROS2 communication
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor executor;
  auto publisher = std::make_shared<PublisherNode<Int32>>("int32_topic");
  auto subscriber = std::make_shared<SubscriberNode<Int32>>("int32_topic");
  executor.add_node(publisher);
  executor.add_node(subscriber);
  executor.spin();

  // Shutdown the ROS2 communication
  rclcpp::shutdown();

  return 0;
}
