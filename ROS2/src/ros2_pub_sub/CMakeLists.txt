cmake_minimum_required(VERSION 3.5)
project(ros2_pub_sub)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(custom_msg REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(OpenCV REQUIRED COMPONENTS core highgui imgproc videoio)

include_directories(include)
include_directories(include ${custom_msg_INCLUDE_DIRS})
# include_directories("/home/jshat8y7/Desktop/mygit/IPC-Performance-Benchmarks/ROS2/install/custom_msg/include/")

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# Compilation and Installtion
add_executable(test_string src/test_string.cpp)
target_link_libraries(test_string
  ${builtin_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  opencv_core
  opencv_highgui
  opencv_imgproc)

add_executable(test_int32 src/test_int32.cpp)
target_link_libraries(test_int32
  ${builtin_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  opencv_core
  opencv_highgui
  opencv_imgproc)

add_executable(test_float32 src/test_float32.cpp)
target_link_libraries(test_float32
  ${builtin_interfaces_TARGETS}
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  opencv_core
  opencv_highgui
  opencv_imgproc)


add_executable(test_image_pub src/test_img_pub.cpp)
ament_target_dependencies(test_image_pub
  "builtin_interfaces"
  "sensor_msgs"
  "rclcpp"
  "custom_msg"
)
target_link_libraries(test_image_pub
  opencv_core
  opencv_highgui
  opencv_imgproc
)

add_executable(test_image_sub src/test_img_sub.cpp)
ament_target_dependencies(test_image_sub
  "builtin_interfaces"
  "sensor_msgs"
  "rclcpp"
  "custom_msg"
)
target_link_libraries(test_image_sub
  opencv_core
  opencv_highgui
  opencv_imgproc
)

add_executable(test_pointcloud_pub src/test_pc_pub.cpp)
ament_target_dependencies(test_pointcloud_pub
  "builtin_interfaces"
  "sensor_msgs"
  "rclcpp"
  "custom_msg"
)
target_link_libraries(test_pointcloud_pub
  opencv_core
  opencv_highgui
  opencv_imgproc
)

add_executable(test_pointcloud_sub src/test_pc_sub.cpp)
ament_target_dependencies(test_pointcloud_sub
  "builtin_interfaces"
  "sensor_msgs"
  "rclcpp"
  "custom_msg"
)
target_link_libraries(test_pointcloud_sub
  opencv_core
  opencv_highgui
  opencv_imgproc
)
  
install(TARGETS
  test_string
  test_int32
  test_float32
  test_image_pub 
  test_image_sub
  test_pointcloud_pub
  test_pointcloud_sub
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
