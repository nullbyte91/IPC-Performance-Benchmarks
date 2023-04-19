cmake_minimum_required(VERSION 3.0)
project(zmq_msgpack)

set(CMAKE_CXX_STANDARD 14)

find_package(cppzmq REQUIRED)
find_package(OpenCV REQUIRED core imgproc video highgui calib3d imgcodecs) 
find_package(msgpack REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(zmq_publisher zmq_publisher.cpp)
target_link_libraries(zmq_publisher pthread cppzmq ${OpenCV_LIBS})

add_executable(zmq_subscriber zmq_subscriber.cpp)
target_link_libraries(zmq_subscriber pthread cppzmq ${OpenCV_LIBS})

add_executable(zmq_pc_publisher zmq_pc_publisher.cpp)
target_link_libraries(zmq_pc_publisher pthread cppzmq ${OpenCV_LIBS})

add_executable(zmq_pc_subscriber zmq_pc_subscriber.cpp)
target_link_libraries(zmq_pc_subscriber pthread cppzmq ${OpenCV_LIBS})