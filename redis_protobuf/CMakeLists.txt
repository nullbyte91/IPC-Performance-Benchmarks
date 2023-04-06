cmake_minimum_required(VERSION 3.0)
project(redis)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ZeroMQ library
find_path(HIREDIS_HEADER hiredis)
find_package(OpenCV REQUIRED core imgproc video highgui calib3d imgcodecs) 

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}  # Add this line
)


find_library(HIREDIS_LIB hiredis)
find_path(REDIS_PLUS_PLUS_HEADER sw)
find_package(Protobuf REQUIRED)

protobuf_generate_cpp(PROTO_SRC PROTO_HDRS proto/pointcloud.proto proto/image.proto)
include_directories(${PROTOBUF_INCLUDE_DIRS})
# file(GLOB PROTO_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/*.cc")

# Create the executable
add_executable(redis_publisher redis_publisher.cpp ${PROTO_SRC} ${PROTO_HDRS})
add_executable(redis_subscriber redis_subscriber.cpp ${PROTO_SRC} ${PROTO_HDRS})
add_executable(redis_pc_publisher redis_pc_publisher.cpp ${PROTO_SRC} ${PROTO_HDRS})
add_executable(redis_pc_subscriber redis_pc_subscriber.cpp ${PROTO_SRC} ${PROTO_HDRS})

# Link the required libraries
target_link_libraries(redis_publisher ${PROTOBUF_LIBRARIES} ${OpenCV_LIBS} hiredis)
target_link_libraries(redis_subscriber ${PROTOBUF_LIBRARIES} ${OpenCV_LIBS} hiredis)
target_link_libraries(redis_pc_publisher ${PROTOBUF_LIBRARIES} ${OpenCV_LIBS} hiredis)
target_link_libraries(redis_pc_subscriber ${PROTOBUF_LIBRARIES} ${OpenCV_LIBS} hiredis)