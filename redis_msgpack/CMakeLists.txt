cmake_minimum_required(VERSION 3.0)
project(redis)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ZeroMQ library
find_path(HIREDIS_HEADER hiredis)
find_package(OpenCV REQUIRED core imgproc video highgui calib3d imgcodecs) 

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

find_library(HIREDIS_LIB hiredis)
find_path(REDIS_PLUS_PLUS_HEADER sw)
find_package(msgpack REQUIRED)

# Add source files
set(SOURCES
    redis_publisher.cpp
    redis_subscriber.cpp
)

# Build publisher executable
add_executable(redis_publisher redis_publisher.cpp)
target_link_libraries(redis_publisher ${REDIS_PLUS_PLUS_LIB} ${OpenCV_LIBS} -lhiredis -lredis++ -pthread)

# Build subscriber executable
add_executable(redis_subscriber redis_subscriber.cpp)
target_link_libraries(redis_subscriber ${REDIS_PLUS_PLUS_LIB} ${OpenCV_LIBS} -lhiredis -lredis++ -pthread)