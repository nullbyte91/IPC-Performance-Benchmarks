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
function(build_executable target_name source_name)
  add_executable(${target_name} ${source_name})
  target_link_libraries(${target_name} ${REDIS_PLUS_PLUS_LIB} ${OpenCV_LIBS} -lhiredis -lredis++ -pthread)
endfunction()

build_executable(redis_publisher redis_publisher.cpp)
build_executable(redis_subscriber redis_subscriber.cpp)
build_executable(redis_pc_publisher redis_pc_publisher.cpp)
build_executable(redis_pc_subscriber redis_pc_subscriber.cpp)