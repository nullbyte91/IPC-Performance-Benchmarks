cmake_minimum_required(VERSION 3.10)
project(socket)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

# Build publisher executable
add_executable(socket_publisher socket_publisher.cpp)
target_link_libraries(socket_publisher ${REDIS_PLUS_PLUS_LIB} ${OpenCV_LIBS} -lhiredis -lredis++ -pthread)

# Build subscriber executable
add_executable(socket_subscriber socket_subscriber.cpp)
target_link_libraries(socket_subscriber ${REDIS_PLUS_PLUS_LIB} ${OpenCV_LIBS} -lhiredis -lredis++ -pthread)

# target_link_libraries(subscriber ${OpenCV_LIBS} ${CMAKE_THREAD_LIBS_INIT})
