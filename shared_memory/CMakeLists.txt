cmake_minimum_required(VERSION 3.0.0)
project(circular-buffer-queue VERSION 0.1.0)

set(Boost_USE_MULTITHREADED OFF)
add_definitions(-DBOOST_ALL_NO_LIB)
find_package( Boost 1.54 COMPONENTS system thread REQUIRED )
find_package( OpenCV REQUIRED )

include_directories(
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS})

set(LINK_LIBRARIES
    rt
    pthread
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS})

add_executable(shared_memory_subscriber shared_memory_subscriber.cpp)
target_link_libraries(shared_memory_subscriber ${LINK_LIBRARIES})

add_executable(shared_memory_publisher shared_memory_publisher.cpp)
target_link_libraries(shared_memory_publisher ${LINK_LIBRARIES})