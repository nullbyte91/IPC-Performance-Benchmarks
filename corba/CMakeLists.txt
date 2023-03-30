cmake_minimum_required(VERSION 3.10)
project(ImageTransfer)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find omniORB
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(omniORB4 REQUIRED)

# Add include directories
include_directories(${OpenCV_INCLUDE_DIRS} ${OMNIORB4_INCLUDE_DIRS})

# Create server executable
add_executable(server server.cpp ImageTransferSK.cc)
target_link_libraries(server ${OpenCV_LIBS} ${OMNIORB4_LIBRARIES})

# Create client executable
add_executable(client client.cpp ImageTransferSK.cc)
target_link_libraries(client ${OpenCV_LIBS} ${OMNIORB4_LIBRARIES})
