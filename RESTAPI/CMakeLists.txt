cmake_minimum_required(VERSION 3.10)

# Set the project name
project(RESTAPI)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)

# Find OpenCV
find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)

# Include directories for the libraries
include_directories(/usr/local/include/nlohmann)

#Add the executable targets
add_executable(server server.cpp)
add_executable(client client.cpp)

# Link the libraries
target_link_libraries(server ${OpenCV_LIBS} pthread nlohmann_json::nlohmann_json)
target_link_libraries(client ${OpenCV_LIBS} pthread nlohmann_json::nlohmann_json)