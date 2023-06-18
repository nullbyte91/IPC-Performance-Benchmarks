cmake_minimum_required(VERSION 3.5)
project(MQTTPubSub)

# specify which C++ standard you need 
set(CMAKE_CXX_STANDARD 14)

# locate libraries
find_package(OpenCV REQUIRED)
find_package(PahoMqttCpp REQUIRED)

# specify the C++ source files
set(PUBLISHER_SOURCES publisher.cpp)
set(SUBSCRIBER_SOURCES subscriber.cpp)

# add the executables
add_executable(MQTTImagePublisher ${PUBLISHER_SOURCES})
add_executable(MQTTImageSubscriber ${SUBSCRIBER_SOURCES})

# link libraries to the executables
target_link_libraries(MQTTImagePublisher ${OpenCV_LIBS} paho-mqttpp3 paho-mqtt3a)
target_link_libraries(MQTTImageSubscriber ${OpenCV_LIBS} paho-mqttpp3 paho-mqtt3a)
