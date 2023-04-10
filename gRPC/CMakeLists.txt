cmake_minimum_required(VERSION 3.5.1)

project(gRPC)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
  add_definitions(-D_WIN32_WINNT=0x600)
endif()

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)

if(GRPC_AS_SUBMODULE)
  add_subdirectory(../../.. ${CMAKE_CURRENT_BINARY_DIR}/grpc EXCLUDE_FROM_ALL)
  message(STATUS "Using gRPC via add_subdirectory.")
  set(_PROTOBUF_LIBPROTOBUF libprotobuf)
  set(_REFLECTION grpc++_reflection)
  if(CMAKE_CROSSCOMPILING)
    find_program(_PROTOBUF_PROTOC protoc)
  else()
    set(_PROTOBUF_PROTOC $<TARGET_FILE:protobuf::protoc>)
  endif()
  set(_GRPC_GRPCPP grpc++)
  if(CMAKE_CROSSCOMPILING)
    find_program(_GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)
  else()
    set(_GRPC_CPP_PLUGIN_EXECUTABLE $<TARGET_FILE:grpc_cpp_plugin>)
  endif()
else()
  set(protobuf_MODULE_COMPATIBLE TRUE)
  find_package(Protobuf CONFIG REQUIRED)
  message(STATUS "Using protobuf ${Protobuf_VERSION}")
  set(_PROTOBUF_LIBPROTOBUF protobuf::libprotobuf)
  set(_REFLECTION gRPC::grpc++_reflection)
  if(CMAKE_CROSSCOMPILING)
    find_program(_PROTOBUF_PROTOC protoc)
  else()
    set(_PROTOBUF_PROTOC $<TARGET_FILE:protobuf::protoc>)
  endif()
  find_package(gRPC CONFIG REQUIRED)
  message(STATUS "Using gRPC ${gRPC_VERSION}")
  set(_GRPC_GRPCPP gRPC::grpc++)
  if(CMAKE_CROSSCOMPILING)
    find_program(_GRPC_CPP_PLUGIN_EXECUTABLE grpc_cpp_plugin)
  else()
    set(_GRPC_CPP_PLUGIN_EXECUTABLE $<TARGET_FILE:gRPC::grpc_cpp_plugin>)
  endif()
endif()

get_filename_component(hw_proto "proto/image_transfer.proto" ABSOLUTE)
get_filename_component(hw_proto_path "${hw_proto}" PATH)

set(hw_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/image_transfer.pb.cc")
set(hw_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/image_transfer.pb.h")
set(hw_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/image_transfer.grpc.pb.cc")
set(hw_grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/image_transfer.grpc.pb.h")
add_custom_command(
      OUTPUT "${hw_proto_srcs}" "${hw_proto_hdrs}" "${hw_grpc_srcs}" "${hw_grpc_hdrs}"
      COMMAND ${_PROTOBUF_PROTOC}
      ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
        --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
        -I "${hw_proto_path}"
        --plugin=protoc-gen-grpc="${_GRPC_CPP_PLUGIN_EXECUTABLE}"
        "${hw_proto}"
      DEPENDS "${hw_proto}")

include_directories("${CMAKE_CURRENT_BINARY_DIR}")

# foreach(_target
#   server client)
#   add_executable(${_target} "${_target}.cpp"
#     ${hw_proto_srcs}
#     ${hw_grpc_srcs})
#   target_link_libraries(${_target}
#     ${_REFLECTION}
#     ${_GRPC_GRPCPP}
#     ${_PROTOBUF_LIBPROTOBUF}
#     ${OpenCV_LIBS})
# endforeach()

# get_filename_component(image_transfer_proto "proto/image_transfer.proto" ABSOLUTE)
# get_filename_component(image_transfer_proto_path "${image_transfer_proto}" PATH)

# set(image_transfer_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/image_transfer.pb.cc")
# set(image_transfer_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/image_transfer.pb.h")
# set(image_transfer_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/image_transfer.grpc.pb.cc")
# set(image_transfer_grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/image_transfer.grpc.pb.h")
# add_custom_command(
#       OUTPUT "${image_transfer_proto_srcs}" "${image_transfer_proto_hdrs}" "${image_transfer_grpc_srcs}" "${image_transfer_grpc_hdrs}"
#       COMMAND ${Protobuf_PROTOC_EXECUTABLE}
#       ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
#         --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
#         -I "${image_transfer_proto_path}"
#         --plugin=protoc-gen-grpc="${gRPC_CPP_PLUGIN_EXECUTABLE}"
#         "${image_transfer_proto}"
#       DEPENDS "${image_transfer_proto}")

# # Point Cloud Proto
# get_filename_component(point_cloud_proto "proto/point_cloud.proto" ABSOLUTE)
# get_filename_component(point_cloud_proto_path "${point_cloud_proto}" PATH)

# set(point_cloud_proto_srcs "${CMAKE_CURRENT_BINARY_DIR}/point_cloud.pb.cc")
# set(point_cloud_proto_hdrs "${CMAKE_CURRENT_BINARY_DIR}/point_cloud.pb.h")
# set(point_cloud_grpc_srcs "${CMAKE_CURRENT_BINARY_DIR}/point_cloud.grpc.pb.cc")
# set(point_cloud_grpc_hdrs "${CMAKE_CURRENT_BINARY_DIR}/point_cloud.grpc.pb.h")
# add_custom_command(
#       OUTPUT "${point_cloud_proto_srcs}" "${point_cloud_proto_hdrs}" "${point_cloud_grpc_srcs}" "${point_cloud_grpc_hdrs}"
#       COMMAND ${Protobuf_PROTOC_EXECUTABLE}
#       ARGS --grpc_out "${CMAKE_CURRENT_BINARY_DIR}"
#         --cpp_out "${CMAKE_CURRENT_BINARY_DIR}"
#         -I "${point_cloud_proto_path}"
#         --plugin=protoc-gen-grpc="${gRPC_CPP_PLUGIN_EXECUTABLE}"
#         "${point_cloud_proto}"
#       DEPENDS "${point_cloud_proto}")

function(build_executable target_name source_name)
  add_executable(${target_name} ${source_name} ${hw_proto_srcs} ${hw_grpc_srcs})
  target_link_libraries(${target_name} ${_REFLECTION} ${_GRPC_GRPCPP} ${_PROTOBUF_LIBPROTOBUF} ${OpenCV_LIBS})
endfunction()

build_executable(grpc_img_pub grpc_img_pub.cpp)
build_executable(grpc_img_sub grpc_img_sub.cpp)

