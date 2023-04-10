cmake_minimum_required(VERSION 3.15)

project(transfer_protos)

find_package(Protobuf REQUIRED)
find_package(gRPC REQUIRED)

# Generate protobuf and gRPC files for image_transfer.proto
set(IMAGE_TRANSFER_PROTO image_transfer.proto)
protobuf_generate(LANGUAGE cpp OUT_VAR IMAGE_TRANSFER_PB_CPP_FILES PROTOS ${IMAGE_TRANSFER_PROTO})
grpc_generate(LANGUAGE cpp OUT_VAR IMAGE_TRANSFER_GRPC_CPP_FILES PROTOS ${IMAGE_TRANSFER_PROTO})

# Create a library for image_transfer proto
add_library(image_transfer_proto ${IMAGE_TRANSFER_PB_CPP_FILES} ${IMAGE_TRANSFER_GRPC_CPP_FILES})
target_link_libraries(image_transfer_proto
    gRPC::grpc++_reflection
    protobuf::libprotobuf
)

# Generate protobuf and gRPC files for point_cloud.proto
set(POINT_CLOUD_PROTO point_cloud.proto)
protobuf_generate(LANGUAGE cpp OUT_VAR POINT_CLOUD_PB_CPP_FILES PROTOS ${POINT_CLOUD_PROTO})
grpc_generate(LANGUAGE cpp OUT_VAR POINT_CLOUD_GRPC_CPP_FILES PROTOS ${POINT_CLOUD_PROTO})

# Create a library for point_cloud proto
add_library(point_cloud_proto ${POINT_CLOUD_PB_CPP_FILES} ${POINT_CLOUD_GRPC_CPP_FILES})
target_link_libraries(point_cloud_proto
    gRPC::grpc++_reflection
    protobuf::libprotobuf
)
