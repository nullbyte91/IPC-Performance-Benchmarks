cmake_minimum_required(VERSION 3.15)

set(PROTO_FILES image_transfer.proto)

protobuf_generate(LANGUAGE cpp OUT_VAR PB_CPP_FILES)
grpc_generate(LANGUAGE cpp OUT_VAR GRPC_CPP_FILES)

add_library(image_transfer_proto ${PB_CPP_FILES} ${GRPC_CPP_FILES})
target_link_libraries(image_transfer_proto
    gRPC::grpc++_reflection
    protobuf::libprotobuf
)
