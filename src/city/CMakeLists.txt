find_package(protobuf CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)

file(GLOB proto_files
  agent/v2/*.pb.h
  geo/v2/*.pb.h
  map/v2/*.pb.h
  routing/v2/*.pb.h
  trip/v2/*.pb.h
)

FOREACH(FIL ${proto_files})
  GET_FILENAME_COMPONENT(FIL_WE ${FIL} NAME_WE)

  # message("FIL_WE>>>${FIL_WE}")
  string(REGEX REPLACE ".+/(.+)\\..*" "\\1" FILE_NAME ${FIL})

  # message("FILE_NAME>>>${FILE_NAME}")
  string(REGEX REPLACE "(.+)\\${FILE_NAME}.*" "\\1" FILE_PATH ${FIL})

  # message("FILE_PATH>>>${FILE_PATH}")
  LIST(APPEND TOTAL_PROTO_SRCS "${CMAKE_SOURCE_DIR}/src/city/routing/v2/routing_service.grpc.pb.h")
  LIST(APPEND TOTAL_PROTO_SRCS "${CMAKE_SOURCE_DIR}/src/city/routing/v2/routing_service.grpc.pb.cc")
  LIST(APPEND TOTAL_PROTO_SRCS "${FILE_PATH}${FIL_WE}.pb.h")
  LIST(APPEND TOTAL_PROTO_SRCS "${FILE_PATH}${FIL_WE}.pb.cc")

  # message("TOTAL_PROTO_SRCS>>>${TOTAL_PROTO_SRCS}")
ENDFOREACH()

add_library(city ${TOTAL_PROTO_SRCS})
target_link_libraries(city
  protobuf::libprotobuf
  gRPC::grpc++
)

# message(A ${CMAKE_SOURCE_DIR})
target_include_directories(city
  PUBLIC ${CMAKE_SOURCE_DIR}/src
)
