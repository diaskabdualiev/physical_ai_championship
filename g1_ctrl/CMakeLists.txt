cmake_minimum_required(VERSION 3.12)
project(g1_29dof_controller)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(yaml-cpp REQUIRED)

# Detect architecture for onnxruntime
execute_process(COMMAND uname -m OUTPUT_VARIABLE ARCH OUTPUT_STRIP_TRAILING_WHITESPACE)
if(ARCH STREQUAL "aarch64")
  set(ONNX_DIR ${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-aarch64-1.22.0)
elseif(ARCH STREQUAL "x86_64")
  set(ONNX_DIR ${PROJECT_SOURCE_DIR}/thirdparty/onnxruntime-linux-x64-1.22.0)
else()
  message(FATAL_ERROR "Unsupported architecture: ${ARCH}")
endif()

include_directories(
  /usr/include/eigen3
  /usr/local/include/ddscxx # dds
  /usr/local/include/iceoryx/v2.0.2 # dds
  include
  ${ONNX_DIR}/include
  ${PROJECT_SOURCE_DIR}/include/
)

link_libraries(
  unitree_sdk2 ddsc ddscxx rt pthread # dds
  libboost_program_options.a libyaml-cpp.a fmt
  ${ONNX_DIR}/lib/libonnxruntime.so
)

file(GLOB_RECURSE ADD_SRC_LIST
  ${PROJECT_SOURCE_DIR}/src/*.cpp
)

add_library(${PROJECT_NAME}_lib ${ADD_SRC_LIST})
link_libraries(${PROJECT_NAME}_lib)

add_executable(g1_ctrl main.cpp)