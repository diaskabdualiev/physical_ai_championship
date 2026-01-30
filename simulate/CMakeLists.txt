cmake_minimum_required(VERSION 3.16)
project(unitree_mujoco)

if (CMAKE_BUILD_TYPE MATCHES "Debug")
  message("Debug mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -g -O0 -fPIC")
else()
  message("Release mode")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3 -DNDEBUG -fPIC")
endif()

list(APPEND CMAKE_PREFIX_PATH "/opt/unitree_robotics/lib/cmake")
find_package(unitree_sdk2 REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)
find_package(OpenCV REQUIRED)


# Detect architecture for MuJoCo
execute_process(COMMAND uname -m OUTPUT_VARIABLE ARCH OUTPUT_STRIP_TRAILING_WHITESPACE)
if(ARCH STREQUAL "aarch64")
  set(MUJOCO_DIR ${PROJECT_SOURCE_DIR}/mujoco-3.3.6-linux-aarch64)
elseif(ARCH STREQUAL "x86_64")
  set(MUJOCO_DIR ${PROJECT_SOURCE_DIR}/mujoco-3.3.6-linux-x86_64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${ARCH}")
endif()

# Fallback to generic mujoco/ symlink if arch-specific dir not found
if(NOT EXISTS ${MUJOCO_DIR})
  if(EXISTS ${PROJECT_SOURCE_DIR}/mujoco)
    set(MUJOCO_DIR ${PROJECT_SOURCE_DIR}/mujoco)
    message(STATUS "Using mujoco/ symlink")
  else()
    message(FATAL_ERROR "MuJoCo not found. Download it:\n"
      "  wget https://github.com/google-deepmind/mujoco/releases/download/3.3.6/mujoco-3.3.6-linux-${ARCH}.tar.gz\n"
      "  tar xzf mujoco-3.3.6-linux-${ARCH}.tar.gz\n"
      "  mv mujoco-3.3.6 mujoco-3.3.6-linux-${ARCH}")
  endif()
endif()
message(STATUS "MuJoCo: ${MUJOCO_DIR}")

include_directories(
  ${MUJOCO_DIR}/include
  ${MUJOCO_DIR}/simulate
  src/lodepng
  ${OpenCV_INCLUDE_DIRS}
)
link_directories(${MUJOCO_DIR}/lib)

file(GLOB SIM_SRC
  src/joystick/joystick.cc

  # exclude mujoco simulate/main.cc
  ${MUJOCO_DIR}/simulate/glfw_*.cc
  ${MUJOCO_DIR}/simulate/platform_*.cc
  ${MUJOCO_DIR}/simulate/simulate.cc
  src/lodepng/lodepng.cpp
)

link_libraries(
  pthread
  mujoco
  glfw
  yaml-cpp
  unitree_sdk2
  boost_program_options
  fmt
  ${OpenCV_LIBS}
)

add_executable(unitree_mujoco ${SIM_SRC} src/main.cc)

add_executable(jstest src/joystick/jstest.cc src/joystick/joystick.cc)