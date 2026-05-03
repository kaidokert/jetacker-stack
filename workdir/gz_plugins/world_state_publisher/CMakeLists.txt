cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(world_state_publisher)

# Find Gazebo
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-sim8 REQUIRED)
set(GZ_SIM_VER ${gz-sim8_VERSION_MAJOR})

# Find other dependencies
find_package(gz-common5 REQUIRED COMPONENTS profiler)
find_package(gz-math7 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(sdformat14 REQUIRED)

# Build WorldStatePublisher (unified pose + joint state publisher)
add_library(WorldStatePublisher SHARED
  WorldStatePublisher.cc
  WorldStatePublisher.hh
)

target_include_directories(WorldStatePublisher
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(WorldStatePublisher
  PRIVATE
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    gz-common5::gz-common5
    gz-math7::gz-math7
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    sdformat14::sdformat14
)

# Set output directory
set_target_properties(WorldStatePublisher PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Build joint_reset helper (standalone executable, not a plugin)
add_executable(joint_reset joint_reset.cc)
target_link_libraries(joint_reset
  PRIVATE
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
)
set_target_properties(joint_reset PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Print info
message(STATUS "Building WorldStatePublisher plugin")
message(STATUS "Building joint_reset helper")
message(STATUS "  Gazebo Sim version: ${GZ_SIM_VER}")
message(STATUS "  Output: ${CMAKE_BINARY_DIR}/libWorldStatePublisher.so")
message(STATUS "  Output: ${CMAKE_BINARY_DIR}/joint_reset")
