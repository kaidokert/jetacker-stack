cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(world_pose_publisher)

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

# Build our custom WorldPosePublisher
add_library(WorldPosePublisher SHARED
  custom/WorldPosePublisher.cc
  custom/WorldPosePublisher.hh
)

target_include_directories(WorldPosePublisher
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/custom
)

target_link_libraries(WorldPosePublisher
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
set_target_properties(WorldPosePublisher PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Build instrumented upstream PosePublisher (for investigation)
add_library(PosePublisherInstrumented SHARED
  upstream/PosePublisher_instrumented.cc
  upstream/PosePublisher.hh
)

target_include_directories(PosePublisherInstrumented
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/upstream
)

target_link_libraries(PosePublisherInstrumented
  PRIVATE
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER}
    gz-common5::gz-common5
    gz-math7::gz-math7
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    sdformat14::sdformat14
)

set_target_properties(PosePublisherInstrumented PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Install rules (optional - we'll use directly from build dir)
# install(TARGETS WorldPosePublisher
#   LIBRARY DESTINATION lib/gz-sim-${GZ_SIM_VER}/plugins
# )

# Print info
message(STATUS "Building WorldPosePublisher plugin")
message(STATUS "  Gazebo Sim version: ${GZ_SIM_VER}")
message(STATUS "  Output: ${CMAKE_BINARY_DIR}/libWorldPosePublisher.so")
message(STATUS "Building PosePublisherInstrumented plugin (for investigation)")
message(STATUS "  Output: ${CMAKE_BINARY_DIR}/libPosePublisherInstrumented.so")
