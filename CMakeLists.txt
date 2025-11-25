cmake_minimum_required(VERSION 3.10)
project(gazebo_map_creator_standalone VERSION 1.0.0)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# Find required packages
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem)
find_package(PCL 1.2 REQUIRED COMPONENTS common io)
find_package(octomap REQUIRED)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

# Link directories
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

# Add definitions
add_definitions(${PCL_DEFINITIONS})

# Build the Gazebo plugin library (manual trigger version)
add_library(gazebo_map_creator_plugin SHARED
  src/gazebo_map_creator_plugin.cpp
)

target_link_libraries(gazebo_map_creator_plugin
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  pthread
)

# Build the auto-generating plugin library
add_library(gazebo_map_creator_plugin_auto SHARED
  src/gazebo_map_creator_plugin_auto.cpp
  src/map_creator_core.cpp
)

target_link_libraries(gazebo_map_creator_plugin_auto
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  pthread
)

# Build the standalone command-line tool
add_executable(map_creator_cli
  src/map_creator_cli.cpp
  src/map_creator_core.cpp
)

target_link_libraries(map_creator_cli
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  pthread
)

# Installation
install(TARGETS gazebo_map_creator_plugin gazebo_map_creator_plugin_auto
  LIBRARY DESTINATION lib
)

install(TARGETS map_creator_cli
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

# Print configuration summary
message(STATUS "")
message(STATUS "Configuration Summary:")
message(STATUS "  Project: ${PROJECT_NAME} ${PROJECT_VERSION}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Gazebo: ${GAZEBO_VERSION}")
message(STATUS "  PCL: ${PCL_VERSION}")
message(STATUS "")
