cmake_minimum_required(VERSION 3.0.2)
project(gnss_ros_standardization)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)

find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  GnssObservation.msg
  GnssObservations.msg
  GnssEphemeris.msg
  GlonassEphemeris.msg
  GnssEphemerides.msg
  GnssSolution.msg
  ExternalGnssSolution.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    message_runtime
  DEPENDS EIGEN3
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Phase 1 ROS1/Noetic integration:
# normalize an already-solved GNSS/PVT/RTK solution into GnssSolution.
add_executable(external_solution_adapter
  src/adapters/external_solution_adapter.cpp
)

add_dependencies(external_solution_adapter
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(external_solution_adapter
  ${catkin_LIBRARIES}
)

install(TARGETS external_solution_adapter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
