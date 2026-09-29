cmake_minimum_required(VERSION 3.0.2)
project(gnss_ros_standardization)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

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
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    message_runtime
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# ---------------------------------------------------------------------------
# RTKLIB core
# ---------------------------------------------------------------------------
set(RTKLIB_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/RTKLIB")
if(NOT EXISTS "${RTKLIB_ROOT}/src/rtklib.h")
  message(FATAL_ERROR
    "RTKLIB submodule is missing. Run: git submodule update --init --recursive")
endif()

file(GLOB RTKLIB_CORE "${RTKLIB_ROOT}/src/*.c")
file(GLOB RTKLIB_RCV  "${RTKLIB_ROOT}/src/rcv/*.c")
set(RTKLIB_SOURCES ${RTKLIB_CORE} ${RTKLIB_RCV})

# Standalone applications are not part of the reusable library.
foreach(app
    convbin.c rnx2rtkp.c str2str.c rtknavi.c rtkrcv.c rtkplot.c
    rtkgetopt.c rtksvr.c convkml.c)
  list(REMOVE_ITEM RTKLIB_SOURCES "${RTKLIB_ROOT}/src/${app}")
endforeach()

# These receiver backends are intentionally excluded in the upstream ROS2
# project because this RTKLIB fork removed nav_t::leaps used by them.
list(REMOVE_ITEM RTKLIB_SOURCES
  "${RTKLIB_ROOT}/src/rcv/comnav.c"
  "${RTKLIB_ROOT}/src/rcv/tersus.c"
)

add_library(gnss_rtkcore STATIC ${RTKLIB_SOURCES})
target_include_directories(gnss_rtkcore PUBLIC "${RTKLIB_ROOT}/src")
target_compile_definitions(gnss_rtkcore PUBLIC
  ENAGLO
  ENAGAL
  ENACMP
  ENAQZS
  ENASBS
  ENAIRN
  NEXOBS=3
)
if(UNIX)
  target_link_libraries(gnss_rtkcore m Threads::Threads)
endif()
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(gnss_rtkcore PRIVATE -w)
endif()

function(grs_add_ros1_node target source)
  add_executable(${target} ${source})
  add_dependencies(${target}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS})
  target_include_directories(${target} PRIVATE "${RTKLIB_ROOT}/src")
  target_link_libraries(${target}
    gnss_rtkcore
    ${catkin_LIBRARIES}
    Threads::Threads)
endfunction()

# ---------------------------------------------------------------------------
# Two input modes, one public output contract
# ---------------------------------------------------------------------------

# Mode A: receiver/RTCM raw byte stream -> standardized observations/ephemerides
grs_add_ros1_node(raw_gnss_decoder
  src/ros1/raw_gnss_decoder.cpp)

# Raw observations -> standalone SPP solution
grs_add_ros1_node(single_point_positioning
  src/ros1/single_point_positioning.cpp)

# Rover/base raw observations -> RTK solution
grs_add_ros1_node(real_time_kinematic
  src/ros1/real_time_kinematic.cpp)

# Mode B: already-solved BLH/cov/status -> standardized solution
add_executable(external_solution_adapter
  src/adapters/external_solution_adapter.cpp)
add_dependencies(external_solution_adapter
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})
target_link_libraries(external_solution_adapter
  ${catkin_LIBRARIES})

install(TARGETS
  raw_gnss_decoder
  single_point_positioning
  real_time_kinematic
  external_solution_adapter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  OPTIONAL
)
