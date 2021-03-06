cmake_minimum_required(VERSION 2.8.3)
project(struck)

if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "No build type selected, defaulting to Release")
    set(CMAKE_BUILD_TYPE "Release")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_HOME_DIRECTORY}/cmake ${CMAKE_MODULE_PATH})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

#find_package(Eigen REQUIRED)
find_package(OpenCV  REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  sensor_msgs  
 cv_bridge  
 image_transport  
  genmsg
  tf
)
add_message_files(
  FILES
  roi_input.msg
  track_output.msg
#  raw_ctrl_sp.msg
#  trj_ctrl_sp.msg
#  state_est.msg
#  output.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
  INCLUDE_DIRS include

#  LIBRARIES struck
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  src
  ${catkin_INCLUDE_DIRS}
  include
#  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

file(GLOB_RECURSE SRC RELATIVE ${PROJECT_SOURCE_DIR} src/*.cpp)
file(GLOB_RECURSE HEADERS RELATIVE ${PROJECT_SOURCE_DIR} src/*.h)

add_executable(struck ${HEADERS} ${SRC})
add_dependencies(struck ${catkin_EXPORTED_TARGETS})
add_dependencies(struck struck_generate_messages_cpp)
target_link_libraries(struck
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
)

