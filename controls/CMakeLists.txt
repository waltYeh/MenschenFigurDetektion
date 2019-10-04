cmake_minimum_required(VERSION 2.8.3)
project(controls)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  genmsg
)

add_message_files(
  FILES
  pose.msg
)

generate_messages(
   DEPENDENCIES
  std_msgs
#  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(controller src/controller.cpp)
target_link_libraries(controller
 ${catkin_LIBRARIES} 
  )
