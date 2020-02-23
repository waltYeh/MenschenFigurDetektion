cmake_minimum_required(VERSION 2.8.3)
project(fdetection)

# Build options
option(USE_OPENMP      "Set to ON to build use openmp" ON)
option(USE_SSE         "Set to ON to build use SSE"  ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")  
if(WIN32 AND NOT CYGWIN)  
    set(HOME $ENV{PROFILE})  
else()  
    set(HOME $ENV{HOME})  
endif()  

# Use SSE
if (USE_SSE)
    add_definitions(-DUSE_SSE)
    message(STATUS "Use SSE")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse4.1")
endif()

# Use OpenMP
if (USE_OPENMP)
    find_package(OpenMP QUIET)
    if (OPENMP_FOUND)
        message(STATUS "Use OpenMP")
        add_definitions(-DUSE_OPENMP)
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    endif()
endif()
  
find_package(catkin REQUIRED COMPONENTS  
 roscpp  
 rospy  
 std_msgs  
 sensor_msgs  
 cv_bridge  
 image_transport
 message_generation  
)  

add_message_files(
 FILES
 output.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

set(OpenCV_DIR /opt/ros/kinetic/include) 
find_package(OpenCV REQUIRED)  
message("PB->OPENCV = ${OpenCV_INCLUDE_DIRS}")



catkin_package(CATKIN_DEPENDS  
roscpp  
std_msgs  
sensor_msgs
message_runtime  
)  

include_directories(  
include  
${OpenCV_INCLUDE_DIRS}  
${catkin_INCLUDE_DIRS}  
)  
  

  
### find files  
  
file(GLOB_RECURSE HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h*)  
file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c*)  
  
add_executable(fdetect  
${HEADER_FILES}  
${SOURCE_FILES}  
) 
add_dependencies(fdetect ${catkin_EXPORTED_TARGETS})
add_dependencies(fdetect fdetection_generate_messages_cpp)
target_link_libraries(fdetect  
${catkin_LIBRARIES}  
${OpenCV_LIBRARIES}   
)  

