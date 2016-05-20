project(clc_gtsam)
cmake_minimum_required(VERSION 2.6)
#set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
##ROS catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  roscpp
  sensor_msgs
  tf
  image_transport
  cv_bridge
)
find_package(cmake_modules REQUIRED)
## Auxilary Libraries
if(UNIX AND NOT APPLE)
       set(LINUX TRUE)
   endif()

   # if(NOT LINUX) should work, too, if you need that
   if(LINUX)
       message(STATUS ">>> Linux")
       find_package(Eigen REQUIRED)
#       include_directories(EIGEN3_INCLUDE_DIR)
       include_directories(.)

   else()
       message(STATUS ">>> Not Linux")
       include_directories(/usr/local/Cellar/eigen/3.2.7/include/eigen3)
   endif()

find_package(GTSAM)
find_package(OpenCV 3.1.0 REQUIRED)
##Build
include_directories(CLC)
add_library(clc CLC/CLC.cpp)

include_directories(${GTSAM_INCLUDE_DIRS})
#include_directories(/opt/intel/mkl/include)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)


catkin_package(
 INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIRS} 
#/opt/intel/mkl/include
 LIBRARIES clc_test_gazebo
 CATKIN_DEPENDS gazebo_ros geometry_msgs roscpp sensor_msgs tf cv_bridge image_transport
 DEPENDS system_lib eigen
)

## Declare a ros cpp executable
add_executable(clc_test_gazebo_node src/clc_test_gazebo_node.cpp)
add_executable(clc_test_zed_node src/clc_test_zed_node.cpp)
add_executable(clc_test src/main.cpp)

target_link_libraries(clc
  ${OpenCV_LIBS}
)
target_link_libraries(clc_test
    gtsam
    ${OpenCV_LIBS}
    clc
)
target_link_libraries(clc_test_zed_node
    clc
    gtsam
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)
target_link_libraries(clc_test_gazebo_node
    clc
    gtsam
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)
