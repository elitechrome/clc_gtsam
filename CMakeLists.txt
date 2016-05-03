project(clc_gtsam)
cmake_minimum_required(VERSION 2.6)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(GTSAM)
#find_package(Eigen3 REQUIRED)
find_package( OpenCV )

#include_directories(EIGEN3_INCLUDE_DIR)
include_directories(/usr/local/Cellar/eigen/3.2.7/include/eigen3)

include_directories(CLC)
add_library(clc CLC/CLC.cpp)

include_directories(${GTSAM_INCLUDE_DIRS})
include_directories(/opt/intel/mkl/include)

add_executable(clc_test src/main.cpp)
target_link_libraries(clc_test gtsam ${OpenCV_LIBS} clc)
