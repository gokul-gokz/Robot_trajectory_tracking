cmake_minimum_required(VERSION 2.8.9)
project(Robot_IK_trajectory_tracking)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(
        include
        ${Eigen3_INCLUDE_DIRS}
)
add_executable(Robot src/Robot.cpp)
target_link_libraries (Robot Eigen3::Eigen)

#include_directories( include ${OpenCV_INCLUDE_DIRS} )
#add_executable(harris_corner harris.cpp)
#target_link_libraries( harris_corner ${OpenCV_LIBS})

