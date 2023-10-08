cmake_minimum_required(VERSION 3.4)
project(full_linear_wheel_odometry_factor_example)

set(CMAKE_CXX_FLAGS  "-std=c++17")

#ubuntu
include_directories(/usr/include/eigen3)

# Find package
find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

add_executable(full_linear_wheel_odometry_factor_example full_linear_wheel_odometry_factor_example.cpp)
target_link_libraries(full_linear_wheel_odometry_factor_example gtsam)
# Add include dirs and link libraries
target_include_directories(full_linear_wheel_odometry_factor_example PUBLIC)
