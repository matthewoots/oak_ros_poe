cmake_minimum_required(VERSION 3.10)
project(oak_ros)

# enable C++17 standard (current directory scope)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# configure build types and build c-flags
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

find_package(Threads REQUIRED)
find_package(OpenCV 4 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
)

# running dependency
catkin_package(
  # INCLUDE_DIRS 
  # LIBRARIES 
  # CATKIN_DEPENDS imu_vn_100
  # DEPENDS
)