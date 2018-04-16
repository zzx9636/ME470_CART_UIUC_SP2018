cmake_minimum_required(VERSION 2.8.3)
project(me470_ros)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall   -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  sensor_msgs
  geometry_msgs
  tf
  cv_bridge
  std_msgs
  cmake_modules
)

## System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

add_custom_target(Apriltag2 ALL COMMAND make 
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/april_lib
)
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
  #DEPENDS OpenCV
)
## Declare a C++ library
add_library(cart_state src/cart_state.cpp)
target_link_libraries(cart_state
     ${catkin_LIBRARIES}
)

add_library(april_detector
   src/april_detector.cpp
)
target_link_libraries(april_detector
   ${catkin_LIBRARIES}
   ${PROJECT_SOURCE_DIR}/include/april_lib/libapriltag.so
 )
add_library(ImageConverter
   src/ImageConverter.cpp
)

target_link_libraries(ImageConverter
    ${catkin_LIBRARIES}
    april_detector   
)

 add_executable(me470_ros_node src/me470_ros_node.cpp)
 target_link_libraries(me470_ros_node
   ${catkin_LIBRARIES}
   ImageConverter
   april_detector
   cart_state
 )

