cmake_minimum_required(VERSION 3.1.0)

project(alldata)

#set( CMAKE_CXX_COMPILER "g++" )
set( CMAKE_BUILD_TYPE "Debug" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
# Enable C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)

include_directories("${PROJECT_INCLUDE_DIR}")
include_directories("${PROJECT_SOURCE_DIR}")
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(realsense2 REQUIRED)
FIND_PACKAGE(PCL REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${realsense2_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories( /include/ )
include_directories(/stairperception/)
include_directories(/rs2/)
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(rs2)
add_subdirectory(stairperception)

add_executable(result main.cpp)
target_link_libraries( result ${OpenCV_LIBS} ${realsense2_LIBRARY} rs2 stairperception)


