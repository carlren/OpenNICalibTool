cmake_minimum_required(VERSION 2.8)
project(RGBDCalib)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

set(OPEN_NI_ROOT /home/carl/Work/3rdparty/OpenNI2)

find_package(OpenCV REQUIRED)
find_package(OpenNI REQUIRED)

IF(MSVC_IDE)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS)
  add_definitions(-DUSING_CMAKE=1)
ELSE(MSVC_IDE)
  set(CFLAGS_WARN "-Wall -Wextra -Wno-unused-parameter -Wno-strict-aliasing")
  set(CMAKE_CXX_FLAGS "-fPIC -O3 -march=native ${CFLAGS_WARN} ${CMAKE_CXX_FLAGS}")
ENDIF(MSVC_IDE)


include_directories("/home/carl/Work/3rdparty")
include_directories("/home/carl/Work/3rdparty/Eigen3")
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${OpenNI_INCLUDE_DIR})

add_executable(RGBDCalib 
main.cpp 
RGBDCalib.h 
RGBDCalib.cpp 
OpenNIEngine.h
OpenNIEngine.cpp)

target_link_libraries(RGBDCalib 
${OpenCV_LIBS} 
${OpenNI_LIBRARY})

