cmake_minimum_required(VERSION 2.8)
project(RGBDCalib)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake/")

#set(OPEN_NI_ROOT /home/carl/Work/3rdparty/OpenNI2/)

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

add_library(OpenNIEngine
OpenNIEngine.h
OpenNIEngine.cpp
)

add_executable(CalibCapture 
OpenNICapture.cpp 
)

add_executable(StereoCalib
StereoCalib.cpp)

add_executable(TestCalibResult
OpenNIEngine.h
OpenNIEngine.cpp
CalibTest.cpp)

target_link_libraries(OpenNIEngine
${OpenCV_LIBS} 
${OpenNI_LIBRARY}
)

target_link_libraries(CalibCapture 
OpenNIEngine
${OpenCV_LIBS} 
${OpenNI_LIBRARY}
)


target_link_libraries(StereoCalib 
${OpenCV_LIBS} 
)

target_link_libraries(TestCalibResult 
OpenNIEngine
${OpenCV_LIBS} 
${OpenNI_LIBRARY})

