cmake_minimum_required(VERSION 2.8.12)

PROJECT(opencv_correspondence)

if(NOT OpenCV_DIR)
  set(OpenCV_DIR </usr/local/include/opencv/)
endif()
find_package( OpenCV REQUIRED COMPONENTS  )
find_package(Qt5Core)

find_package(PCL 1.2 REQUIRED COMPONENTS visualization keypoints io registration common)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

include_directories( ${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(${PROJECT_NAME} main.cpp)

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} Qt5::Core)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
