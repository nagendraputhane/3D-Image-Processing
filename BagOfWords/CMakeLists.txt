cmake_minimum_required(VERSION 2.8.12)

project(BagOfWords)

if(NOT OpenCV_DIR)
  set(OpenCV_DIR /usr/local/lib/opencv/)
endif()

find_package( OpenCV REQUIRED )
find_package(Qt5Core)
FIND_PACKAGE(DBoW2 REQUIRED)
FIND_PACKAGE(DLib REQUIRED)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

include_directories( ${OpenCV_INCLUDE_DIRS})

add_executable(${PROJECT_NAME} "main.cpp")
TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${OpenCV_LIBS} DBoW2 DLib)
target_link_libraries(${PROJECT_NAME} Qt5::Core)

