cmake_minimum_required(VERSION 3.16)

project(task)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)

set(HEADERS ./include/windmill.hpp)
include_directories(./include ${OpenCV_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

add_executable(task main.cpp ./src/windmill.cpp ${HEADERS})
target_link_libraries(task ${OpenCV_LIBS} ${CERES_LIBRARIES})