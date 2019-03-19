cmake_minimum_required(VERSION 3.4)
project(Baseball_Tracker)

find_package( OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
        ${EIGEN3_INCLUDE_DIRS}
)

add_library(Tracker STATIC
        tracker.h
        tracker.cpp
)

target_link_libraries(Tracker ${OpenCV_LIBS})
