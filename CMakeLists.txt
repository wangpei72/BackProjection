cmake_minimum_required(VERSION 3.16)
project(untitled)

set(CMAKE_CXX_STANDARD 14)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(OpenCV 3 REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}

        "/usr/include/eigen3/"
)

add_executable(untitled main.cpp BackProject.cpp BackProject.h)
target_link_libraries(untitled ${OpenCV_LIBS})