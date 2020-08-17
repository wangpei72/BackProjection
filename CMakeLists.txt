cmake_minimum_required(VERSION 3.10.2)
project(untitled)

set(CMAKE_CXX_STANDARD 14)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(OpenCV 3 REQUIRED)
set(TARGET_LIB
        "/usr/lib/x86_64-linux-gnu/libGL.so"
        "/usr/lib/x86_64-linux-gnu/libGLU.so"
        "/usr/lib/x86_64-linux-gnu/libglut.so"
        )
include_directories(
        ${OpenCV_INCLUDE_DIRS}

        "/usr/include/eigen3/"
)

add_executable(untitled main.cpp BackProject.cpp BackProject.h)
target_link_libraries(untitled ${OpenCV_LIBS})

add_executable(visualizer visualize.cpp )
target_link_libraries(visualizer ${TARGET_LIB})