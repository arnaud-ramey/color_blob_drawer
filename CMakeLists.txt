project( color_blob_drawer )
cmake_minimum_required(VERSION 2.8)
# Debug, Release, RelWithDebInfo and MinSizeRe
set(CMAKE_BUILD_TYPE RelWithDebInfo)
SET(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra") # add extra warnings

find_package( OpenCV REQUIRED )

add_library(disjoint_sets2 disjoint_sets2.cpp
                           disjoint_sets2.h sort_utils.h comp_labeller_interface.h)
target_link_libraries(disjoint_sets2 ${OpenCV_LIBS})

add_executable( color_blob_drawer color_blob_drawer.cpp timer.h)
target_link_libraries( color_blob_drawer disjoint_sets2)
