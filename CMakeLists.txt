cmake_minimum_required(VERSION 2.8)
project( color_blob_drawer )
# Debug, Release, RelWithDebInfo and MinSizeRe
set(CMAKE_BUILD_TYPE RelWithDebInfo)
find_package( OpenCV REQUIRED )


add_library(disjoint_sets2 disjoint_sets2.cpp)
target_link_libraries(disjoint_sets2 ${OpenCV_LIBS})

add_executable( color_blob_drawer color_blob_drawer.cpp )
target_link_libraries( color_blob_drawer ${OpenCV_LIBS} disjoint_sets2)
