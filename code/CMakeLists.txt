cmake_minimum_required(VERSION 3.5)
project(code)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
find_package(PCL 1.8 REQUIRED)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${CV_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_directories(${CV_LIBRARY_DIRS})

link_libraries(${OpenCV_LIBS})
link_libraries(${PCL_LIBRARIES})

add_executable(code compare_trajectory_velo_cam.cpp load_velodyne.cpp load_stereo_cam.cpp keypoint_compare.cpp)
#add_executable(code match_velo_narf_frames.cpp load_velodyne.cpp load_stereo_cam.cpp)# keypoint_compare.cpp keypoint_compare.h)
#add_executable(code matrix.cpp octo_model.cpp octo_model.h proc_velodyne.cpp)
#add_executable (range_image_visualization range_image_visualization.cpp)
#target_link_libraries (range_image_visualization ${PCL_LIBRARIES})


