#### CMake Settings ####

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
cmake_policy(SET CMP0015 OLD)	#// Do not add relative path if the designated path is not exist
add_definitions(-DUNICODE -D_UNICODE)	#// Set Unicode, not MultiBytes


#### Solution Settings ####		* MODIFY HERE*

# OpenCV
set(OPENCV_INC_DIRS "C:/OpenCV2.4.6/install/include")
set(OPENCV_LIB_DIRS "C:/OpenCV2.4.6/install/lib")
set(OPENCV246_LIB_RELEASE optimized opencv_core246.lib optimized opencv_highgui246.lib optimized opencv_imgproc246.lib)
set(OPENCV246_LIB_DEBUG debug opencv_core246d.lib debug opencv_highgui246d.lib debug opencv_imgproc246d.lib)
set(OPENCV_LIB ${OPENCV246_LIB_RELEASE} ${OPENCV246_LIB_DEBUG})

# OpenNI2
set(OPENNI2_INC_DIRS "C:/Program Files (x86)/OpenNI2/Include")
set(OPENNI2_LIB_DIRS "C:/Program Files (x86)/OpenNI2/Lib")
set(OPENNI2_LIB OpenNI2.lib)

#### Solution Settings ####

project(structure_grabber)


#### Includes and Libraries ####

find_package(PCL 1.7 REQUIRED)
include_directories(include ${PCL_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS_PLUS} ${OPENCV_INC_DIRS} ${OPENNI2_INC_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OPENCV_LIB_DIRS} ${OPENNI2_LIB_DIRS})
add_definitions(${PCL_DEFINITIONS})


#### Required files ####

set(ONI2_GRABBER oni2_grabber.h oni2_grabber.cpp)
set(STRUCTURE_GRABBER structure_grabber.h structure_grabber.cpp ${ONI2_GRABBER})


#### Projects Settings ####

add_executable(structure_depth_image_viewer structure_depth_image_viewer.cpp ${STRUCTURE_GRABBER})
target_link_libraries(structure_depth_image_viewer ${OPENNI2_LIB} ${OPENCV_LIB})

add_executable(structure_colored_depth_image_viewer structure_colored_depth_image_viewer.cpp ${STRUCTURE_GRABBER})
target_link_libraries(structure_colored_depth_image_viewer ${OPENNI2_LIB} ${OPENCV_LIB})

add_executable(structure_point_cloud_viewer structure_point_cloud_viewer.cpp ${STRUCTURE_GRABBER})
target_link_libraries(structure_point_cloud_viewer ${OPENNI2_LIB} ${PCL_LIBRARIES})