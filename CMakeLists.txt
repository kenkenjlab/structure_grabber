#### CMake Settings ####

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
cmake_policy(SET CMP0015 OLD)	#// Do not add relative path if the designated path is not exist
add_definitions(-DUNICODE -D_UNICODE)	#// Set Unicode, not MultiBytes
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


#### Solution Settings ####		* MODIFY HERE*

# OpenCV
find_package( OpenCV REQUIRED )

# OpenNI2
include(FindPkgConfig)
pkg_check_modules(OPENNI libopenni)

#### Solution Settings ####

project(structure_grabber)


#### Includes and Libraries ####

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#### Required files ####

set(ONI2_GRABBER oni2_grabber.h oni2_grabber.cpp)
set(STRUCTURE_GRABBER structure_grabber.h structure_grabber.cpp ${ONI2_GRABBER})


#### Projects Settings ####

add_executable(structure_depth_image_viewer structure_depth_image_viewer.cpp ${STRUCTURE_GRABBER})
target_link_libraries(structure_depth_image_viewer ${OPENNI2_LIB} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(structure_colored_depth_image_viewer structure_colored_depth_image_viewer.cpp ${STRUCTURE_GRABBER})
target_link_libraries(structure_colored_depth_image_viewer ${OPENNI2_LIB} ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(structure_point_cloud_viewer structure_point_cloud_viewer.cpp ${STRUCTURE_GRABBER})
target_link_libraries(structure_point_cloud_viewer ${OPENNI2_LIB} ${OpenCV_LIBS} ${PCL_LIBRARIES})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
