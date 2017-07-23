cmake_minimum_required(VERSION 2.8.3)
project(linux_kinect)

# Dependencies

# catkin ROS
find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs sensor_msgs geometry_msgs
  message_generation
  )
if(NOT catkin_LIBRARIES)
  message(FATAL "catkin required but not supported")
endif()

# pcl
find_package(PCL 1.8)
if(NOT PCL_FOUND)
  message(WARNING "PCL not found, disabled")
else()
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
endif()

# opencv
find_package(OpenCV 3)
if(NOT OpenCV_FOUND)
  message(WARNING "opencv not found, disabled")
else()
  include_directories(${OpenCV_INCLUDE_DIRS})
  message(STATUS "OpenCV library status:")
  message(STATUS "    version: ${OpenCV_VERSION}")
  message(STATUS "    libraries: ${OpenCV_LIBS}")
  message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
endif()

# opencv contrib
include(CheckIncludeFileCXX)
check_include_file_cxx(opencv2/saliency.hpp FOUND_OpenCV_CONTRIB)
if(NOT FOUND_OpenCV_CONTRIB)
  message(WARNING "opencv contrib not found, disabled")
else()
  message(STATUS "found opencv contrib")
endif()

add_message_files(
  FILES
  Face.msg
  )
add_service_files(
  FILES
  Ocr.srv
  KinectRequest.srv
  KinectPoints.srv
  KinectImage.srv
  )
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  )

add_definitions(-std=c++11)
include_directories(include ${catkin_INCLUDE_DIRS})


# libraries

if(OpenCV_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES kinect_interface windows_interface
    CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs std_msgs
    )
  add_library(windows_interface src/WindowsInterface.cc)
else()
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES kinect_interface
    CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs std_msgs
    )
endif()

add_library(kinect_interface src/KinectInterface.cc)

add_executable(extract_rgb_from_depth
  nodes/extract_rgb_from_depth.cc
  )
target_link_libraries(extract_rgb_from_depth
  ${catkin_LIBRARIES})

add_executable(extract_rgb_from_depth_filtered
  nodes/extract_rgb_from_depth_filtered.cc
  )
target_link_libraries(extract_rgb_from_depth_filtered
  ${catkin_LIBRARIES})

add_executable(simple_axis_threshold
  nodes/simple_axis_threshold.cc
  )
target_link_libraries(simple_axis_threshold
  ${catkin_LIBRARIES})

# examples

add_executable(points_sample
  examples/points.cc
  )
target_link_libraries(points_sample
  ${catkin_LIBRARIES} kinect_interface)

add_executable(points_compressed_sample
  examples/points_compressed.cc
  )
target_link_libraries(points_compressed_sample
  ${catkin_LIBRARIES} kinect_interface)

add_executable(image_sample
  examples/image.cc
  )
target_link_libraries(image_sample
  ${catkin_LIBRARIES} kinect_interface)

add_executable(speech_matching_sample
  examples/speech_matching.cc
  )
target_link_libraries(speech_matching_sample
  ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(tts_sample
  examples/tts.cc
  )
target_link_libraries(tts_sample
  ${catkin_LIBRARIES} ${Boost_LIBRARIES})

if (OpenCV_FOUND AND FOUND_OpenCV_CONTRIB)
  add_executable(image_centers_sample
    examples/image_centers.cc
    )
  target_link_libraries(image_centers_sample
    ${catkin_LIBRARIES} ${OpenCV_LIBS} kinect_interface)
endif()

if (OpenCV_FOUND)
  add_executable(ocr_sample
    examples/ocr.cc
    )
  target_link_libraries(ocr_sample
    ${catkin_LIBRARIES} ${OpenCV_LIBS} windows_interface)
endif()

if (OpenCV_FOUND AND PCL_FOUND)
  add_executable(image_bounds_sample
    examples/image_bounds.cc
    )
  target_link_libraries(image_bounds_sample
    ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES}
    kinect_interface)
endif()
