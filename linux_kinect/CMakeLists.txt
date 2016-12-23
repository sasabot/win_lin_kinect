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
if(NOT PCL_INCLUDE_DIRS)
  message(WARNING "PCL not found, disabled")
else()
  include_directories(${PCL_INCLUDE_DIRS})
  link_directories(${PCL_LIBRARY_DIRS})
  add_definitions(${PCL_DEFINITIONS})
  set(FOUND_PCL 1)
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
  set(FOUND_OpenCV 1)
endif()

# add_message_files(
#   FILES
# )
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

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES kinect_interface windows_interface
  CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs std_msgs
)

# libraries

add_library(kinect_interface src/KinectInterface.cc)

if(FOUND_OpenCV)
  add_library(windows_interface src/WindowsInterface.cc)
endif()

# examples

add_executable(points_sample
  examples/points.cc
  src/KinectInterface.cc
  )
target_link_libraries(points_sample
  ${catkin_LIBRARIES})

add_executable(image_sample
  examples/image.cc
  src/KinectInterface.cc
  )
target_link_libraries(image_sample
  ${catkin_LIBRARIES})

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

if (FOUND_OpenCV)
  add_executable(image_centers_sample
    examples/image_centers.cc
    src/KinectInterface.cc
    )
  target_link_libraries(image_centers_sample
    ${catkin_LIBRARIES} ${OpenCV_LIBS})

  add_executable(ocr_sample
    examples/ocr.cc
    src/WindowsInterface.cc
    )
  target_link_libraries(ocr_sample
    ${catkin_LIBRARIES} ${OpenCV_LIBS})
endif()

if (FOUND_OpenCV AND FOUND_PCL)
  add_executable(image_bounds_sample
    examples/image_bounds.cc
    src/KinectInterface.cc
    )
  target_link_libraries(image_bounds_sample
    ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
endif()
