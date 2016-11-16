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

# boost
find_package(Boost REQUIRED COMPONENTS filesystem system signals thread)
if(NOT Boost_INCLUDE_DIRS)
  message(FATAL "boost required but not supported")
else()
  include_directories(include ${Boost_INCLUDE_DIRS})
endif()

set(GENERATE_SRV)
if(GENERATE_SRV)
  # auto-add messages
  add_message_files(
    FILES
  )
  # auto-add services
  add_service_files(
    FILES
  )
  generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
  )
endif()

add_definitions(-std=c++11)
include_directories(lib)

add_executable(points_sample
  examples/points.cc
  lib/KinectInterface.cc
  )
target_link_libraries(points_sample
  ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(image_sample
  examples/image.cc
  lib/KinectInterface.cc
  )
target_link_libraries(image_sample
  ${catkin_LIBRARIES} ${Boost_LIBRARIES})

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

add_executable(person_detection_sample
  examples/person_detection.cc
  lib/KinectInterface.cc
  )
target_link_libraries(person_detection_sample
  ${catkin_LIBRARIES} ${Boost_LIBRARIES})
