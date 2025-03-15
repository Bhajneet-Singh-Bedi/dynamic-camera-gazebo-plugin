cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(camera_position)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-transport11 REQUIRED)
find_package(ignition-msgs8 REQUIRED)
find_package(sdformat12 REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(IGN_GAZEBO_VER ${ignition-gazebo6_VERSION_MAJOR})
set(IGN_TRANSPORT_VER ${ignition-transport11_VERSION_MAJOR})
set(IGN_MSGS_VER ${ignition-msgs8_VERSION_MAJOR})
set(SDF_VER ${sdformat12_VERSION_MAJOR})

add_library(ros2_topic_subscriber SHARED ros2_topic_subscriber.cc)

target_link_libraries(ros2_topic_subscriber
  ignition-gazebo${IGN_GAZEBO_VER}::ignition-gazebo${IGN_GAZEBO_VER}
  ignition-transport${IGN_TRANSPORT_VER}::ignition-transport${IGN_TRANSPORT_VER}
  ignition-msgs${IGN_MSGS_VER}::ignition-msgs${IGN_MSGS_VER}
  sdformat${SDF_VER}::sdformat${SDF_VER}
)

target_include_directories(ros2_topic_subscriber PUBLIC
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)
target_link_libraries(ros2_topic_subscriber
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)

install(TARGETS ros2_topic_subscriber
  DESTINATION lib/${PROJECT_NAME}
)