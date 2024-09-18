cmake_minimum_required(VERSION 3.0.2)
project(image_gps_package)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge
  DEPENDS Boost
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# 编译发布节点
add_executable(image_gps_publisher src/image_gps_publisher.cpp)
target_link_libraries(image_gps_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

# 编译接受节点
add_executable(image_gps_subscriber src/image_gps_subscriber.cpp)
target_link_libraries(image_gps_subscriber
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)
