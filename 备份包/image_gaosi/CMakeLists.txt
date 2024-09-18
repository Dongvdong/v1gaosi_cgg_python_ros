cmake_minimum_required(VERSION 3.0.2)
project(image_gaosi)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  cv_bridge
  message_filters # 消息同步
  image_transport
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs cv_bridge
  DEPENDS Boost
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  "/usr/local/include/eigen3"
)



# 编译发布节点
add_executable(image_pose_publisher src/image_pose_publisher.cpp)
target_link_libraries(image_pose_publisher
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)


