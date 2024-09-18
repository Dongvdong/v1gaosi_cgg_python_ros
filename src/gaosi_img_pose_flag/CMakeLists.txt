cmake_minimum_required(VERSION 3.0.2)
project(gaosi_img_pose_flag)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  cv_bridge
  message_filters # 消息同步
  image_transport
  std_msgs # 自定义消息
  message_generation # 自定义消息
)

# 自定义消息
add_message_files(
  FILES
  PoseImgFlagMsg.msg
)

# 自定义消息
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs sensor_msgs cv_bridge std_msgs message_runtime
  DEPENDS Boost
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  "/usr/local/include/eigen3"
)



# # 编译发布节点
# add_executable(image_pose_flag_publisher src/image_pose_flag_publisher.cpp)
# # 自定义消息引用
# add_dependencies(image_pose_flag_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(image_pose_flag_publisher
#   ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

add_executable(image_pose_flag_subscriber src/image_pose_flag_subscriber.cpp)
add_dependencies(image_pose_flag_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(image_pose_flag_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
