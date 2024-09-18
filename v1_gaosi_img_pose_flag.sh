#!/bin/bash
#外部给与执行权限
#sudo chmod +x run_ros_nodes.sh
# 定义 ROS 安装路径  #安装时候添加到系统路径了 不需要每次都source
ROS_SETUP="/opt/ros/noetic/setup.bash" 
# 定义工作目录路径  自己的工程没有加到系统路径，每次需要source
WORKSPACE_DIR="/home/r9000k/v2_project/gaosi_slam/ros/ros_cgg"


conda_envs="/home/r9000k/anaconda3" # 修改2-1 自己的conda 安装路径
#ROS_cv_briage_dir="/home/r9000k/v1_software/opencv/catkin_ws_cv_bridge/devel/setup.bash" # 修改2-2 自己编译的cv_briage包节点，貌似不用也行 制定了依赖opencv3.4.9 而非自带4.2
#echo $ROS_cv_briage_dir
conda_envs_int=$conda_envs"/etc/profile.d/conda.sh" # 不用改 conda自带初始化文件
echo $conda_envs_int
conda_envs_bin=$conda_envs"/envs/gaussian_splatting/bin" # 不用改 conda自带python安装位置 脚本中需要指定是conda特定的环境python而不是系统默认的
echo $conda_envs_bin
ROS_SETUP="/opt/ros/noetic/setup.bash" #不用改  安装时候添加到系统路径了 不需要每次都source 这里留着


#指定目录

# 启动 ROS Master
echo "Starting ROS 总结点..."
gnome-terminal -- bash -c "\
cd $WORKSPACE_DIR; source devel/setup.bash; \
roscore; \
exec bash"

# 等待 ROS Master 启动
sleep 3

# 运行 C++ 发布节点
echo "Running C++ 发布节点..."
gnome-terminal -- bash -c "\
cd $WORKSPACE_DIR; source devel/setup.bash; \
rosrun gaosi_img_pose_flag image_pose_flag_publisher; \
exec bash"

echo "Running C++ 订阅节点..."
gnome-terminal -- bash -c "\
cd $WORKSPACE_DIR; source devel/setup.bash; \
rosrun gaosi_img_pose_flag image_pose_flag_subscriber; \
exec bash"


# 运行python节点
# python_DIR="${WORKSPACE_DIR}/src/gaosi_img_pose_flag/src"

# echo "Running python image_pose_publisher发布节点..."
# gnome-terminal -- bash -c " \
# cd $WORKSPACE_DIR; source devel/setup.bash; \
# source $conda_envs_int; \ 
# conda activate gaussian_splatting ; \
# cd $python_DIR; \
# /usr/bin/python3 image_pose_publisher.py; \
# exec bash"


# # 运行python节点

# echo "Running python image_pose_subscriber订阅节点..."
# gnome-terminal -- bash -c " \
# cd $WORKSPACE_DIR; source devel/setup.bash; \
# source $conda_envs_int; \ 
# conda activate gaussian_splatting ; \
# cd $python_DIR; \
# /usr/bin/python3 image_pose_subscriber.py; \
# exec bash"
