#!/bin/bash

#外部给与执行权限
#sudo chmod +x run_ros_nodes.sh
# conda activate gaussian_splatting

WORKSPACE_DIR="/home/r9000k/v2_project/gaosi_slam/ros/ros_cgg" # 修改1-1 自己创建的ros节点工程catkin_make根目录
python_DIR="/home/r9000k/v2_project/gaosi_slam/ros/ros_cgg/src/gaosi_img_pose_flag/src" # 修改1-2 自己创建的python脚本位置

data_dir="/home/r9000k/v2_project/data/NWPU"
config_DIR="/home/dongdong/2project/0data/NWPU/FHY_config/GNSS_config.yaml" # 修改1-3 数据集

conda_envs="/home/r9000k/anaconda3" # 修改2-1 自己的conda 安装路径
# ROS_cv_briage_dir = "/home/r9000k/v1_software/opencv/catkin_ws_cv_bridge/devel/setup.bash" # 修改2-2 自己编译的cv_briage包节点，貌似不用也行 制定了依赖opencv3.4.9 而非自带4.2
# echo $ROS_cv_briage_dir
conda_envs_int=$conda_envs"/etc/profile.d/conda.sh" # 不用改 conda自带初始化文件
echo $conda_envs_int
conda_envs_bin=$conda_envs"/envs/gaussian_splatting/bin" # 不用改 conda自带python安装位置 脚本中需要指定是conda特定的环境python而不是系统默认的
echo $conda_envs_bin
ROS_SETUP="/opt/ros/noetic/setup.bash" #不用改  安装时候添加到系统路径了 不需要每次都source 这里留着
#指定目录

# 启动 ROS Master 不用改
echo "Starting ROS 总结点..."
gnome-terminal -- bash -c "\
cd $WORKSPACE_DIR; source devel/setup.bash; \  
roscore; \
exec bash"

# 等待 ROS Master 启动
sleep 3

echo "Running C++ 订阅节点..."
gnome-terminal -- bash -c "\
cd $WORKSPACE_DIR; source devel/setup.bash; \
rosrun gaosi_img_pose_flag image_pose_flag_subscriber; \
exec bash"


# source /home/r9000k/v2_project/gaosi_slam/ros/ros_cgg/devel/setup.bash 
# 运行 python 渲染图节点
# source conda_envs_int 和 source ROS_cv_briage_dir 非必要，但是考虑到脚本经常因为系统环境默认变量找不到导致的路径问题，这里还是强制给了也便于学习了解执行流程。
echo "Running python 订阅节点..."
echo "1 激活conda本身(脚本执行需要) 2 激活conda环境  3运行python 节点 并跟上输入参数[训练模型保存根目录，指定要使用的模型训练次数,要测试的模型精度模式]"
gnome-terminal -- bash -c "\
source $conda_envs_int; \ 
cd $WORKSPACE_DIR; source devel/setup.bash; \
conda activate gaussian_splatting ; \
cd $python_DIR; \
python3 v1_image_pose_subscriber.py \
-m $data_dir/gs_out/train1_out_sh0_num30000 \
--iteration 30000 \
--models baseline ;\
exec bash"

