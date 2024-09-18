#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <map>
#include <utility> // for std::pair
#include <mutex>
#include <thread>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry> // For Quaterniond

// 存储图像和Pose的队列
std::map<ros::Time, std::pair<sensor_msgs::ImageConstPtr, geometry_msgs::PoseStampedConstPtr>> message_queue;
std::mutex queue_mutex; // 互斥锁，用于保护message_queue

ros::Publisher pose_pub;
ros::Subscriber image_sub;
ros::Subscriber pose_sub;

void imageCallback(const sensor_msgs::Image::ConstPtr& img_msg)
{


    std::lock_guard<std::mutex> lock(queue_mutex); // 上锁

    // 查找对应的Pose
    auto it = message_queue.find(img_msg->header.stamp);
    double seconds = img_msg->header.stamp.toSec();
    
    if (it != message_queue.end())
    {

        // 找到匹配的Pose
        ROS_INFO("图像数据到达，存在匹配的pose，更新存入图像数据 %f", seconds);  
        // 这里可以处理图像和Pose
        // std::pair<sensor_msgs::ImageConstPtr, geometry_msgs::PoseStampedConstPtr> paired_data = it->second;
        // processImageAndPose(paired_data.first, paired_data.second);
        message_queue[img_msg->header.stamp].first = img_msg;
        // 移除匹配的消息对
        //message_queue.erase(it);
    }
    else
    {
        ROS_INFO("图像数据到达，没有匹配的pose，更新存入图像数据 %f", seconds);  
        // 如果找不到对应的Pose，将图像存入队列
        message_queue[img_msg->header.stamp].first = img_msg;
    }
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
{
    //ROS_INFO("Received Pose data");

    std::lock_guard<std::mutex> lock(queue_mutex); // 上锁


    Eigen::Vector3d vio_t(pose_msg->pose.position.x, pose_msg->pose.position.y, pose_msg->pose.position.z);
    Eigen::Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.orientation.w;
    vio_q.x() = pose_msg->pose.orientation.x;
    vio_q.y() = pose_msg->pose.orientation.y;
    vio_q.z() = pose_msg->pose.orientation.z;



    // 打印 Pose 信息
    ROS_INFO("Received Pose - x: %f, y: %f, z: %f",
                pose_msg->pose.position.x,
                pose_msg->pose.position.y,
                pose_msg->pose.position.z);

    //td::unique_lock<std::mutex> lock(queue_mutex);
    //data_queue.push(std::make_pair(image, pose_msg));
    //lock.unlock();

    // 查找对应的图像
    auto it = message_queue.find(pose_msg->header.stamp);
    double seconds = pose_msg->header.stamp.toSec();
    if (it != message_queue.end()) // 
    {
        // 找到匹配的图像
        ROS_INFO("pose数据到达，存在匹配的图像，更新存入pose数据  %f", seconds);  
        // 这里可以处理图像和Pose
        // std::pair<sensor_msgs::ImageConstPtr, geometry_msgs::PoseStampedConstPtr> paired_data = it->second;
        // processImageAndPose(paired_data.first, paired_data.second);
        message_queue[pose_msg->header.stamp].second = pose_msg;
        // 移除匹配的消息对
        //message_queue.erase(it);
    }
    else
    {
        ROS_INFO("pose数据到达，没有匹配的图像，暂时存入pose数据  %f", seconds);  
        // 如果找不到对应的图像，将Pose存入队列
        message_queue[pose_msg->header.stamp].second = pose_msg;
    }
}

void publishPose()
{

    
    // // 旋转角度 (90 度转换为弧度)
    // double theta = M_PI / 2; // 90 度是 π/2 弧度
    // // 绕 z 轴旋转的旋转矩阵
    // R << cos(theta), -sin(theta), 0,
    //      sin(theta),  cos(theta), 0,
    //      0,           0,          1;

    Eigen::Matrix4d WGPS_T_WVIO;// vo坐标变换到gps坐标
    WGPS_T_WVIO <<  0, -1, 0, 1,
                    1,  0, 0, 2,
                    0,  0, 1, 3,
                    0,  0, 0, 1;

    Eigen::Matrix3d R = WGPS_T_WVIO.block<3,3>(0,0); // 提取3x3的旋转矩阵
    Eigen::Vector3d t = WGPS_T_WVIO.block<3,1>(0,3); // 提取平移向量

    Eigen::Quaterniond quat(R); // 使用旋转矩阵构造四元数


    // 输出结果
    // std::cout << "Rotation Matrix R:\n" << R << std::endl;
    // std::cout << "Translation Vector t:\n" << t << std::endl;
    // std::cout << "Quaternion:\n" 
    //           << " w: " << quat.w() 
    //           << " x: " << quat.x()
    //           << " y: " << quat.y()
    //           << " z: " << quat.z()
    //           << std::endl; // 四元数的系数  


    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now(); // 设置时间戳
    pose_msg.header.frame_id = "base_link"; // 设置坐标框架
    pose_msg.pose.position.x = t[0]; // 示例位置
    pose_msg.pose.position.y = t[1];
    pose_msg.pose.position.z = t[2];
    pose_msg.pose.orientation.x = quat.x(); // 示例姿态
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();
    pose_msg.pose.orientation.w = quat.w();



    ROS_INFO("send Pose - x: %f, y: %f, z: %f",
                pose_msg.pose.position.x,
                pose_msg.pose.position.y,
                pose_msg.pose.position.z);

    // 发布PoseStamped消息
    pose_pub.publish(pose_msg);
}


void displayThread()
{
    cv::namedWindow("Image", cv::WINDOW_AUTOSIZE); // 创建一个窗口用于显示图像

    while (ros::ok())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 暂停以减少CPU使用率

        std::lock_guard<std::mutex> lock(queue_mutex); // 上锁以保护对 message_queue 的访问

        if (message_queue.empty()) {
            ROS_INFO("Message queue is empty.");
            cv::waitKey(1000);
        } 
        else {
            for (auto it = message_queue.begin(); it != message_queue.end();)
            {
                const auto& [timestamp, paired_data] = *it;
                const sensor_msgs::ImageConstPtr& img_msg = paired_data.first;
                const geometry_msgs::PoseStampedConstPtr& pose_msg = paired_data.second;
                double seconds = timestamp.toSec();

                if (img_msg && pose_msg)
                //if(1)
                {   
                    ROS_INFO("Message queue img_msg和pose_msg数据同时满足，刷新图像 %f", seconds);  
                    try
                    {   
                        // 将ROS图像消息转换为OpenCV图像
                        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
                        cv::imshow("Image", cv_ptr->image);

                        // 打印Pose信息
                        ROS_INFO("Pose - x: %f, y: %f, z: %f", 
                                 pose_msg->pose.position.x, 
                                 pose_msg->pose.position.y, 
                                 pose_msg->pose.position.z);

                        cv::waitKey(1); // 等待1毫秒以处理图像显示
                    }
                    catch (cv_bridge::Exception& e)
                    {
                        ROS_ERROR("cv_bridge exception: %s", e.what());
                        cv::waitKey(1); 
                    }

                    // 移除已处理的消息对
                    it = message_queue.erase(it);
                }
                else
                {   
                    //ROS_INFO("Message queue 数据不同时满足. 切换下一个");
                    ++it;
                }
            }
        }
    }//while

    cv::destroyAllWindows(); // 关闭所有OpenCV窗口
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "combined_node");
    ros::NodeHandle nh;

    // 创建发布者
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_topic", 10);

    // 创建订阅者
    image_sub = nh.subscribe("image_topic", 1, imageCallback);
    pose_sub = nh.subscribe("pose_image_topic", 1, poseCallback);

    // 定时器，用于定期发布PoseStamped消息
    // ros::Timer timer = nh.createTimer(ros::Duration(1.0), [](const ros::TimerEvent&)
    // {
    //     publishPose();
    // });

    //ros::spin();

    // 启动显示线程
    std::thread display_thread(displayThread);
    // &GlobalOptimization::optimize 是一个指向 GlobalOptimization 类成员函数 optimize 的指针，用于指定新线程中要调用的具体函数。
    // this 是指向当前 GlobalOptimization 对象的指针，用于提供给 optimize 成员函数，以便它可以操作当前对象的状态。
    //std::thread threadOpt = std::thread(&GlobalOptimization::optimize, this);

    // 定时器每秒调用一次
    ros::Rate rate(1);
    while (ros::ok())
    {
        publishPose();
        // 调用ROS处理回调函数
        ros::spinOnce();
        rate.sleep();
    }

    // 确保线程在节点退出时正确终止
    display_thread.join();

    return 0;
}
