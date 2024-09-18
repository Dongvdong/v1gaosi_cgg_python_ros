#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Time.h>

#include <queue>
#include <mutex>
#include <thread>
#include <iostream>

#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <gaosi_img_pose_flag/PoseImgFlagMsg.h> // 更换为你包的名字

#include <Eigen/Dense>
#include <Eigen/Geometry> // For Quaterniond
 




// Global variables
std::queue<gaosi_img_pose_flag::PoseImgFlagMsg::ConstPtr> data_queue;
std::mutex queue_mutex;



// 用于过于早位姿的节点
Eigen::Quaterniond Q_c2w ;
Eigen::Vector3d t_c2w;




void publishPose(ros::Publisher& pose_pub, std_msgs::Float64 flag_,Eigen::Quaterniond &quat, Eigen::Vector3d &t)
{
    
    cv::Mat image_;
    //std_msgs::Float64 flag_;
    geometry_msgs::Pose pose_msg;

    pose_msg.position.x = t[0]; // 示例位置
    pose_msg.position.y = t[1];
    pose_msg.position.z = t[2];
    pose_msg.orientation.x = quat.x(); // 示例姿态
    pose_msg.orientation.y = quat.y();
    pose_msg.orientation.z = quat.z();
    pose_msg.orientation.w = quat.w();
 
 
 
    ROS_INFO("send Pose - x: %f, y: %f, z: %f",
                pose_msg.position.x,
                pose_msg.position.y,
                pose_msg.position.z);


    gaosi_img_pose_flag::PoseImgFlagMsg pose_img_flag_msg;
    //pose_msg.image = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_).toImageMsg();
    pose_img_flag_msg.flag = flag_;
    pose_img_flag_msg.pose = pose_msg;

    // 设置当前时间戳
    ros::Time current_time = ros::Time::now();
    pose_img_flag_msg.timestamp = std_msgs::Time(); // 初始化时间戳
    pose_img_flag_msg.timestamp.data = current_time; // 设置当前时间


    // 发布PoseStamped消息
    pose_pub.publish(pose_img_flag_msg);
}



// Callback function to handle incoming messages
void render_callback(const gaosi_img_pose_flag::PoseImgFlagMsg::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(queue_mutex);
    data_queue.push(msg);
}

// Thread function to process the queue
void processQueue()
{
    while (ros::ok())
    {
        std::queue<gaosi_img_pose_flag::PoseImgFlagMsg::ConstPtr> local_queue;

        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            std::swap(local_queue, data_queue); // Safely access the queue
        }

        while (!local_queue.empty())
        {
            auto msg = local_queue.front();
            local_queue.pop();

          
            // 将ROS图像消息转换为OpenCV图像
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg->image, sensor_msgs::image_encodings::BGR8);
            cv::imshow("Rec_Image", cv_ptr->image);
            cv::waitKey(1); 
            
            // Process the message
            ROS_INFO("Processing flag: %.2f", msg->flag.data);
            ROS_INFO("Processing pose: Position(%.2f, %.2f, %.2f), Orientation(%.2f, %.2f, %.2f, %.2f)",
                     msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
                     msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        }

        // Optional: Sleep to avoid busy waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void spinThread()
{
    ros::spin();// 处理回调函数积累消息
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_pose_processor");
    ros::NodeHandle nh;

    ros::Subscriber sub;
    ros::Publisher pose_pub;

    // Initialize the subscriber
    sub = nh.subscribe("render/image_pose_topic", 10, render_callback);
    pose_pub = nh.advertise<gaosi_img_pose_flag::PoseImgFlagMsg>("slam/image_pose_topic", 10);

    // Start a thread to run ros::spin()
    std::thread spin_thread(spinThread); // 处理rospin的线程

    // Create a thread to process the queue
    std::thread processing_thread(processQueue); // 处理接受消息的线程


    Q_c2w = Eigen::Quaterniond::Identity();;
    t_c2w={0,0,0.1};
    std::string control_mode="auto";// 自动 auto  手动 hand
 


    ros::Rate rate(1);// Hz 频率


    if(control_mode=="auto"){
        // 定时器每秒调用一次
        ros::Rate rate(1);
        double i =0;
        while (ros::ok())
        {  
            i=i+0.1;
            if(i>3)i=0;

            t_c2w={i,0,i};
            std_msgs::Float64 Msg_id; // 创建 Float64 消息
            Msg_id.data = i;// 将 double 值赋给消息的 data 成员
        
            // todo 从slam获取想要的位姿
            publishPose(pose_pub,Msg_id,Q_c2w,t_c2w);
            

            rate.sleep();
        }
    }



    
    // Join the processing thread before exiting
    if (processing_thread.joinable())
    {
        processing_thread.join();
    }

    // Join the spin thread before exiting
    if (spin_thread.joinable())
    {
        spin_thread.join();
    }




    return 0;
}
