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


int i = 0;
double x = 0, y = 0, z = 0;
double step_ = 0.1, step_theta = 1.0;
double theta_x = 0, theta_y = 0, theta_z = 0;
cv::Mat image_hand  = cv::Mat::zeros(480, 640, CV_8UC3); 


// 用于过于早位姿的节点
Eigen::Quaterniond Q_c2w ;
Eigen::Vector3d t_c2w;



Eigen::Quaterniond eulerToQuaternion(double pitch, double yaw, double roll) {
    // 使用 Eigen 的四元数构造函数直接创建四元数
    Eigen::Quaterniond q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    return q;
}

Eigen::Matrix3d eulerToRotationMatrix(double pitch, double yaw, double roll) {
    // 使用 Eigen 的四元数计算旋转矩阵
    Eigen::Quaterniond q = eulerToQuaternion(pitch, yaw, roll);
    return q.toRotationMatrix();
}

std::string rond_num(double value,int weishu) {
    // Round the number to 2 decimal places
    double roundedValue = std::round(value * 100.0) / 100.0;
    
    // Use a string stream to format the number
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(weishu) << roundedValue;
    
    return oss.str();
}



int GetHandsPose(Eigen::Quaterniond &Q_c2w, Eigen::Vector3d &t_c2w){

        int state_=0;

        if (image_hand.empty()) {
           return state_;
        }

        cv::namedWindow("cgg_hand_control", cv::WINDOW_NORMAL);

        
        bool new_img = false;
      
        // 设置文字的参数
        double font_scale = 0.5; // 大小
        int thickness = 1; // 粗细
        cv::Scalar color1(255, 0, 0); // 文字颜色
        cv::Scalar color2(0, 0, 255); // 文字颜色

        // // 设置文字
        std::string text1 = "position_xyz: " + (rond_num(x,2)) + " , " +  (rond_num(y ,2)) + " , " +  (rond_num(z,2));
        cv::putText(image_hand, text1, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, font_scale, color1, thickness);

        std::string text2 = "theta_xyz: " +  (rond_num(theta_x,2) ) + " , " +  (rond_num(theta_y ,2)) + " , " +  (rond_num(theta_z,2));
        cv::putText(image_hand, text2, cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, font_scale, color2, thickness);

        cv::imshow("cgg_hand_control", image_hand);
        char key = (char)cv::waitKey(1);

        if (key == 27) { // 按下 'ESC' 键
            std::cout << "退出" << std::endl;
            state_=-1;   
            new_img=false;
            return state_;
            //break;
        } else if (key == 'w') { // 按下 'w' 键
            std::cout << "x前进" << std::endl;
            x += step_;
            i++;
            new_img = true;
        } else if (key == 's') { // 按下 's' 键
            std::cout << "x后退" << std::endl;
            x -= step_;
            i++;
            new_img = true;
        } else if (key == 'a') { // 按下 'a' 键
            std::cout << "y前进" << std::endl;
            y += step_;
            i++;
            new_img = true;
        } else if (key == 'd') { // 按下 'd' 键
            std::cout << "y后退" << std::endl;
            y -= step_;
            i++;
            new_img = true;
        } else if (key == 'q') { // 按下 'q' 键
            std::cout << "z前进" << std::endl;
            z += step_;
            i++;
            new_img = true;
        } else if (key == 'e') { // 按下 'e' 键
            std::cout << "z后退" << std::endl;
            z -= step_;
            i++;
            new_img = true;
        } else if (key == 'i') { // 按下 'i' 键
            std::cout << "x旋转+" << std::endl;
            theta_x += step_theta;
            if (theta_x > 360 || theta_x < -360) theta_x = 0;
            i++;
            new_img = true;
        } else if (key == 'k') { // 按下 'k' 键
            std::cout << "x旋转-" << std::endl;
            theta_x -= step_theta;
            if (theta_x > 360 || theta_x < -360) theta_x = 0;
            i++;
            new_img = true;
        } else if (key == 'j') { // 按下 'j' 键
            std::cout << "y旋转+" << std::endl;
            theta_y += step_theta;
            if (theta_y > 360 || theta_y < -360) theta_y = 0;
            i++;
            new_img = true;
        } else if (key == 'l') { // 按下 'l' 键
            std::cout << "y旋转-" << std::endl;
            theta_y -= step_theta;
            if (theta_y > 360 || theta_y < -360) theta_y = 0;
            i++;
            new_img = true;
        } else if (key == 'u') { // 按下 'u' 键
            std::cout << "z旋转+" << std::endl;
            theta_z += step_theta;
            if (theta_z > 360 || theta_z < -360) theta_z = 0;
            i++;
            new_img = true;
        } else if (key == 'o') { // 按下 'o' 键
            std::cout << "z旋转-" << std::endl;
            theta_z -= step_theta;
            if (theta_z > 360 || theta_z < -360) theta_z = 0;
            i++;
            new_img = true;
        }
        else{

            new_img = false;
            state_=0;
        }
        
       
        if (new_img) {
            state_=1;
            // 示例角度（以弧度为单位）
            double pitch = M_PI*(theta_x/180);  // 30度
            double yaw =  M_PI*(theta_y/180);  // 45度
            double roll =  M_PI*(theta_z/180);  // 60度
            Q_c2w = eulerToQuaternion(pitch, yaw, roll);
            t_c2w={x,y,z};

        }

        return state_;

    
}



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

    std_msgs::Float64 Msg_id; // 创建 Float64 消息
    Msg_id.data =0;
    Q_c2w = Eigen::Quaterniond::Identity();;
    t_c2w={0,0,0.1};
    std::string control_mode="hand";// 自动 auto  手动 hand
 


    ros::Rate rate(1);// Hz 频率

    if(control_mode=="hand"){

        int state_=0;
     
        while ( ros::ok()){
           
            if(ros::ok() && state_==1){      
                Msg_id.data=Msg_id.data+1;
                //ros::spinOnce(); // 不能执行太快 否则来不及处理回调 必须配合ros::Rate rate(10);  rate.sleep(); 单独卡求一个线程处理
                publishPose(pose_pub,Msg_id,Q_c2w,t_c2w);
            }
            else if(state_==-1) {break;}
            
            //std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 暂停以减少CPU使用率
            rate.sleep();

        
            state_= GetHandsPose(Q_c2w,t_c2w);
        }   

    }
    else if(control_mode=="auto"){
        // 定时器每秒调用一次
       
        double i =0;
        while (ros::ok())
        {  
            i=i+0.1;
            if(i>3)i=0;

            t_c2w={i,0,i};

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
