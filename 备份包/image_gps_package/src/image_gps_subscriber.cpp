#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <deque>
#include <utility>

std::deque<std::pair<cv::Mat, ros::Time>> image_queue;
std::queue<sensor_msgs::NavSatFixConstPtr> gpsQueue;
std::mutex gpsQueue_Lock;// 队列锁

void gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg) {
    gpsQueue_Lock.lock();
    gpsQueue.push(gps_msg);
    gpsQueue_Lock.unlock();
}


void imageCallback(const sensor_msgs::ImageConstPtr& image_msg) {
    // Convert ROS image message to OpenCV image
    cv_bridge::CvImagePtr cv_image;
    try {
        cv_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    // Get the timestamp of the image
    double image_timestamp = image_msg->header.stamp.toSec();

    // Store image and timestamp in the queue
    image_queue.emplace_back(cv_image->image, image_timestamp);

    
    gpsQueue_Lock.lock();

    while(!gpsQueue.empty()){

        sensor_msgs::NavSatFixConstPtr GPS_msg = gpsQueue.front();
        double gps_timestamp = GPS_msg->header.stamp.toSec();
        double time_diffrence = image_timestamp - gps_timestamp;
        printf("img t: %f, gps t: %f , time_diffrence: %f \n", image_timestamp, gps_timestamp,time_diffrence);
       

        if(abs(image_timestamp - gps_timestamp)<=0.01){// 图像和gps时间差在0.01s=10ms内的算匹配成功 vins-fusion参考值
            // 在线阶段，假设图像20帧/S 相邻帧间隔50ms 中间值间隔25ms算阈值考虑。
            // 离线阶段，一般会提前手动处理图像和GPS时间戳对齐，图像名字以时间戳保存，GNSS的第一列也是一样的时间戳。

            // 经纬度、海拔
            double latitude = GPS_msg->latitude;
            double longitude = GPS_msg->longitude;
            double altitude = GPS_msg->altitude;
            gpsQueue.pop();// 使用完毕 丢弃
            
            std::cout << std::fixed << std::setprecision(9)
            << "图像和GNSS匹配成功" 
            <<" 经度: "<<latitude
            <<" 纬度: "<<longitude
            <<" 高度: "<<altitude
            <<std::endl;

            // 送入 位姿估计线程

            cv::imshow("Matched Image", cv_image->image);
            cv::waitKey(1);
            break;

          
        }
        else if(gps_timestamp < image_timestamp - 0.01)// GPS先开始采集，图像滞后，比图像早期的GPS丢弃
        {
             gpsQueue.pop();
             //break;
        }
        else if(gps_timestamp > image_timestamp + 0.01)// GPS采集快，图像处理慢，比图像快的GPS保留，等待后续图像来匹配
        {
            break;
        }

    }


    gpsQueue_Lock.unlock();// gps
}



int main(int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "image_gps_matcher");
    ros::NodeHandle nh;

    // Subscribe to image and GPS topics
    ros::Subscriber image_sub = nh.subscribe("/camera/image_raw", 10, imageCallback);
    ros::Subscriber gps_sub = nh.subscribe("/gps/fix", 10, gpsCallback);
    cv::namedWindow("Matched Image", cv::WINDOW_AUTOSIZE);

    // 使用 MultiThreadedSpinner，指定线程数为 4
    //ros::MultiThreadedSpinner spinner(4);
    // 调用 spinner 来处理回调函数
    //spinner.spin();


    /*
        阻塞调用，处理回调函数
        ros::spin() 使节点进入一个循环，不断处理所有回调函数，直到节点被关闭或中断。
        适用于那些只依赖于回调函数的节点，例如只处理订阅消息和服务请求的节点。
        它会阻塞当前线程，使得节点在处理回调函数时不会执行其他代码。
        如果你的节点没有其他需要执行的任务，使用 ros::spin() 可以保持节点活跃，确保所有回调函数得到处理。
    */
    ros::spin();
     /*
        非阻塞调用 
        处理所有到达的回调函数一次，然后返回。
        它是非阻塞的，允许主循环继续执行其他代码。
        当你需要在节点中执行除回调处理之外的其他逻辑时，例如定时任务、计算或其他非 ROS 相关的操作。
        你可以在主循环中调用 ros::spinOnce()，使其在处理回调函数的同时执行其他任务。
     */
    
    //ros::spinOnce();

    return 0;
}
