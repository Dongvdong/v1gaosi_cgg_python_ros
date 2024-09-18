#include <ros/ros.h>
#include <gaosi_img_pose_flag/ImagePose.h> // 更换为你包的名字
#include <queue>
#include <mutex>
#include <thread>
#include <iostream>

// Global variables
std::queue<gaosi_img_pose_flag::ImagePose::ConstPtr> data_queue;
std::mutex queue_mutex;

// Callback function to handle incoming messages
void callback(const gaosi_img_pose_flag::ImagePose::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(queue_mutex);
    data_queue.push(msg);
}

// Thread function to process the queue
void processQueue()
{
    while (ros::ok())
    {
        std::queue<gaosi_img_pose_flag::ImagePose::ConstPtr> local_queue;

        {
            std::lock_guard<std::mutex> lock(queue_mutex);
            std::swap(local_queue, data_queue); // Safely access the queue
        }

        while (!local_queue.empty())
        {
            auto msg = local_queue.front();
            local_queue.pop();

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

    // Initialize the subscriber
    ros::Subscriber sub = nh.subscribe("image_pose_topic", 10, callback);

    // Create a thread to process the queue
    std::thread processing_thread(processQueue);

    // Start a thread to run ros::spin()
    std::thread spin_thread(spinThread); 


    ros::Rate rate(1);// Hz 频率
    while (ros::ok())
    {
        // Call ros::spinOnce() to process callbacks
        //ros::spinOnce();

        // Process the queue
        //processQueue();
        std::cout<< " 主线程暂停" <<std::endl;

        // Sleep for the rest of the loop period
        rate.sleep();
    }
    //ros::spin();

    
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
