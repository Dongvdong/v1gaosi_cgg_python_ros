#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <gaosi_img_pose_flag/PoseImgFlagMsg.h> // 更换为你包的名字

class ImagePosePublisher
{
public:
    ImagePosePublisher()
    {
        // Initialize publisher
        pub_ = nh_.advertise<gaosi_img_pose_flag::PoseImgFlagMsg>("image_pose_topic", 10);

        // Load or create a sample image
        image_ = cv::Mat::zeros(480, 640, CV_8UC3); // Black image

        // Set flag
        flag_.data = 1.23; // Example double value

        // Set pose
        pose_.position.x = 1.0;
        pose_.position.y = 2.0;
        pose_.position.z = 3.0;
        pose_.orientation.x = 0.0;
        pose_.orientation.y = 0.0;
        pose_.orientation.z = 0.0;
        pose_.orientation.w = 1.0;

        // Set publish rate
        ros::Rate loop_rate(10); // 10 Hz

        while (ros::ok())
        {
            gaosi_img_pose_flag::PoseImgFlagMsg msg;

            // Convert OpenCV image to ROS image message
            msg.image = *cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_).toImageMsg();
            msg.flag = flag_;
            msg.pose = pose_;

            // Publish message
            pub_.publish(msg);

            ros::spinOnce();
            loop_rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;

    cv::Mat image_;
    std_msgs::Float64 flag_;
    geometry_msgs::Pose pose_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_pose_publisher");
    ImagePosePublisher image_pose_publisher;
    return 0;
}
