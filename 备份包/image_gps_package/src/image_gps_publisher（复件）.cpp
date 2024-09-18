#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
#include <fstream>
#include <sstream>
#include <iostream>

class ImageGPS_Reader {


public:

    std::vector<boost::filesystem::path> image_files_;
    struct GPSData {
        std::string timestamp;
        double lat, lon, alt;
    };
    std::map<std::string, GPSData> gps_data_;


    ImageGPS_Reader() {
        std::string data_dir="/home/dongdong/2project/0data/NWPU/";
        std::string gps_dir=data_dir+"/config/gps.txt";
        std::string img_dir=data_dir+"img";
        // Load GPS data
        loadGPSData(gps_dir);

        // Load image filenames
        loadImageFilenames(img_dir);
    }

    void loadGPSData(const std::string& gps_file) {
        std::ifstream file(gps_file);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open GPS file: %s", gps_file.c_str());
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string timestamp;
            double lat, lon, alt;
            if (!(iss >> timestamp >> lat >> lon >> alt)) { break; }
            gps_data_[timestamp] = {timestamp,lat, lon, alt};
        }
        file.close();
    }

    void loadImageFilenames(const std::string& img_folder) {
        namespace fs = boost::filesystem;
        fs::directory_iterator end_itr;
        for (fs::directory_iterator itr(img_folder); itr != end_itr; ++itr) {
            if (fs::is_regular_file(itr->status()) && itr->path().extension() == ".jpg") {
                image_files_.push_back(itr->path());
            }
        }
    }



};

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_gps_publisher");

    ImageGPS_Reader reader;
    
    // Initialize ROS node handle
    ros::NodeHandle nh;
    // Create publishers
    ros::Publisher image_pub_ = nh.advertise<sensor_msgs::Image>("/camera/image_raw", 10);
    ros::Publisher gps_pub_ = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix", 10);


    ros::Rate rate(10);  // 10 Hz

    while (ros::ok()) {
        if (!reader.image_files_.empty() && !reader.gps_data_.empty()) {
            for (const auto& img_file : reader.image_files_) {
                std::string timestamp = img_file.stem().string();  // Extract timestamp from filename
                
                if (reader.gps_data_.find(timestamp) != reader.gps_data_.end()) {// 确保GNSS对应图像存在
                    std::string img_path = img_file.string();
                    cv::Mat cv_image = cv::imread(img_path, cv::IMREAD_COLOR);
                    if (!cv_image.empty()) {
                        // 发布图像

                        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
                        msg->header.stamp = ros::Time(std::stod(timestamp));                
                        image_pub_.publish(msg);

                        // 发布GNSS信息
                        auto gps = reader.gps_data_[timestamp];
                        sensor_msgs::NavSatFix gps_msg;
                        gps_msg.header.stamp = ros::Time(std::stod(timestamp));                      
                        gps_msg.latitude = gps.lat;
                        gps_msg.longitude = gps.lon;
                        gps_msg.altitude = gps.alt;
                        gps_pub_.publish(gps_msg);

                        ROS_INFO("Published image: %s and GPS data", img_file.filename().string().c_str());
                        rate.sleep(); // 按照设定的频率休眠
                    }
                }
            }
        }
        //处理一次所有的回调函数。这个调用会处理所有的回调并清空消息队列。对于消息处理来说，它确保了消息的及时处理。
        ros::spinOnce();
    }//while

    return 0;
}
