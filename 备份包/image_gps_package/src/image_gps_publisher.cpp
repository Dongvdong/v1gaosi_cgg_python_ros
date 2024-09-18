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
#include <thread>
#include <mutex>

class ImageGPS_Reader {
public:
    std::vector<boost::filesystem::path> image_files_;
    struct GPSData {
        std::string timestamp;
        double lat, lon, alt;
    };
    std::map<std::string, GPSData> gps_data_;

    ImageGPS_Reader() {
        std::string data_dir = "/home/dongdong/2project/0data/NWPU/";
        std::string gps_dir = data_dir + "/config/gps.txt";
        std::string img_dir = data_dir + "img";
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
            gps_data_[timestamp] = {timestamp, lat, lon, alt};
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

// Global variables for shared state and mutex
ImageGPS_Reader reader;
ros::Publisher image_pub_;
ros::Publisher gps_pub_;
std::mutex mtx;

void publishImages() {
    
    std::vector<boost::filesystem::path> image_files_copy;
    {
        std::lock_guard<std::mutex> lock(mtx);
        image_files_copy = reader.image_files_;
    }

    ros::Rate rate(10);  // 10 Hz
    while (ros::ok()) {
        for (const auto& img_file : image_files_copy) {
            std::string timestamp = img_file.stem().string();  // Extract timestamp from filename
            
            cv::Mat cv_image = cv::imread(img_file.string(), cv::IMREAD_COLOR);
            if (!cv_image.empty()) {
                // 发布图像
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image).toImageMsg();
                msg->header.stamp = ros::Time(std::stod(timestamp));                
                image_pub_.publish(msg);
                ROS_INFO("Published image: %s", img_file.filename().string().c_str());
            }
            rate.sleep();
        }
    }
}

void publishGPSData() {

    std::map<std::string, ImageGPS_Reader::GPSData> gps_data_copy;
    {
        std::lock_guard<std::mutex> lock(mtx);
        gps_data_copy = reader.gps_data_;
    }

    ros::Rate rate(10);  // 10 Hz
    while (ros::ok()) {

        for (const auto& entry : gps_data_copy) {
            const auto& timestamp = entry.first;
            const auto& gps = entry.second;

            // 发布GNSS信息
            sensor_msgs::NavSatFix gps_msg;
            gps_msg.header.stamp = ros::Time(std::stod(timestamp));                      
            gps_msg.latitude = gps.lat;
            gps_msg.longitude = gps.lon;
            gps_msg.altitude = gps.alt;
            gps_pub_.publish(gps_msg);
            ROS_INFO("Published GPS data for timestamp: %s", timestamp.c_str());
            rate.sleep();
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_gps_publisher");
    ros::NodeHandle nh;

    // Create publishers
    image_pub_ = nh.advertise<sensor_msgs::Image>("/camera/image_raw", 10);
    gps_pub_ = nh.advertise<sensor_msgs::NavSatFix>("/gps/fix", 10);

    // Start threads for publishing images and GPS data
    std::thread image_thread(publishImages);
    std::thread gps_thread(publishGPSData);

    // Wait for threads to finish (they run indefinitely in this example)
    image_thread.join();
    gps_thread.join();

    return 0;
}
