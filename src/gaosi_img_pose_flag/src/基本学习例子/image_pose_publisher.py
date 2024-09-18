#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from gaosi_img_pose_flag.msg import ImagePose  # 更换为你包的名字
from cv_bridge import CvBridge
import numpy as np
import cv2

class ImagePosePublisher:
    def __init__(self):
        # Initialize node
        rospy.init_node('image_pose_publisher', anonymous=True)

        # Initialize publishers
        self.pub = rospy.Publisher('image_pose_topic', ImagePose, queue_size=10)

        # Create a bridge between OpenCV and ROS
        self.bridge = CvBridge()
        # Create or load a sample image
        self.image = np.zeros((480, 640, 3), dtype=np.uint8)  # Black image

        # Set flag
        self.flag = Float64()
        self.flag.data = 1.23  # Example double value

        # Set pose
        self.pose = Pose()
        self.pose.position.x = 1.0
        self.pose.position.y = 2.0
        self.pose.position.z = 3.0
        self.pose.orientation.x = 0.0
        self.pose.orientation.y = 0.0
        self.pose.orientation.z = 0.0
        self.pose.orientation.w = 1.0

        # Set publish rate
        self.rate = rospy.Rate(10)  # 10 Hz

        self.publish()

    def publish(self):
        while not rospy.is_shutdown():
            msg = ImagePose()

            # Convert OpenCV image to ROS image message
            msg.image = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
            msg.flag = self.flag
            msg.pose = self.pose

            # Publish message
            self.pub.publish(msg)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        ImagePosePublisher()
    except rospy.ROSInterruptException:
        pass
