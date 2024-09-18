#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose
from gaosi_img_pose_flag.msg import ImagePose  # 更换为你包的名字
from cv_bridge import CvBridge
import cv2

class ImagePoseSubscriber:
    def __init__(self):
        # Initialize node
        rospy.init_node('image_pose_subscriber', anonymous=True)

        # Initialize subscriber
        self.sub = rospy.Subscriber('image_pose_topic', ImagePose, self.callback)

        # Create a bridge between OpenCV and ROS
        self.bridge = CvBridge()

        # Create an OpenCV window
        cv2.namedWindow("Received Image")

    def callback(self, msg):
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg.image, desired_encoding="bgr8")
            cv2.imshow("Received Image", cv_image)
            cv2.waitKey(30)
        except Exception as e:
            rospy.logerr("Failed to convert image: %s", str(e))

        rospy.loginfo("Received flag: %.2f", msg.flag.data)
        rospy.loginfo("Received pose: Position(%.2f, %.2f, %.2f), Orientation(%.2f, %.2f, %.2f, %.2f)",
                      msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
                      msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        subscriber = ImagePoseSubscriber()
        subscriber.run()
    except rospy.ROSInterruptException:
        pass
