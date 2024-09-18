import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
from collections import deque

class PoseImagePublisher:
    def __init__(self):
        self.pose_sub = rospy.Subscriber('pose_topic', PoseStamped, self.pose_callback)
        self.image_pub = rospy.Publisher('image_topic', Image, queue_size=10)
        self.pose_pub = rospy.Publisher('pose_image_topic', PoseStamped, queue_size=10)
        self.bridge = CvBridge()

        self.pose_queue = deque()  # Queue to store pose messages with timestamps

    def pose_callback(self, msg):
        # Store the pose message with timestamp in the queue
        self.pose_queue.append((msg.header.stamp, msg))
        print("收到  x", msg.pose.position.x, "y", msg.pose.position.y, "z", msg.pose.position.z)

    def publish_image_with_pose(self):
        rate = rospy.Rate(1)  # 1 Hz

        while not rospy.is_shutdown():
            if self.pose_queue:
                timestamp, pose_msg = self.pose_queue.popleft()
                
                #random_x = np.random.uniform(-10, 10) 
                #x=random_x
                x = pose_msg.pose.position.x
                y = pose_msg.pose.position.y
                z = pose_msg.pose.position.z

                qx = pose_msg.pose.orientation.x
                qy = pose_msg.pose.orientation.y
                qz = pose_msg.pose.orientation.z
                qw = pose_msg.pose.orientation.w

                # Create an image
                random_image = np.random.randint(0, 256, (480, 640, 3), dtype=np.uint8)
                cv2.putText(random_image, f'Translation: ({x:.2f}, {y:.2f}, {z:.2f})', (10, 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                cv2.putText(random_image, f'Rotation: ({qx:.2f}, {qy:.2f}, {qz:.2f}, {qw:.2f})', (10, 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

                try:
                    # Set the timestamp for the image and pose
                    image_msg = self.bridge.cv2_to_imgmsg(random_image, "bgr8")
                    image_msg.header.stamp = timestamp
                    pose_msg.header.stamp = timestamp

                    # Publish pose and image
                    self.pose_pub.publish(pose_msg)
                    self.image_pub.publish(image_msg)
                    print("图像数据发送", " 位姿xyz ", x, y, z)
                except CvBridgeError as e:
                    rospy.logerr(f'CvBridge Error: {e}')

            rate.sleep()

def main():
    rospy.init_node('node2', anonymous=True)
    pose_image_publisher = PoseImagePublisher()
    pose_image_publisher.publish_image_with_pose()

if __name__ == '__main__':
    main()
