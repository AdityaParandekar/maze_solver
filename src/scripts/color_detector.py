#!/usr/bin/env python3
import numpy as np
import cv2
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

cv2_img = np.array([0])
color_boundaries = {
    "red":    ([0, 87, 20], [180, 255, 255]),
    "blue":   ([94, 80, 2],   [120, 255, 255]),
    "green": ([25, 52, 72], [102, 255, 255])
}

bridge = CvBridge()
x=0.
y=0.

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

def image_callback(msg):
    global cv2_img
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        # cv2.imwrite('camera_image.jpeg', cv2_img)
        # rospy.sleep(2)
    except CvBridgeError:
        pass

def detect():
    # image = cv2.imread(img)
    global cv2_img
    image = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2HSV)
    for color_name, (lower, upper) in color_boundaries.items():
        # create NumPy arrays from the boundaries
        lower = np.array(lower, dtype = np.uint8)
        upper = np.array(upper, dtype = np.uint8)

        # find the colors within the specified boundaries and apply the mask
        mask = cv2.inRange(image, lower, upper)
        output = cv2.bitwise_and(image, image, mask = mask)

        if mask.any():
            rospy.loginfo_once(f"{color_name} color detected")
    
    rospy.sleep(2)

def goal():
    global x, y
    if abs(x-4.7) < 0.5 and abs(y-0.9281) < 0.5:
        detect()



if __name__ == "__main__":
    rospy.init_node("color_detector")
    sub_image = rospy.Subscriber("/realsense/color/image_raw", Image, image_callback )
    sub_odom = rospy.Subscriber("/odom", Odometry, newOdom)
    while not rospy.is_shutdown():
        goal()