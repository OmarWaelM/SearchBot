#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# ################################################################################
# This script publishes images from a webcam to a ROS topic.
# Make sure to update the ip address of the webcam in the cam_IP variable.
# ################################################################################

# Tailscale IP
#https://100.85.103.12:8080/video 

cam_IP = "http://192.168.1.118:8080/video"

def webcam_publisher():
    rospy.init_node('webcam_publisher', anonymous=True)
    pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
    rate = rospy.Rate(30) # 30hz
    bridge = CvBridge()
    cap = cv2.VideoCapture(cam_IP)
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            image_message = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            pub.publish(image_message)
        rate.sleep()
        
if __name__ == '__main__':
    try:
        webcam_publisher()
    except rospy.ROSInterruptException:
        pass