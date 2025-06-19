#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import time
import sys

# #####################################################################################################################
# This code uses MiDaS to estimate depth from a camera stream and publishes the RGB and depth images to ROS topics.
# Current model is MiDaS_small, but you can switch to MiDaS_large or MiDaS_Hybrid for better accuracy.
# change model to:
# Small: torch.hub.load('intel-isl/MiDaS', 'MiDaS_small')
# Large: torch.hub.load('isl-org/MiDaS', 'MiDaS_large')
# Hybrid: torch.hub.load('isl-org/MiDaS', 'MiDaS_Hybrid')
# Some issues:
# 1. Hybrid model has a latency of 10-15 seconds on my laptop
# 2. Large model is even slower
# 3. Small model is fast but im not sure about the accuracy
# Please test the code with all models to see which one works best our application.
# Make sure to install the required packages: pip install opencv-python torch torchvision
# #####################################################################################################################

# Initialize MiDaS on GPU if available
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model = torch.hub.load('isl-org/MiDaS', 'MiDaS_Hybrid').eval().to(device)
bridge = CvBridge()

rospy.init_node('midas_rtabmap')
rgb_pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=1)
depth_pub = rospy.Publisher('/camera/depth/image_raw', Image, queue_size=1)
info_pub = rospy.Publisher('/camera/rgb/camera_info', CameraInfo, queue_size=1)

header = Header()
header.frame_id = "camera_link"

def publish_camera_info():
    # Mock camera calibration (replace with real values)
    info = CameraInfo()
    info.width = 640
    info.height = 480
    info.header = header
    info.K = [448.13455323, 0, 308.22983219,
                0, 451.57876765, 237.18429739,
                0, 0, 1]
    return info

cap = cv2.VideoCapture("http://192.168.35.18:8080/video")

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret: continue

    now = rospy.Time.now()

    # Resize and convert to RGB
    frame = cv2.resize(frame, (640, 480))
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # MiDaS depth estimation
    input_tensor = torch.from_numpy(frame_rgb).permute(2,0,1).unsqueeze(0).float().to(device)
    with torch.no_grad():
        depth = model(input_tensor)
    depth = depth.squeeze().cpu().numpy()

    # Normalize depth for visualization
    depth_norm = cv2.normalize(depth, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    depth_norm = cv2.applyColorMap(depth_norm, cv2.COLORMAP_INFERNO)

    # Publish to ROS
    rgb_msg = bridge.cv2_to_imgmsg(frame, "rgb8")
    rgb_msg.header = header
    rgb_msg.header.stamp = now

    depth_raw = depth.astype(np.float32)  # Convert depth to float32
    depth_msg = bridge.cv2_to_imgmsg(depth_raw, "32FC1")
    depth_msg.header = header
    depth_msg.header.stamp = now

    info_msg = publish_camera_info()
    info_msg.header.stamp = now

    rgb_pub.publish(rgb_msg)
    depth_pub.publish(depth_msg)
    info_pub.publish(info_msg)

    cv2.imshow("RGB", frame)
    cv2.imshow("Depth", depth_norm)
    if cv2.waitKey(1) == 27: break

cap.release()