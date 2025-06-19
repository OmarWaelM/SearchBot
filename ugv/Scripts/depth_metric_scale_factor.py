#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch

# Initialize MiDaS
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model = torch.hub.load('intel-isl/MiDaS', 'MiDaS_small').eval().to(device)
bridge = CvBridge()

# ################################################################
# SCALE CALIBRATION PARAMETERS
# 1. Place an object at KNOWN DISTANCE (e.g., 1.0 meter)
# 2. Click on the object in the depth window to get its raw MiDaS value
# 3. Calculate: scale_factor = true_distance / clicked_depth_value
# 4. Update the scale_factor below
# ################################################################
scale_factor = 185  # Default (update this after calibration!)
clicked_depth = None

def mouse_callback(event, x, y, flags, param):
    global clicked_depth
    if event == cv2.EVENT_LBUTTONDOWN:
        clicked_depth = depth_map[y, x]
        print(f"Clicked depth value: {clicked_depth:.4f}")
        print(f"Suggested scale factor for 0.5m: {0.5 * clicked_depth:.5f}")

def publish_camera_info():
    info = CameraInfo()
    info.width = 640
    info.height = 480
    info.K = [448.13455323, 0, 308.22983219,
                0, 451.57876765, 237.18429739,
                0, 0, 1]  # Fake intrinsics
    return info

rospy.init_node('midas_rtabmap')
rgb_pub = rospy.Publisher('/camera/rgb/image_raw', Image, queue_size=1)
depth_pub = rospy.Publisher('/camera/depth/image_raw', Image, queue_size=1)
info_pub = rospy.Publisher('/camera/rgb/camera_info', CameraInfo, queue_size=1)

cap = cv2.VideoCapture("http://192.168.1.115:8080/video")
cv2.namedWindow("Depth")
cv2.setMouseCallback("Depth", mouse_callback)

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret: continue

    now = rospy.Time.now()
    header = Header()
    header.stamp = now
    header.frame_id = "camera_link"

    # Resize and convert to RGB
    frame = cv2.resize(frame, (640, 480))
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
   
    # MiDaS depth estimation
    input_tensor = torch.from_numpy(frame_rgb).permute(2,0,1).unsqueeze(0).float().to(device)
    with torch.no_grad():
        depth_map = model(input_tensor).squeeze().cpu().numpy()
   
    # Convert to metric depth
    depth_metric = depth_map / scale_factor

    frame_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
    frame_msg.header = header

    depth_msg = bridge.cv2_to_imgmsg(depth_metric, "32FC1")
    depth_msg.header = header
    
    info_msg = publish_camera_info()
    info_msg.header = header

    # Publish to ROS (using metric depth)
    rgb_pub.publish(frame_msg)
    depth_pub.publish(depth_msg)
    info_pub.publish(info_msg)
   
    # Visualization (normalized)
    depth_norm = cv2.normalize(depth_map, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
    depth_norm = cv2.applyColorMap(depth_norm, cv2.COLORMAP_INFERNO)

    cv2.imshow("RGB", frame)
    cv2.imshow("Depth", depth_norm)
    if cv2.waitKey(1) == 27:
        break