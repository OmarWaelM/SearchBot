#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch

# Initialize MiDaS and ROS
device = 'cuda' if torch.cuda.is_available() else 'cpu'
model = torch.hub.load('intel-isl/MiDaS', 'MiDaS_small').eval().to(device)
bridge = CvBridge()

# Scale factor from calibration (replace with your value)
SCALE_FACTOR = 2.34  # Example: 1.0m real distance = 0.4275 MiDaS units → 1/0.4275 ≈ 2.34

def main():
    rospy.init_node('midas_laserscan')
   
    # Publishers
    depth_pub = rospy.Publisher('/camera/depth/image_metric', Image, queue_size=1)
    info_pub = rospy.Publisher('/camera/depth/camera_info', CameraInfo, queue_size=1)
   
    # Camera info (replace with your camera's intrinsics)
    camera_info = CameraInfo()
    camera_info.width = 640
    camera_info.height = 480
    camera_info.K = [448.13455323, 0, 308.22983219,
                0, 451.57876765, 237.18429739,
                0, 0, 1] # Example intrinsics
   
    cap = cv2.VideoCapture("http://192.168.35.18:8080/video")  # Your IP camera
   
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret: continue
       
        # MiDaS processing
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_tensor = torch.from_numpy(frame_rgb).permute(2,0,1).unsqueeze(0).float().to(device)
       
        with torch.no_grad():
            depth = model(input_tensor)
       
        # Convert to metric depth and numpy
        depth_metric = depth.squeeze().cpu().numpy() * SCALE_FACTOR
       
        # Publish metric depth for laserscan conversion
        depth_msg = bridge.cv2_to_imgmsg(depth_metric.astype(np.float32), "32FC1")
        depth_msg.header.stamp = rospy.Time.now()
        depth_msg.header.frame_id = "camera_depth_frame"
       
        depth_pub.publish(depth_msg)
        camera_info.header = depth_msg.header
        info_pub.publish(camera_info)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
