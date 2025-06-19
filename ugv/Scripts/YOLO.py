#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
import os
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
from ultralytics import YOLO

# ################################################################
# This script detects objects using YOLO11n and calculates their
# distance, width, and angle from the camera. It publishes the data to a topic
# to add an object make sure the height is known
# Camera calibration needs to be done and camera URL needs to be set
# ################################################################

#                            ROS Setup
rospy.init_node("yolo_navigator")
obstacle_pub = rospy.Publisher('/obstacle_direction', Vector3, queue_size=5)
goal_pub = rospy.Publisher('/goal', Vector3, queue_size=5)
rospy.Rate(30)
bridge = CvBridge()

#                       Camera Calibration 
camera_matrix = np.array([[448.13455323, 0, 308.22983219],
                          [0, 451.57876765, 237.18429739],
                          [0, 0, 1]], dtype=float)
dist_coeffs = np.array([ 0.17120527, -0.49938996,  0.00151788, -0.0050784,   0.4006422 ], dtype=float)

#                          YOLO Model  
model = YOLO("yolo11n.pt") 

#                      Object Configuration 
OBJECT_CLASSES = {
    "goal": ["cup"],
    "obstacle": ["bottle"]
}

#                 Assumed Object Heights (meters)
ASSUMED_WIDTHS = {
    "bottle": 0.06,
    "cup": 0.05
}

# ==== Camera Stream ====
cap = cv2.VideoCapture("http://192.168.35.18:8080/video")

def calculate_metrics(box, class_name, frame_height):
    """Calculate real-world distance and width using perspective geometry"""
    x1, y1, x2, y2 = box
    pixel_height = y2 - y1
    pixel_width = x2 - x1

    if class_name in ASSUMED_WIDTHS:
        real_width = ASSUMED_WIDTHS[class_name]  # repurposing as assumed width
        distance = (real_width * camera_matrix[0,0]) / pixel_width
        # Recompute height in meters for consistency (optional)
        real_height = (pixel_height * distance) / camera_matrix[1,1]
    else:
        # Fallback method if width is unknown
        distance = (0.2 * camera_matrix[0,0]) / pixel_width
        real_width = (pixel_width * distance) / camera_matrix[0,0]

    center_x = (x1 + x2) / 2
    angle = np.degrees(np.arctan2(center_x - frame.shape[1]/2, camera_matrix[0,0]))
    
    return distance, real_width, angle

#                             Main Loop 
while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        rospy.logerr("Camera error")
        break

    results = model(frame, verbose=False)
    detections = {"goal": [], "obstacle": []}

    for result in results:
        for box in result.boxes:
            class_id = int(box.cls)
            class_name = model.names[class_id]
            conf = float(box.conf)
            xyxy = box.xyxy[0].cpu().numpy()
            
            # Classify objects
            for obj_type, classes in OBJECT_CLASSES.items():
                if class_name in classes:
                    distance, width_meters, angle = calculate_metrics(xyxy, class_name, frame.shape[0])
                    detections[obj_type].append((xyxy, distance, width_meters, angle, class_name))
                    break

    # Process detections
    for obj_type in detections:
        for i, (xyxy, dist, width, angle, name) in enumerate(detections[obj_type]):
            color = (0, 255, 0) if obj_type == "goal" else (0, 0, 255)
            cv2.rectangle(frame, (int(xyxy[0]), int(xyxy[1])), 
                        (int(xyxy[2]), int(xyxy[3])), color, 2)
            
            label = f"{name} {dist:.2f}m {width:.2f}m {angle:.1f}°"
            cv2.putText(frame, label, (int(xyxy[0]), int(xyxy[1])-10),
                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

            if i == 0 and obj_type == "obstacle":  # Publish closest of each type
                msg = Vector3()
                msg.x = dist      # Distance 
                msg.y = angle     # Angle 
                msg.z = width     # Width 
                obstacle_pub.publish(msg)
                
                print(f"{obj_type.upper()}: {name} | Dist: {dist:.2f}m | Width: {width:.2f}m | Angle: {angle:.1f}°")
            elif i == 0 and obj_type == "goal":  # Publish goal
                msg = Vector3()
                msg.x = dist
                msg.y = angle
                msg.z = width
                goal_pub.publish(msg)
                print(f"{obj_type.upper()}: {name} | Dist: {dist:.2f}m | Width: {width:.2f}m | Angle: {angle:.1f}°")

    cv2.imshow("YOLO Navigation", frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()