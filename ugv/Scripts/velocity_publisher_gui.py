#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16
from enum import Enum

# ################################################################################
# Code subscribes to 3 velocity topics and publishes to the UGV's velocity topic
# The operating mode is set by the GUI
# To be used when all is said and done, topics and enums can be changed but needs to stay consistent with GUI script
# ################################################################################

class operating_mode(Enum):
    IDLE = 0
    AUTOMATIC_SAR = 1
    AUTOMATIC_MAZE_SOLVE = 2
    MANUAL = 3
op = operating_mode.IDLE

def manual_callback(data):
    global op
    if op == operating_mode.MANUAL:
        # If the operating mode is MANUAL, publish the manual command velocity
        pub.publish(data)

def sar_callback(data):
    global op
    if op == operating_mode.AUTOMATIC_SAR:
        # If the operating mode is AUTOMATIC_SAR, publish the SAR command velocity
        pub.publish(data)

def maze_callback(data):
    global op
    if op == operating_mode.AUTOMATIC_MAZE_SOLVE:
        # If the operating mode is AUTOMATIC_MAZE_SOLVE, publish the maze command velocity
        pub.publish(data)

def mode_callback(data):
    global op
    op = operating_mode(data.data)

rospy.init_node('velocity_publisher', anonymous=True)
pub = rospy.Publisher('/ugv/cmd_vel', Twist, queue_size=10)
msg = Twist()

manual_sub = rospy.Subscriber('/ugv/manual_cmd_vel', Twist, manual_callback)
sar_sub = rospy.Subscriber('/ugv/sar_cmd_vel', Twist, sar_callback)
maze_sub = rospy.Subscriber('/ugv/maze_cmd_vel', Twist, maze_callback)
mode_sub = rospy.Subscriber('/ugv/mode', Int16, mode_callback)

if __name__ == '__main__':
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass