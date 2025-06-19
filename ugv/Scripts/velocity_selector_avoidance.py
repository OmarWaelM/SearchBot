#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

# ###########################################################################
# This is the velocity selector for the midterm phase.
# Another version is used in the final phase (velocity_selector.py).
# Can be used as a failsafe.
# ###########################################################################

class CmdSelector:
    def __init__(self):
        rospy.init_node('cmd_selector_node')

        self.cmd_pub = rospy.Publisher('/ugv/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/cmd_vel_goal', Twist, self.goal_callback)
        rospy.Subscriber('/cmd_vel_obstacle', Twist, self.obstacle_callback)
        rospy.Subscriber('/obstacle_detected', Bool, self.obstacle_status_callback)

        self.last_goal_cmd = Twist()
        self.last_obstacle_cmd = Twist()
        self.obstacle_detected = False

        self.rate = rospy.Rate(30)
        self.run()

    def goal_callback(self, msg):
        self.last_goal_cmd = msg

    def obstacle_callback(self, msg):
        self.last_obstacle_cmd = msg

    def obstacle_status_callback(self, msg):
        self.obstacle_detected = msg.data
        print(f"Obstacle detected")

    def run(self):
        while not rospy.is_shutdown():
            if self.obstacle_detected:
                self.cmd_pub.publish(self.last_obstacle_cmd)
            else:
                self.cmd_pub.publish(self.last_goal_cmd)

            self.rate.sleep()

if __name__ == '__main__':
    try:
        CmdSelector()
    except rospy.ROSInterruptException:
        pass
