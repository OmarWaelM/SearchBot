#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool

class LocalPlanner:
    def __init__(self):
        rospy.init_node('local_planner_node')

        self.cmd_pub = rospy.Publisher('/cmd_vel_obstacle', Twist, queue_size=10)
        self.obstacle_status_pub = rospy.Publisher('/obstacle_detected', Bool, queue_size=1)
        rospy.Subscriber('/obstacle_direction', Vector3, self.obstacle_callback)

        # Obstacle parameters
        self.obstacle_detected = False
        self.handling_obstacle = False  # to prevent repeated handling

        # Thresholds
        self.safe_distance = 0.7  # meters
        self.angle_threshold = 35.0  # degrees

        # Movement parameters
        self.forward_speed = 0.2
        self.turn_speed = 3.0  # radians/sec

        self.twist = Twist()
        self.rate = rospy.Rate(30)

        rospy.loginfo("Local planner ready.")
        rospy.spin()

    def obstacle_callback(self, msg):
        distance = msg.x
        angle = msg.y

        # Check if new obstacle is within dangerous zone
        if distance < self.safe_distance and abs(angle) < self.angle_threshold:
            if not self.handling_obstacle:
                rospy.loginfo("Obstacle detected! Handling avoidance.")
                self.handling_obstacle = True
                self.handle_obstacle(angle)
        else:
            # Publish "no obstacle" status only if previously handling one
            if self.handling_obstacle:
                self.obstacle_detected = False
                self.obstacle_status_pub.publish(Bool(data=False))
                self.handling_obstacle = False

    def handle_obstacle(self, angle):
        self.obstacle_detected = True
        self.obstacle_status_pub.publish(Bool(data=True))

        # Compute turning intensity: centered obstacles need stronger reaction
        angle_ratio = 1 - abs(angle) / self.angle_threshold
        angle_ratio = max(0.0, min(angle_ratio, 1.0))  # Clamp between [0, 1]
        turn_duration = 0.7 + 0.5 * angle_ratio        # between 0.3 and 1.0 sec
        forward_duration = 0.3 + 0.5 * angle_ratio     # between 0.3 and 0.8 sec

        # Stop any current motion
        self.cmd_pub.publish(Twist())
        rospy.sleep(0.1)

        # Turn away from obstacle
        twist = Twist()
        twist.angular.z = -self.turn_speed if angle > 0 else self.turn_speed
        self.cmd_pub.publish(twist)
        rospy.sleep(turn_duration)

        # Move forward slightly
        twist.linear.x = self.forward_speed
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)
        rospy.Timer(rospy.Duration(forward_duration), self.stop_movement, oneshot=True)

    def stop_movement(self, event):
        self.cmd_pub.publish(Twist())  # Stop the robot
        self.obstacle_detected = False
        self.handling_obstacle = False
        self.obstacle_status_pub.publish(Bool(data=False))
        rospy.loginfo("Obstacle handled, resuming goal following.")

if __name__ == '__main__':
    try:
        LocalPlanner()
    except rospy.ROSInterruptException:
        pass
