#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Point
from math import pow, atan2, sqrt, pi
from nav_msgs.msg import Odometry
#from playsound import playsound
#import pyttsx3

# ################################################################
# Original go to goal script taking inputs from the terminal
# ################################################################

# Initialize text-to-speech engine
#engine = pyttsx3.init()

#def speak(text):
    #engine.say(text)
    #engine.runAndWait()

class TurtleBot:
    def __init__(self):
        # Control parameters (tuned for better performance)
        self.Kp_linear = 0.5    # Proportional gain for linear velocity
        self.Kp_angular = 1.3   # Proportional gain for angular velocity
        self.max_linear_vel = 0.8
        self.max_angular_vel = 6.0
        
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Using your original topics
        self.velocity_publisher = rospy.Publisher('/cmd_vel_goal', Twist, queue_size=10) # change later
        self.pose_subscriber = rospy.Subscriber('/ugv/position', Odometry, self.update_pose)

        self.current_pose = Point()
        self.rate = rospy.Rate(30)
        
    def update_pose(self, data):
        """Callback function for Odometry messages"""
        self.current_pose.x = data.pose.pose.position.x
        self.current_pose.y = data.pose.pose.position.y
        
        # Assuming orientation.z is already in degrees
        self.current_pose.z = data.pose.pose.orientation.z
        
    def euclidean_distance(self, goal_pose):
        """Calculate distance to goal"""
        return sqrt(pow((goal_pose.x - self.current_pose.x), 2) +
               pow((goal_pose.y - self.current_pose.y), 2))

    def steering_angle(self, goal_pose):
        """Calculate angle to goal in radians"""
        return atan2(goal_pose.y - self.current_pose.y, 
                    goal_pose.x - self.current_pose.x)

    def move2goal(self):
        """Move to goal position"""
        goal_pose = Point()

        # Get user input
        goal_pose.x = float(input("Set your x goal: "))
        goal_pose.y = float(input("Set your y goal: "))
        distance_tolerance = float(input("Set your tolerance: "))

        vel_msg = Twist()
        last_angle_diff = 0  # For debugging

        while self.euclidean_distance(goal_pose) >= distance_tolerance and not rospy.is_shutdown():
            # Calculate desired angle and difference (in radians)
            desired_angle_rad = self.steering_angle(goal_pose)
            angle_diff = desired_angle_rad - self.current_pose.z
            
            # Normalize angle to [-pi, pi]
            while angle_diff > pi:
                angle_diff -= 2 * pi
            while angle_diff < -pi:
                angle_diff += 2 * pi
                
            last_angle_diff = angle_diff * 180 / pi  # Convert to degrees for debugging
            
            # Angular velocity control
            vel_msg.angular.z = self.Kp_angular * angle_diff
            vel_msg.angular.z = max(min(vel_msg.angular.z, self.max_angular_vel), -self.max_angular_vel)
            
            # Only move forward when facing roughly the right direction
            if abs(angle_diff) < pi/6:  # ~30 degrees
                vel_msg.linear.x = min(
                    self.Kp_linear * self.euclidean_distance(goal_pose),
                    self.max_linear_vel
                )
            else:
                vel_msg.linear.x = 0.0
            
            # Debug output
            rospy.loginfo_throttle(1.0, (
                f"Current: ({self.current_pose.x:.2f}, {self.current_pose.y:.2f}, {self.current_pose.z:.1f}°) | "
                f"Goal: ({goal_pose.x:.2f}, {goal_pose.y:.2f}) | "
                f"Angle diff: {last_angle_diff:.1f}° | "
                f"Velocities: lin={vel_msg.linear.x:.2f}, ang={vel_msg.angular.z:.2f}"
            ))
            
            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

        # Stop when reached goal
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo(f"Goal reached! Final position: ({self.current_pose.x:.2f}, {self.current_pose.y:.2f})")
        #speak("Goal reached")
        #playsound('/home/vboxuser/Downloads/Video.mp3')  # Replace with actual file path

if __name__ == '__main__':
    try:
        bot = TurtleBot()
        while not rospy.is_shutdown():
            # Wait for the user to press Enter to start moving
            bot.move2goal()
    except rospy.ROSInterruptException:
        pass