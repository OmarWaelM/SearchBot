#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Point
from math import pow, atan2, sqrt, pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
import numpy as np
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
        self.goal_subscriber = rospy.Subscriber('/goal', Vector3, self.goal_callback)  # For auto mode

        self.current_pose = Point()
        self.rate = rospy.Rate(30)
    
    def goal_callback(self, msg):
        self.velocity_publisher.publish(Twist())  # Stop current motion
        object_x = msg.x * np.cos(self.current_pose.z - msg.y) + self.current_pose.x
        object_y = msg.x * np.sin(self.current_pose.z - msg.y) + self.current_pose.y
        rospy.loginfo("Cup location found at (%.2f, %.2f)", object_x, object_y)
        rospy.sleep(1)
        goal_pose = Point()
        goal_pose.x = 0
        goal_pose.y = 0
        self.move2goal(goal_pose, distance_tolerance=0.2)  # Start spiral search
        self.velocity_publisher.publish(Twist())  # Stop current motion
        rospy.signal_shutdown("Goal reached, shutting down.")

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

    def move2goal(self, goal_pose=None, distance_tolerance=None):
        """Modified to work with both manual/auto input"""
        if goal_pose is None:  # Manual mode
            goal_pose = Point()
            goal_pose.x = float(input("Set your x goal: "))
            goal_pose.y = float(input("Set your y goal: "))
            distance_tolerance = float(input("Set your tolerance: "))
        else:  # Auto mode (spiral search)
            distance_tolerance = 0.2  # Default for spiral
        
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

    def spiral_search(self, max_radius=1.0, radius_increment=0.2, angle_step=0.3):
        """Autonomous spiral search pattern"""
        radius = 0.3  # Start near center
        angle = 0

        while radius <= max_radius and not rospy.is_shutdown():
            # Calculate next waypoint (relative to current position)
            goal_pose = Point()
            goal_pose.x = self.current_pose.x + radius * np.cos(angle)
            goal_pose.y = self.current_pose.y + radius * np.sin(angle)

            #Use your existing controller to move,
            self.move2goal(goal_pose, distance_tolerance=0.15)  # Tight tolerance

            #--- YOLO INTEGRATION POINT ---,
            #Update spiral,
            angle += angle_step
            radius += radius_increment / (2 * pi / angle_step)  # Smooth increaset

        rospy.loginfo("Search completed (item not found)")
        return False
        
    def bounded_random_search(self, x_range=1.7, y_range=1.4, steps=5): 
        """Random walk within a rectangular area centered on the start position."""
        start_x = self.current_pose.x
        start_y = self.current_pose.y

        for i in range(steps):
            # Random point within the area
            rand_x = np.random.uniform(start_x - x_range/2, start_x + x_range/2)
            rand_y = np.random.uniform(start_y - y_range/2, start_y + y_range/2)

            goal_pose = Point()
            goal_pose.x = rand_x
            goal_pose.y = rand_y

            rospy.loginfo(f"[Step {i+1}] Moving to: ({goal_pose.x:.2f}, {goal_pose.y:.2f})")
            self.move2goal(goal_pose, distance_tolerance=0.2)

            rospy.loginfo("Bounded random search completed.")
 
if __name__ == '__main__':
    try:
        bot = TurtleBot()
        mode = input("Choose mode (1=manual, 2=random search, 3=spiral search): ")

        if mode == "1":
            bot.move2goal()  # Manual goal
        elif mode == "2":
            bot.bounded_random_search(x_range=1.2, y_range=0.8, steps=5)
        elif mode == "3":
            bot.spiral_search(max_radius=1.0, radius_increment=0.2, angle_step=0.3)

    except rospy.ROSInterruptException:
        pass