#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ############################################################
# This script plots the trajectory of a robot based on its position and shows it in real-time.
# Future updates will have it send the trajectory on an image topic to be used with the maingui.py script.
# ############################################################

#Arrays to store the x and y positions of the robot
x_data = []
y_data = []

def pos_callback(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y 
    x_data.append(x)
    y_data.append(y)
   
rospy.init_node('trajectory_plot', anonymous=True)
rospy.Subscriber('/ugv/position', Odometry, pos_callback)

#Setting up the plot    
fig,ax= plt.subplots()
ax.set_title('Robot Trajectory')
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.grid(True)
line, = ax.plot([], [], 'bo')  # Blue dots

def update(frame):
    line.set_data(x_data, y_data)  # Set new X/Y data for the plot
    ax.relim()  # Recalculate limits based on the new data
    ax.autoscale_view()  # Automatically adjust the axes
    return line,  

ani = animation.FuncAnimation(fig, update, interval=200) # updates every 0.2s
plt.show()