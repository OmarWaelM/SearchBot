#!/usr/bin/env python3
import cv2
import numpy as np
import dearpygui.dearpygui as dpg
import rospy
import os
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Int16
from cv_bridge import CvBridge
from enum import Enum

# ################################################################################
# This is the main GUI script
# It has 3  modes: Automatic SAR, Automatic Maze Solving, and Manual Operation. They can be changed at any time.
# No need to run it when testing we can leave it till the end.
# Be sure to update the screen dimensions to match your screen resolution.
# All fonts used are found in the font folder
# ################################################################################

dpg.create_context()

#Screen Dimensions
screen_width = 2488 # needs to be adjusted per your screen resolution resolution_width
screen_height = 1536 # needs to be adjusted per your screen resolution resolution_height
video_window_width = int(screen_width * 0.75)
video_window_height = int(screen_height * 0.75)
feed_selection_height = int(screen_height * 0.05)
feed_selection_width = video_window_width
video_width = 1860 # needs to be adjusted per your screen resolution (resolution_height * 0.75 - 15)
video_height = 1140 # needs to be adjusted per your screen resolution (resolution_width * 0.75 - 20)
info_width = screen_width - video_width
info_height = screen_height
bottom_bar_height = screen_height - video_window_height - feed_selection_height
bottom_bar_width = video_window_width

# Parameters
class operating_mode(Enum):
    SPLASH_SCREEN = 0
    AUTOMATIC_SAR = 1
    AUTOMATIC_MAZE_SOLVE = 2
    MANUAL = 3
op = operating_mode.SPLASH_SCREEN

#Fonts
script_dir = os.path.dirname(os.path.realpath(__file__))  # path to main_gui.py
font_path_transrobotics =  os.path.join(script_dir, "../font/sf-transrobotics-font/SfTransrobotics-gBv4.ttf")
font_path_electrickicks = os.path.join(script_dir, "../font/electric-kicks-font/ElectricKicksItalic-PKW5g.ttf")
with dpg.font_registry():
    dpg.add_font(font_path_transrobotics, 36, tag="font_36pt")
    dpg.add_font(font_path_transrobotics, 48, tag="font_48pt")
    dpg.add_font(font_path_transrobotics, 24, tag="font_24pt")
    dpg.add_font(font_path_transrobotics, 30, tag="font_30pt")
    dpg.add_font(font_path_electrickicks, 250, tag="splash_font_250pt")

#Texture
with dpg.texture_registry(show=False):
    dummy_frame = np.zeros((video_height, video_width, 4), dtype=np.float32)  # RGBA
    texture_camera = dpg.add_dynamic_texture(video_width, video_height, dummy_frame.flatten())
    texture_map = dpg.add_dynamic_texture(video_width, video_height, dummy_frame.flatten())

#ROS Topic Info
x_position = 0
y_position = 0
orientation = 0
linear_vel = 0
angular_vel = 0

rospy.init_node("GUI", anonymous=True)
rate = rospy.Rate(30)

def camera_callback(data):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    frame = cv2.resize(frame, (video_width, video_height))
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA).astype(np.float32) / 255.0
    dpg.set_value(texture_camera, frame.flatten())

def map_callback(data):
    bridge = CvBridge()
    frame = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    frame = cv2.resize(frame, (video_width, video_height))
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA).astype(np.float32) / 255.0
    dpg.set_value(texture_map, frame.flatten())

def odometry_callback(data):
    global x_position, y_position, orientation
    x_position = data.position.x
    y_position = data.position.y
    orientation = data.orientation.z
    dpg.set_value("x_position", "X: " + str(x_position))
    dpg.set_value("y_position", "Y: " + str(y_position))
    dpg.set_value("orientation", "Z: " + str(orientation) + "°", tag="orientation")

def vel_callback(data):
    global linear_vel, angular_vel
    linear_vel = data.linear.x
    angular_vel = data.angular.z
    dpg.set_value("linear_vel_auto", "Linear: " + str(linear_vel) + " m/s")
    dpg.set_value("angular_vel_auto", "Angular: " + str(angular_vel) + " rad/s")

# Subscribe to the ROS topic
rospy.Subscriber("/camera/image_raw", Image, camera_callback)
rospy.Subscriber("/ugv/map", Image, map_callback)
rospy.Subscriber("/ugv/position", Pose, odometry_callback)
rospy.Subscriber("/ugv/cmd_vel", Twist, vel_callback)

mode_pub = rospy.Publisher("/ugv/mode", Int16, queue_size=10)
mode_msg = Int16()
manual_vel_pub = rospy.Publisher("/ugv/manual_cmd_vel", Twist, queue_size=10)
manual_vel_msg = Twist()

# Create viewport
dpg.create_viewport(title="UGV", width=screen_width, height=screen_height, resizable=True)
dpg.setup_dearpygui()
dpg.show_viewport()

# Splash Screen
with dpg.window(label="Splash Screen", tag="splash_screen", no_title_bar=True, width=screen_width, 
                height=screen_height, pos=(0, 0), no_move=True, no_resize=True, show=True):
    dpg.add_text("Stalker", pos=(int(0.325*screen_width), int(1.5*screen_height/14)), tag="splash_text")
    dpg.add_button(label="Automatic SAR", width=int(0.4*screen_width), height=int(2*screen_height/14), pos=(int(0.3*screen_width), int(5*screen_height/14)), tag="automatic_sar_button")
    dpg.add_button(label="Automatic Maze Solving", width=int(0.4*screen_width), height=int(2*screen_height/14), pos=(int(0.3*screen_width), int(8*screen_height/14)), tag="automatic_maze_solve_button")
    dpg.add_button(label="Manual Operation", width=int(0.4*screen_width), height=int(2*screen_height/14), pos=(int(0.3*screen_width), int(11*screen_height/14)), tag="manual_button")

# Selecting Feed Window (Automatic and Manual)
with dpg.window(label="Feed_Selection", tag="feed_selection", no_title_bar=True, width=feed_selection_width, 
                height=feed_selection_height, pos=(0, 0), no_move=True, no_resize=True, show=False):
    dpg.add_button(label="Camera Feed", width=int(feed_selection_width/2), height=feed_selection_height, pos=(0,0), tag="camera_feed_sel")
    dpg.add_button(label="Map Feed", width=int(feed_selection_width/2), height=feed_selection_height, pos= (int(feed_selection_width/2), 0), tag="map_feed_sel")

# Video Feed Window (Automatic and Manual)
with dpg.window(label="Video Feed", tag="ugv_video", no_title_bar=True, width = video_window_width, 
                height = video_window_height, pos=(0, feed_selection_height), no_move=True, no_resize=True, show=False):
    dpg.add_image(texture_camera, pos=(0,0))

# Map Feed Window (Automatic and Manual)
with dpg.window(label="Map Feed", tag="ugv_map", no_title_bar=True, width = video_window_width,
                height = video_window_height, pos=(0, feed_selection_height), no_move=True, no_resize=True, show=False):
    dpg.add_image(texture_map, pos=(0,0))

# Info Window (Automatic Only)
with dpg.window(label = "Info_Auto", tag="ugv_info_auto", no_title_bar = True, width = info_width, height = info_height, 
                pos=(video_window_width, 0), no_move=True, no_resize=True, show=False):
    dpg.add_spacer(height=100)
    dpg.add_text("Position:", tag="position_title_auto")
    dpg.add_spacer(height=40)
    dpg.add_text("X: " + str(x_position), tag="x_position_auto")
    dpg.add_spacer(height=40)
    dpg.add_text("Y: " + str(x_position), tag="y_position_auto")
    dpg.add_spacer(height=60)
    dpg.add_text("Orientation:", tag="orientation_title_auto")
    dpg.add_spacer(height=40)
    dpg.add_text("Z: " + str(orientation) + "°", tag="orientation_auto")
    dpg.add_spacer(height=60)
    dpg.add_text("Velocity:", tag="velocity_title_auto")
    dpg.add_spacer(height=40)
    dpg.add_text("Linear: " + str(linear_vel) + " m/s", tag="linear_vel_auto")
    dpg.add_spacer(height=40)
    dpg.add_text("Angular: " + str(angular_vel) + " rad/s", tag="angular_vel_auto")
    dpg.add_spacer(height=60)

# Info Window (Manual Only)
with dpg.window(label = "Info_Manual", tag="ugv_info_manual", no_title_bar = True, width = info_width, height = info_height,
                pos=(video_window_width, 0), no_move=True, no_resize=True, show=False):
    dpg.add_spacer(height=100)
    dpg.add_text("Position:", tag="position_title_manual")
    dpg.add_spacer(height=40)
    dpg.add_text("X: " + str(x_position), tag="x_position_manual")
    dpg.add_spacer(height=40)
    dpg.add_text("Y: " + str(x_position), tag="y_position_manual")
    dpg.add_spacer(height=60)
    dpg.add_text("Orientation:", tag="orientation_title_manual")
    dpg.add_spacer(height=40)
    dpg.add_text("Z: " + str(orientation) + "°", tag="orientation_manual")
    dpg.add_spacer(height=60)
    dpg.add_text("Linear Velocity:", tag="velocity_title_manual")
    dpg.add_spacer(height=20)
    dpg.add_slider_double(label="Linear Velocity", default_value=0, min_value=-2, max_value=2, width=info_width-20, tag="linear_vel_slider")
    dpg.add_spacer(height=40)
    dpg.add_text("Angular Velocity:", tag="angular_vel_title_manual")
    dpg.add_spacer(height=20)
    dpg.add_slider_double(label="Angular Velocity", default_value=0, min_value=-5, max_value=5, width=info_width-20, tag="angular_vel_slider")

# Control Panel Window (Automatic)
with dpg.window(label="Bottom Bar Automatic", tag="bottom_bar_auto", no_title_bar=True, width=bottom_bar_width, 
                height=bottom_bar_height, pos=(0, int(screen_height*0.8)), no_move=True, no_resize=True, show=False):
    number_of_buttons = 1
    denominator = 3*number_of_buttons + 1
    dpg.add_text("Control Panel", pos=(20,25), tag="control_panel_title")
    dpg.add_button(label="Switch Mode", width = int(2*bottom_bar_width/denominator), height = int(bottom_bar_height/3), pos=(int(bottom_bar_width/denominator), int(bottom_bar_height/3)), tag="switch_mode_button")

# Control Panel Window (Manual)
with dpg.window(label="Bottom Bar Manual", tag="bottom_bar_manual", no_title_bar=True, width=bottom_bar_width,
                height=bottom_bar_height, pos=(0, int(screen_height*0.8)), no_move=True, no_resize=True, show=False):
    number_of_buttons = 5
    denominator = 3*number_of_buttons + 1
    dpg.add_text("Control Panel", pos=(20,25), tag="control_panel_title_manual")
    dpg.add_button(label="Switch Mode", width = int(2*bottom_bar_width/denominator), height = int(bottom_bar_height/3), pos=(int(bottom_bar_width/denominator), int(bottom_bar_height/3)), tag="switch_mode_button_manual")
    dpg.add_button(label="Stop", width = int(2*bottom_bar_width/denominator), height = int(bottom_bar_height/3), pos=(int(4*bottom_bar_width/denominator), int(bottom_bar_height/3)), tag="stop_button_manual")
    dpg.add_button(label="Linear", width = int(2*bottom_bar_width/denominator), height = int(bottom_bar_height/3), pos=(int(7*bottom_bar_width/denominator), int(bottom_bar_height/3)), tag="linear_only_button")
    dpg.add_button(label="Angular", width = int(2*bottom_bar_width/denominator), height = int(bottom_bar_height/3), pos=(int(10*bottom_bar_width/denominator), int(bottom_bar_height/3)), tag="angular_only_button")
    dpg.add_button(label="Lin + Ang", width = int(2*bottom_bar_width/denominator), height = int(bottom_bar_height/3), pos=(int(13*bottom_bar_width/denominator), int(bottom_bar_height/3)), tag="linear_and_angular_button")

#Font binding
dpg.bind_item_font("splash_text", "splash_font_250pt")
dpg.bind_item_font("automatic_sar_button", "font_48pt")
dpg.bind_item_font("automatic_maze_solve_button", "font_48pt")
dpg.bind_item_font("manual_button", "font_48pt")
dpg.bind_item_font("camera_feed_sel", "font_48pt")
dpg.bind_item_font("map_feed_sel", "font_48pt")
dpg.bind_item_font("position_title_auto", "font_48pt")
dpg.bind_item_font("x_position_auto", "font_30pt")
dpg.bind_item_font("y_position_auto", "font_30pt")
dpg.bind_item_font("orientation_title_auto", "font_48pt")
dpg.bind_item_font("orientation_auto", "font_30pt")
dpg.bind_item_font("velocity_title_auto", "font_48pt")
dpg.bind_item_font("linear_vel_auto", "font_30pt")
dpg.bind_item_font("angular_vel_auto", "font_30pt")
dpg.bind_item_font("position_title_manual", "font_48pt")
dpg.bind_item_font("x_position_manual", "font_30pt")
dpg.bind_item_font("y_position_manual", "font_30pt")
dpg.bind_item_font("orientation_title_manual", "font_48pt")
dpg.bind_item_font("orientation_manual", "font_30pt")
dpg.bind_item_font("velocity_title_manual", "font_48pt")
dpg.bind_item_font("linear_vel_slider", "font_48pt")
dpg.bind_item_font("angular_vel_title_manual", "font_48pt")
dpg.bind_item_font("angular_vel_slider", "font_48pt")
dpg.bind_item_font("control_panel_title", "font_48pt")
dpg.bind_item_font("switch_mode_button", "font_30pt")
dpg.bind_item_font("control_panel_title_manual", "font_48pt")
dpg.bind_item_font("switch_mode_button_manual", "font_30pt")
dpg.bind_item_font("stop_button_manual", "font_30pt")
dpg.bind_item_font("linear_only_button", "font_30pt")
dpg.bind_item_font("angular_only_button", "font_30pt")
dpg.bind_item_font("linear_and_angular_button", "font_30pt")

# Button Callbacks
def automatic_sar_callback(sender, data):
    dpg.hide_item("splash_screen")
    dpg.show_item("feed_selection")
    dpg.show_item("ugv_video")
    dpg.show_item("ugv_info_auto")
    dpg.show_item("bottom_bar_auto")
    op = operating_mode.AUTOMATIC_SAR
    mode_msg.data = op.value
    mode_pub.publish(mode_msg)
dpg.set_item_callback("automatic_sar_button", automatic_sar_callback)

def automatic_maze_solve_callback(sender, data):
    dpg.hide_item("splash_screen")
    dpg.show_item("feed_selection")
    dpg.show_item("ugv_video")
    dpg.show_item("ugv_info_auto")
    dpg.show_item("bottom_bar_auto")
    op = operating_mode.AUTOMATIC_MAZE_SOLVE
    mode_msg.data = op.value
    mode_pub.publish(mode_msg)
dpg.set_item_callback("automatic_maze_solve_button", automatic_maze_solve_callback)

def manual_callback(sender, data):
    dpg.hide_item("splash_screen")
    dpg.show_item("feed_selection")
    dpg.show_item("ugv_video")
    dpg.show_item("ugv_info_manual")
    dpg.show_item("bottom_bar_manual")
    op = operating_mode.MANUAL
    mode_msg.data = op.value
    mode_pub.publish(mode_msg)
dpg.set_item_callback("manual_button", manual_callback)

def camera_feed_callback(sender, data):
    dpg.show_item("ugv_video")
    dpg.hide_item("ugv_map")
dpg.set_item_callback("camera_feed_sel", camera_feed_callback)

def map_feed_callback(sender, data):
    dpg.show_item("ugv_map")
    dpg.hide_item("ugv_video")
dpg.set_item_callback("map_feed_sel", map_feed_callback)

def switch_mode_callback(sender, data):
    dpg.hide_item("feed_selection")
    dpg.hide_item("ugv_video")
    dpg.hide_item("ugv_info_auto")
    dpg.hide_item("ugv_info_manual")
    dpg.hide_item("bottom_bar_auto")
    dpg.hide_item("bottom_bar_manual")
    dpg.show_item("splash_screen")
    op = operating_mode.SPLASH_SCREEN
    mode_msg.data = op.value
    mode_pub.publish(mode_msg)
dpg.set_item_callback("switch_mode_button", switch_mode_callback)
dpg.set_item_callback("switch_mode_button_manual", switch_mode_callback)

def stop_callback(sender, data):
    manual_vel_msg.linear.x = 0
    manual_vel_msg.angular.z = 0
    manual_vel_pub.publish(manual_vel_msg)
dpg.set_item_callback("stop_button_manual", stop_callback)

def linear_only_callback(sender, data):
    manual_vel_msg.linear.x = dpg.get_value("linear_vel_slider")
    manual_vel_msg.angular.z = 0
    manual_vel_pub.publish(manual_vel_msg)
dpg.set_item_callback("linear_only_button", linear_only_callback)

def angular_only_callback(sender, data):
    manual_vel_msg.angular.z = dpg.get_value("angular_vel_slider")
    manual_vel_msg.linear.x = 0
    manual_vel_pub.publish(manual_vel_msg)
dpg.set_item_callback("angular_only_button", angular_only_callback)

def linear_and_angular_callback(sender, data):
    manual_vel_msg.linear.x = dpg.get_value("linear_vel_slider")
    manual_vel_msg.angular.z = dpg.get_value("angular_vel_slider")
    manual_vel_pub.publish(manual_vel_msg)
dpg.set_item_callback("linear_and_angular_button", linear_and_angular_callback)

# Register a callback to update the frame ~30 times per second
while dpg.is_dearpygui_running():
    dpg.render_dearpygui_frame()
dpg.start_dearpygui()
dpg.destroy_context()