#!/usr/bin/env python3
import rospy
import json
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
import paho.mqtt.client as mqtt
import os

# ################################################################################
# This code bridges between ROS topics and MQTT messages.
# It subscribes to ROS topics for command velocity, LED control, and pose updates,
# and forwards these messages to a MQTT broker (EMQX).
# and publishes odometry data received from an ESP32 from the MQTT broker.
# The MQTT broker is configured to use a SSL certificate locatedin the cert directory.
# Change the placeholder link, username and password with your EMQX server details (lines 90, 99)
# ################################################################################

# Called when MQTT connects successfully
def on_connect(client, userdata, flags, rc):
    client.subscribe("ugv/position")

# Called when MQTT receives telemetry from ESP32
def on_message(client, userdata, msg):
    payload = msg.payload.decode()

    try:
        # Expecting: x,y,z,qx,qy,qz,qw,lin_x,lin_y,lin_z,ang_x,ang_y,ang_z
        data = list(map(float, payload.split(",")))
        if len(data) != 13:
            rospy.logerr("Invalid odometry CSV format")
            return

        odometry_msg = Odometry()
        odometry_msg.header.stamp = rospy.Time.now()
        odometry_msg.header.frame_id = "odom"
        odometry_msg.child_frame_id = "base_link"

        # Pose
        odometry_msg.pose.pose.position.x = data[0]
        odometry_msg.pose.pose.position.y = data[1]
        odometry_msg.pose.pose.position.z = data[2]
        odometry_msg.pose.pose.orientation.x = data[3]
        odometry_msg.pose.pose.orientation.y = data[4]
        odometry_msg.pose.pose.orientation.z = data[5]
        odometry_msg.pose.pose.orientation.w = data[6]

        # Twist
        odometry_msg.twist.twist.linear.x = data[7]
        odometry_msg.twist.twist.linear.y = data[8]
        odometry_msg.twist.twist.linear.z = data[9]
        odometry_msg.twist.twist.angular.x = data[10]
        odometry_msg.twist.twist.angular.y = data[11]
        odometry_msg.twist.twist.angular.z = data[12]

        odom_pub.publish(odometry_msg)

    except Exception as e:
        rospy.logerr(f"Error parsing Odometry CSV data: {e}")


def cmd_vel_callback(msg):
    linear = msg.linear.x
    angular = msg.angular.z
    # Format to "0.4,0.2" string to send over MQTT
    command = f"{linear:.2f},{angular:.2f}"
    mqtt_client.publish("ugv/cmd_vel", command)

def LED_callback(msg):
    rospy.loginfo("LED command received")
    mqtt_client.publish("ugv/LED", "1")  # Send LED command to ESP32

def update_pose_callback(msg):
    # Convert Pose to string and publish to MQTT
    pose = f"{msg.position.x:.2f},{msg.position.y:.2f},{msg.orientation.z:.2f}"
    mqtt_client.publish("ugv/update_pose", pose)

rospy.init_node("MQTT_bridge")
odom_pub = rospy.Publisher("/ugv/position", Odometry, queue_size=10)
rospy.Subscriber("/ugv/cmd_vel", Twist, cmd_vel_callback)
rospy.Subscriber("/ugv/LED", Empty, LED_callback)
rospy.Subscriber("/ugv/update_pose", Pose, update_pose_callback)


mqtt_client = mqtt.Client(protocol=mqtt.MQTTv311)
script_dir = os.path.dirname(os.path.realpath(__file__))
tls_dir = os.path.join(script_dir, "../cert/emqxsl-ca.crt")

mqtt_client.tls_set(ca_certs=tls_dir)
mqtt_client.username_pw_set("your_username", "your_password")
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message
connected = False

if __name__ == "__main__":
    while not connected:
        try:
            print("Connecting to MQTT broker...")
            mqtt_client.connect("your_link.emqxsl.com", 8883, 60)
            connected = True
            print("Connected to MQTT broker")
        except Exception as e:
            print(f"MQTT connection failed: {e}")
            rospy.sleep(2)

    mqtt_client.loop_start()
    rospy.spin()
