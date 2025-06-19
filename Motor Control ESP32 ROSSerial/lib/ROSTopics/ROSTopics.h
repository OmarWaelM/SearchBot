#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>
#include <ros.h>
#include <Kinematics.h>
#include <ESP32_WiFi.h>

ros::NodeHandle Robot;

/*----------ROSTopic Timiings---------------*/
const int rosTime = 105;
int rosLastTime = 0;

/*-------------Target Speeds---------------*/
float speedR = 0.0;
float speedL = 0.0;

/*----------------Publisher----------------*/
nav_msgs::Odometry odometry_msg;
ros::Publisher odometry_pub("/ugv/position", &odometry_msg);

void cmd_vel_callback(const geometry_msgs::Twist& msg) {
    inverse_kinematics(msg, speedR, speedL);
}

void update_pos_callback(const geometry_msgs::Pose& msg) {
    x_global = msg.position.x;
    y_global = msg.position.y;
    yaw_global = msg.orientation.z;
}

void led_callback(const std_msgs::Empty& msg) {
    digitalWrite(13, HIGH - digitalRead(13)); // Toggle LED on pin 13
}  

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/ugv/cmd_vel", cmd_vel_callback);

ros::Subscriber<geometry_msgs::Pose> update_pos_sub("/ugv/update_pos", update_pos_callback);

ros::Subscriber<std_msgs::Empty> led_sub("/ugv/LED", led_callback);

void ROSTopic_Setup() {
    Robot.getHardware()->setConnection(server, serverPort);
    Robot.initNode();
    Robot.subscribe(cmd_vel_sub);
    Robot.subscribe(update_pos_sub);
    Robot.subscribe(led_sub);
    Robot.advertise(odometry_pub);
    pinMode(13, OUTPUT);
}

void ROSTopic_Update(float speedR, float speedL, float current_yaw)
{
    if (millis() - rosLastTime > rosTime)
    { 
        nav_msgs::Odometry odom = forward_kinematics(speedR, speedL, current_yaw);
        odometry_pub.publish(&odom);
        Robot.spinOnce();
        rosLastTime = millis();
    }
}
