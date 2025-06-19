#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros.h>

float x_global = 0;
float y_global = 0;
float yaw_global = 0;
float prev_timestep = 0;

nav_msgs::Odometry forward_kinematics(float speedR, float speedL, float yaw)
{
    nav_msgs::Odometry output;
    int timestep = millis() - prev_timestep;
    
    float speedMperSecR = speedR * 0.065 * 2 * PI / 860.0;
    float speedMperSecL = speedL * 0.065 * 2 * PI / 860.0;
    
    float velocity = (speedMperSecR + speedMperSecL) / 2.0;
    float angularVelocity = (speedMperSecR - speedMperSecL) / 0.19;

    x_global = x_global + velocity * cos(yaw*PI/180) * timestep / 1000;
    y_global = y_global + velocity * sin(yaw*PI/180) * timestep / 1000;

    float x_velocity = velocity * cos(yaw*PI/180);
    float y_velocity = velocity * sin(yaw*PI/180);

    output.pose.pose.position.x = x_global;
    output.pose.pose.position.y = y_global;
    output.pose.pose.position.z = 0;
    output.pose.pose.orientation.x = 0;
    output.pose.pose.orientation.y = 0;
    output.pose.pose.orientation.z = yaw;
    output.pose.pose.orientation.w = 1;
    output.twist.twist.linear.x = x_velocity;
    output.twist.twist.linear.y = y_velocity;
    output.twist.twist.angular.z = angularVelocity;

    output.child_frame_id = "base_link";
    prev_timestep = millis();

    return output;
}

void inverse_kinematics(geometry_msgs::Twist msg, float &speedR, float &speedL)
{
    float x = msg.linear.x;
    float yaw = msg.angular.z;

    float speedmperSecR = x - (yaw * (0.19/2));
    float speedmperSecL = x + (yaw * (0.19/2));
    
    speedR = speedmperSecR * 860.0 / (2 * PI * 0.065);
    speedL = speedmperSecL * 860.0 / (2 * PI * 0.065);
}