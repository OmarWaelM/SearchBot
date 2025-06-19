#ifndef MQTT_HANDLER_H
#define MQTT_HANDLER_H

#include <PubSubClient.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ROSTopics.h>

// MQTT connection settings
const char* mqtt_server = "your_path.emqxsl.com"; //Change to your EMQX link
const char* mqtt_user = "your_username"; //Change to your EMQX username
const char * mqtt_password = "your_password"; //Change to your EMQX password
const char* mqtt_topic_vel = "ugv/cmd_vel";
const char* mqtt_topic_update_pos = "ugv/update_pos";
const char* mqtt_topic_led = "ugv/LED";
const char* mqtt_topic_position = "ugv/position";
WiFiClientSecure espClient;
PubSubClient mqtt_client(espClient);

const char* root_ca = \
"-----BEGIN CERTIFICATE-----\n"
"MIIDrzCCApegAwIBAgIQCDvgVpBCRrGhdWrJWZHHSjANBgkqhkiG9w0BAQUFADBh\n"
"MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
"d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBD\n"
"QTAeFw0wNjExMTAwMDAwMDBaFw0zMTExMTAwMDAwMDBaMGExCzAJBgNVBAYTAlVT\n"
"MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n"
"b20xIDAeBgNVBAMTF0RpZ2lDZXJ0IEdsb2JhbCBSb290IENBMIIBIjANBgkqhkiG\n"
"9w0BAQEFAAOCAQ8AMIIBCgKCAQEA4jvhEXLeqKTTo1eqUKKPC3eQyaKl7hLOllsB\n"
"CSDMAZOnTjC3U/dDxGkAV53ijSLdhwZAAIEJzs4bg7/fzTtxRuLWZscFs3YnFo97\n"
"nh6Vfe63SKMI2tavegw5BmV/Sl0fvBf4q77uKNd0f3p4mVmFaG5cIzJLv07A6Fpt\n"
"43C/dxC//AH2hdmoRBBYMql1GNXRor5H4idq9Joz+EkIYIvUX7Q6hL+hqkpMfT7P\n"
"T19sdl6gSzeRntwi5m3OFBqOasv+zbMUZBfHWymeMr/y7vrTC0LUq7dBMtoM1O/4\n"
"gdW7jVg/tRvoSSiicNoxBN33shbyTApOB6jtSj1etX+jkMOvJwIDAQABo2MwYTAO\n"
"BgNVHQ8BAf8EBAMCAYYwDwYDVR0TAQH/BAUwAwEB/zAdBgNVHQ4EFgQUA95QNVbR\n"
"TLtm8KPiGxvDl7I90VUwHwYDVR0jBBgwFoAUA95QNVbRTLtm8KPiGxvDl7I90VUw\n"
"DQYJKoZIhvcNAQEFBQADggEBAMucN6pIExIK+t1EnE9SsPTfrgT1eXkIoyQY/Esr\n"
"hMAtudXH/vTBH1jLuG2cenTnmCmrEbXjcKChzUyImZOMkXDiqw8cvpOp/2PV5Adg\n"
"06O/nVsJ8dWO41P0jmP6P6fbtGbfYmbW0W5BjfIttep3Sp+dWOIrWcBAI+0tKIJF\n"
"PnlUkiaY4IBIqDfv8NZ5YBberOgOzW6sRBc4L0na4UU+Krk2U886UAb3LujEV0ls\n"
"YSEY1QSteDwsOoBrp+uvFRTp2InBuThs4pFsiv9kuXclVzDAGySj4dzp30d8tbQk\n"
"CAUw7C29C79Fv1C5qfPrmAESrciIxpg0X40KPMbp1ZWVbd4=\n"
"-----END CERTIFICATE-----\n";

//  to process received MQTT messages
void mqtt_callback(char* topic, byte* payload, unsigned int length)
 {
  Serial.print("Message arrived [");
  String msg = "";
  for (int i = 0; i < length; i++) 
  {
    msg += (char)payload[i];
  }

  if (strcmp(topic, mqtt_topic_vel) == 0)
  {
    float linear, angular;
    if (sscanf(msg.c_str(), "%f,%f", &linear, &angular) == 2) 
    {
        // checks if the incoming msg is 2 float variables seperated by comma
        geometry_msgs::Twist twist_msg;
        twist_msg.linear.x = linear;
        twist_msg.angular.z = angular;
        cmd_vel_callback(twist_msg);
    }
  }
  else if (strcmp(topic, mqtt_topic_update_pos) == 0)
  {
    float x, y, yaw;
    if (sscanf(msg.c_str(), "%f,%f,%f", &x, &y, &yaw) == 3) 
    {
        geometry_msgs::Pose pose_msg;
        pose_msg.position.x = x;
        pose_msg.position.y = y;
        pose_msg.orientation.z = yaw;
        pose_msg.orientation.w = 1.0;
        update_pos_callback(pose_msg);
    }
  }
  else if (strcmp(topic, mqtt_topic_led) == 0)
  {
    std_msgs::Empty msg;
    led_callback(msg);
  }

}

// Setup MQTT 
void setupMQTT() 
{  
  mqtt_client.setServer(mqtt_server, 8883);
  mqtt_client.setCallback(mqtt_callback);
  espClient.setCACert(root_ca);
  pinMode(13, OUTPUT);
}

// Reconnect to broker
void handleMQTT() 
{
  if (!mqtt_client.connected()) 
  {
    while (!mqtt_client.connected()) 
    {
      Serial.print("Connecting to MQTT broker...");
      if (mqtt_client.connect("ugvclient", mqtt_user, mqtt_password)) 
      {
        Serial.println("Connected!");
        mqtt_client.subscribe(mqtt_topic_vel);
        mqtt_client.subscribe(mqtt_topic_update_pos);
        mqtt_client.subscribe(mqtt_topic_led);
      } 
      else 
      {
        Serial.print("Failed, rc=");
        Serial.print(mqtt_client.state());
        Serial.println(". Retrying in 2 seconds...");
        delay(2000);
      }
    }
  }
  mqtt_client.loop();
}

void pub_odometry(float speedR, float speedL, float current_yaw) 
{
  if (mqtt_client.connected()) 
  {
    if (millis() - rosLastTime > rosTime) 
    {    
        nav_msgs::Odometry odom = forward_kinematics(speedR, speedL, current_yaw);
        String msg = String(odom.pose.pose.position.x) + "," + 
                    String(odom.pose.pose.position.y) + "," + 
                    String(odom.pose.pose.position.z) + "," +
                    String(odom.pose.pose.orientation.x) + "," +
                    String(odom.pose.pose.orientation.y) + "," +
                    String(odom.pose.pose.orientation.z) + "," +
                    String(odom.pose.pose.orientation.w) + "," +
                    String(odom.twist.twist.linear.x) + "," +
                    String(odom.twist.twist.linear.y) + "," +
                    String(odom.twist.twist.linear.z) + "," +
                    String(odom.twist.twist.angular.x) + "," +
                    String(odom.twist.twist.angular.y) + "," +
                    String(odom.twist.twist.angular.z);

        mqtt_client.publish(mqtt_topic_position, msg.c_str());
        rosLastTime = millis();
    }
  }
 
}

#endif 