#include "WiFi.h"
#include <ros.h>

IPAddress ip(192, 168, 35, 174);
IPAddress server(192, 168, 35, 169);
uint16_t serverPort = 11411;
const char*  ssid = "Galaxy A55 5G 14E1";
const char*  password = "abcdefgh";

void setupWiFi()
{
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { delay(500);Serial.print("."); }
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
    Serial.print("IP:   ");
    Serial.println(WiFi.localIP());
}