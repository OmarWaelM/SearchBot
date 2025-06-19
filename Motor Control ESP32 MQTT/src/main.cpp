#include <HardwareSerial.h>
#include <ESP32Encoder.h>
#include "MotorControl.h"
#include <Gyroscope.h>
//#include <ROSTopics.h>
#include <MQTT.h>

/*---------------Motor pins---------------*/
const int M1_For  = 17;
const int M1_Back = 5;
const int M2_For  = 19;
const int M2_Back = 18;
const int M3_For  = 25;
const int M3_Back = 26;
const int M4_For  = 33;
const int M4_Back = 32;

const int M1_encA = 4;
const int M1_encB = 16;
const int M2_encA = 39;
const int M2_encB = 36;
const int M3_encA = 14;
const int M3_encB = 27;
const int M4_encA = 35;
const int M4_encB = 34;

Motor motor1(M1_encA, M1_encB, M1_For, M1_Back, 0.02, 0.02, 0.01, 0, 0);
Motor motor2(M2_encA, M2_encB, M2_For, M2_Back, 0.02, 0.02, 0.01, 0, 2);
Motor motor3(M3_encA, M3_encB, M3_For, M3_Back, 0.02, 0.02, 0.01, 0, 4);
Motor motor4(M4_encA, M4_encB, M4_For, M4_Back, 0.02, 0.02, 0.01, 0, 6);


void setup() {
  Serial.begin(115200);
  setupWiFi();
  //ROSTopic_Setup();
  mpu_setup();
  setupMQTT();
}

void loop() {
  handleMQTT();
  double speed1 = motor1.update();
  double speed2 = motor2.update();
  double speed3 = motor3.update();
  double speed4 = motor4.update();

  mpu_loop();
  float yaw = ypr[0] * 180/M_PI;
  
  if (speedL == speedR)
  {
    motor1.setSetpoint(speedL);
    motor2.setSetpoint(speedL);
    motor3.setSetpoint(speedR);
    motor4.setSetpoint(speedR);
  }
  else
  {
    motor1.setSetpoint(speedL);
    motor2.setSetpoint(speedL);
    motor3.setSetpoint(speedR);
    motor4.setSetpoint(speedR);
  }

  //ROSTopic_Update((speed1 + speed2)/2, (speed3 + speed4)/2, yaw);
  pub_odometry((speed1 + speed2)/2, (speed3 + speed4)/2, yaw);
}