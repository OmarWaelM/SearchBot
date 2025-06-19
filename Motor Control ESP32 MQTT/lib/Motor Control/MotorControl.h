#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <ESP32Encoder.h>
#include <Arduino.h>

// PID Controller Class for Motor Control
class Motor {
public:
    // Constructor (Only declare here, no definition!)
    Motor(int encA, int encB, int pwmFwd, int pwmRev, double Kp, double Ki, double Kd, double setpoint, int pwmChannel);

    // Function declarations (Only declare here)
    double update();
    void setSetpoint(double newSetpoint);

private:
    // Motor and encoder pins
    int ENCODER_A, ENCODER_B;
    int MOTOR_PWM_FWD, MOTOR_PWM_REV;

    // PWM Parameters
    int PWM_CHANNEL_FWD;
    int PWM_CHANNEL_REV;
    const int PWM_FREQ = 12500;
    const int PWM_RES = 8;

    // PID Parameters
    double Kp, Ki, Kd;
    double setpoint, speed, error, error_prev1 = 0, error_prev2 = 0;
    double output = 0, output_prev = 0;

    // Encoder
    ESP32Encoder encoder;

    // Timing variables
    unsigned long lastTime = 0;
    const double sampleTime = 50; // Sample time in milliseconds
};

#endif
