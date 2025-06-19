#include "MotorControl.h"

// Constructor (Define here, not in .h file!)
Motor::Motor(int encA, int encB, int pwmFwd, int pwmRev, double Kp, double Ki, double Kd, double setpoint, int pwmChannel)
    : ENCODER_A(encA), ENCODER_B(encB), MOTOR_PWM_FWD(pwmFwd), MOTOR_PWM_REV(pwmRev),
      Kp(Kp), Ki(Ki), Kd(Kd), setpoint(setpoint), PWM_CHANNEL_FWD(pwmChannel), PWM_CHANNEL_REV(pwmChannel + 1) 
{
    encoder.attachHalfQuad(ENCODER_A, ENCODER_B);
    encoder.setCount(0);
    pinMode(MOTOR_PWM_FWD, OUTPUT);
    pinMode(MOTOR_PWM_REV, OUTPUT);
    ledcSetup(PWM_CHANNEL_FWD, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR_PWM_FWD, PWM_CHANNEL_FWD);
    ledcSetup(PWM_CHANNEL_REV, PWM_FREQ, PWM_RES);
    ledcAttachPin(MOTOR_PWM_REV, PWM_CHANNEL_REV);
}

// Update motor speed using velocity-form PID and return current speed
double Motor::update() {
    unsigned long currentTime = millis();
    if (currentTime - lastTime >= sampleTime) {
        lastTime = currentTime;
        speed = (encoder.getCount() * (1000.0 / sampleTime));
        encoder.clearCount(); 
        if (setpoint == 0) {
            ledcWrite(PWM_CHANNEL_FWD, 0);
            ledcWrite(PWM_CHANNEL_REV, 0);
            error_prev1 = 0;
            error_prev2 = 0;
            return speed;
        }
        error = setpoint - speed;
        output = output_prev + Kp * (error - error_prev1) + Ki * error + Kd * (error - 2 * error_prev1 + error_prev2);

        output = constrain(output, -255, 255);

        if (output > 0) {
            ledcWrite(PWM_CHANNEL_FWD, output);
            ledcWrite(PWM_CHANNEL_REV, 0);
        } else {
            ledcWrite(PWM_CHANNEL_FWD, 0);
            ledcWrite(PWM_CHANNEL_REV, -output);
        }

        output_prev = output;
        error_prev2 = error_prev1;
        error_prev1 = error;
    }
    return speed;
}

// Set new speed setpoint
void Motor::setSetpoint(double newSetpoint) {
    setpoint = newSetpoint;
}
