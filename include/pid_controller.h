#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

// PID Controller Header File
// This file defines the PID class for implementing a PID controller.

class PIDController {
public:
    float compute(float setpoint, float input, float dt, float kp, float ki, float kd) {
        float error = setpoint - input;
        float proportional = kp * error;
        _integral += error * dt;
        float integral = ki * _integral;
        float derivative = kd * (error - _last_error) / dt;
        _last_error = error;

        float gain = proportional + integral + derivative;
        // Serial.print("PID values:");
        // Serial.print(proportional);
        // Serial.print(",");
        // Serial.print(integral);
        // Serial.print(",");
        // Serial.println(derivative);
        return gain;
    }

    void reset() {
        _integral = 0.0;
        _last_error = 0.0;
    }
private:
    float _integral = 0.0;
    float _last_error = 0.0;
};

#endif // PID_CONTROLLER_H