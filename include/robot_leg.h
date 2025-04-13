#ifndef ROBOT_LEG_H
#define ROBOT_LEG_H

#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <cmath>
#include <ESP32Encoder.h>
#include <freertos/FreeRTOS.h>  
#include <freertos/semphr.h> 
#include <Wire.h>

#define SLEEP 16  // used to idle the motor and make sure the drive is working, no need to change
#define PMODE 27  // used to set the drive mode for the motor controller, leave as defined below
#define EN_PWM 32 // PWM to set motor "torque/speed, etc..."
#define DIR 33    // Change the direction of motor travel

#define PWM_RESOLUTION 8         // PWM resolution, 0-255


struct SensorData {
    bool is_new_data = false; // Flag to indicate if new data is available
    float delta_t = 0.0; // Time difference
    float roll_deg = 0.0; // Roll angle, in [-180, 180] degrees
};

struct Parameters {
    int mode = 0; // Mode of operation, 0 for stop, 1 for run.
    float fps = 50.0; // Frames per second to update robot brain
    float max_value = 255.0; // Maximum control signal value
    float alpha = 0.01; // Alpha value for gain to control signal conversion
    float kp = 0.1; // PID controller proportional gain.
    float ki = 0.1; // PID controller integral gain.
    float kd = 0.1; // PID controller derivative gain.
};

class RobotLeg {
public:
    RobotLeg(int side, int min_pwm=10) : _side(side), _min_pwm(min_pwm) {}

    bool Begin() {
        Serial.begin(9600);
        delay(1000);

        // Setup the pins for the motor control.
        pinMode(SLEEP, OUTPUT);
        pinMode(PMODE, OUTPUT);
        pinMode(DIR, OUTPUT);
        digitalWrite(SLEEP, HIGH);
        digitalWrite(PMODE, LOW);
        digitalWrite(DIR, LOW);
        ledcSetup(0, 10000, PWM_RESOLUTION);
        ledcAttachPin(EN_PWM, 0);

        _encoder.attachHalfQuad(25, 26);
        _encoder.setCount(0);

        // set starting count value after attaching
        Serial.println("Encoder Start = " + String((int32_t)_encoder.getCount()));

        return true;
    }

    void setControlSignalAndDrive(int signal) {
         // Change the signal sign based on the motor side.
        _control_signal *= signal * _side;

        // Control the motor with the control signal.
        if (_control_signal > 0) {
            digitalWrite(DIR, HIGH); // Forward direction
        } else {
            digitalWrite(DIR, LOW); // Reverse direction
        }
        // The control_signle should between [1, 100], 
        // where 1 means the minimum value of the rotation,
        // and 100 means the maximum value of the rotation.
        // The motor need to map the control signal to the PWM value.
        int abs_control_signal = abs(_control_signal);
        const int MIN_PWM_TO_ROTATE = 10; // Minimum PWM to rotate the motor.
        int pwm = map(abs_control_signal, 0, 100, _min_pwm, _max_pwm);

        ledcWrite(0, pwm); // Set the PWM signal to the motor.

        float ratio = pwm * 1.0 / _max_pwm;
        Serial.println("Control signal: " + String(signal) + " PWM: " + String(pwm) + " ratio: " + String(ratio));
    }

private:
    // Working side.
    int _side = 1; // 1 for left, -1 for right.

    // Minimum PWM to rotate the motor.
    const int _min_pwm = 10; // Minimum PWM to rotate the motor from stationary position.
    const int _max_pwm = pow(2, PWM_RESOLUTION) - 1; // Maximum PWM value.

    // Encoder
    ESP32Encoder _encoder;

    // Control signal variables (PWM with sign).
    int _control_signal = 0; // Control signal.
    int _pwm = 0; // Control signal.
};

#endif // ROBOT_LEG_H