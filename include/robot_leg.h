#ifndef ROBOT_LEG_H
#define ROBOT_LEG_H

#include <algorithm>

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
#define ENCODER_RESOLUTION 0.088 // 360/4096


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

    int setControlSignalAndDrive(int signal) {
         // Change the signal sign based on the motor side.
        _control_signal = signal * _side;

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
        // int pwm = map(abs_control_signal, 0, 100, _min_pwm, _max_pwm);

        if (abs_control_signal < _min_pwm) {
            abs_control_signal = _min_pwm;
        } else if (abs_control_signal > _max_pwm) {
            abs_control_signal = _max_pwm;
        }

        ledcWrite(0, abs_control_signal); // Set the PWM signal to the motor.
        return _control_signal;

        // float ratio = abs_control_signal * 1.0 / _max_pwm;
        // Serial.println("PWM: " + String(_control_signal) + " ratio: " + String(ratio));
    }

    float getAngularVelocity() {
        // Calculate angular velocity (degrees/second)
        unsigned long current_time = millis();
        float delta_time = (current_time - _last_measured_time) * 1e-3; // Convert to seconds
        int32_t current_count = _encoder.getCount();
        float angular_velocity = ((current_count - _last_measured_count) * ENCODER_RESOLUTION) / delta_time;

        // Update last values for the next calculation
        _last_measured_time = current_time;
        _last_measured_count = current_count;

        return angular_velocity;
    }



private:
    // Working side.
    int _side = 1; // 1 for left, -1 for right.

    // Minimum PWM to rotate the motor.
    const int _min_pwm = 0; // Minimum PWM to rotate the motor from stationary position.
    const int _max_pwm = pow(2, PWM_RESOLUTION) - 1; // Maximum PWM value.

    // Encoder
    ESP32Encoder _encoder;

    // Control signal variables (PWM with sign).
    int _control_signal = 0; // Control signal.
    int _pwm = 0; // Control signal.

    // The fields to store the encoder status.
    int _last_measured_count = 0; // Last measured count.
    unsigned long _last_measured_time = 0; // Last measured time.
};

#endif // ROBOT_LEG_H