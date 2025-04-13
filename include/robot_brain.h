#ifndef ROBOT_BRAIN_H
#define ROBOT_BRAIN_H

#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <cmath>
#include <freertos/FreeRTOS.h>  
#include <freertos/semphr.h> 
#include <Wire.h>

#include "pid_controller.h"


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

class RobotBrain {
public:
    RobotBrain() {
        _control_signal_mutex = xSemaphoreCreateMutex();
        _target_mutex = xSemaphoreCreateMutex();
        _params_mutex = xSemaphoreCreateMutex();

        if (_control_signal_mutex == NULL || _target_mutex == NULL || _params_mutex == NULL) {
            Serial.println("Failed to create mutex!");
            while(1); // Handle mutex creation failure
        }
    }

    // Initialize the IMU and set the acceleration range.
    // The default range is 4G, but it can be changed to 2G, 8G, or 16G.
    bool Begin(uint8_t acceleration_range = LSM9DS1_ACCELRANGE_4G) {
        if (!IMU.begin()) {
            Serial.println("Failed to initialize IMU!");
            return false;
        }
        IMU.setAccelerationRange(acceleration_range);

        Serial.print("IMU initialized by range:");
        Serial.println(acceleration_range);

        getSensorData(); // Initialize sensor data and timer.
        return true;
    }

    const SensorData& getSensorData() {
        float ax = 0.0, ay = 0.0, az = 0.0;
        float roll = 0.0;

        _sensor_data.is_new_data = false; // Reset the flag

        if (IMU.accelerationAvailable()) {
            float current_time = millis() * 1e-3; // Convert to seconds
            IMU.readAcceleration(ax, ay, az);
            roll = atan2(-ay, sqrt(pow(az, 2) + pow(ax, 2)));  // In rad.
            float roll_deg = roll / M_PI * 180.0; // Convert to degrees 
            float delta_t = current_time - _last_sensor_time;

            _sensor_data.delta_t = delta_t;
            _sensor_data.roll_deg = roll_deg;
            _sensor_data.is_new_data = true;

            _last_sensor_time = current_time;
        }
        return _sensor_data;
    }

    // Convert gain to control signal in range [-max_value, max_value].
    double gainToControl(float gain, float alpha, float max_value) {
        float norm_gain = tanh(gain * alpha);  // Normalize gain to [-1.0, 1.0].
        return norm_gain * max_value;
    }

    // Get control signal based on target value, PID gain, and max value.
    double getControlSignal(double target, double alpha, double max_value) {
        Parameters params = safe_getParam();
        if (params.mode == 0) {
            Serial.println("Stop mode");
            return 0.0; // Stop mode, return 0.
        }

        const SensorData& sensor_data = getSensorData();

        float pid_gain = 0.0;
        float delta_t = 1e-4; // Default delta_t to avoid division by zero.
        if (!sensor_data.is_new_data) {
            delta_t = sensor_data.delta_t;
        }
        pid_gain = _pid.compute(target, sensor_data.roll_deg, delta_t, params.kp, params.ki, params.kd);
        float control_signal = gainToControl(pid_gain, alpha, max_value);
        Serial.print(sensor_data.roll_deg);
        Serial.print(",");
        Serial.print(sensor_data.delta_t);
        Serial.print(",");
        Serial.print(pid_gain);
        Serial.print(",");
        Serial.println(control_signal);
        return control_signal;
    }

    // Get the last control signal with mutex lock.
    float safe_getLastControlSignal() {
        float control_signal = 0.0;
        if(xSemaphoreTake(_control_signal_mutex, portMAX_DELAY)) {
             control_signal = _control_signal;
            xSemaphoreGive(_control_signal_mutex);
        }
        return control_signal;
    }

    // Set the control signal with mutex lock.
    float safe_setControlSignal(float control_signal) {
        if(xSemaphoreTake(_control_signal_mutex, portMAX_DELAY)) {
            _control_signal = control_signal;
            xSemaphoreGive(_control_signal_mutex);
        }
        return _control_signal;
    }

    float safe_getTarget() {
        float target = 0.0;
        if(xSemaphoreTake(_target_mutex, portMAX_DELAY)) {
            target = _target;
            xSemaphoreGive(_target_mutex);
        }
        return target;
    }

    void safe_setTarget(float target) {
        if(xSemaphoreTake(_target_mutex, portMAX_DELAY)) {
            _target = target;
            xSemaphoreGive(_target_mutex);
        }
    }

    Parameters safe_getParam() {
        Parameters params;
        if(xSemaphoreTake(_params_mutex, portMAX_DELAY)) {
            params = _params;
            xSemaphoreGive(_params_mutex);
        }
        return params;
    }

    void safe_setParam(Parameters params) {
        if(xSemaphoreTake(_params_mutex, portMAX_DELAY)) {
            _params = params;
            xSemaphoreGive(_params_mutex);
        }
    }

    void step() {
        float target = safe_getTarget();
        Parameters params = safe_getParam();

        // Get the control signal based on the target value and parameters.
        float control_signal = getControlSignal(target, params.alpha, params.max_value);

        // Set the control signal.
        safe_setControlSignal(control_signal);

        // Delay for the specified frames per second (fps).
        vTaskDelay(pdMS_TO_TICKS(1000.0 / params.fps));  // Delay with concurrency.
    }

private:
    // IMU variables
    float _last_sensor_time = 0.0; // Time of last sensor reading.

    // IMU object.
    SensorData _sensor_data;  // Stores the last sensor data.

    // Control signal variables (PWM with sign).
    float _control_signal = 0.0; // Control signal.
    SemaphoreHandle_t _control_signal_mutex; // Mutex for control signal access.

    // The target value for the PID controller.
    float _target = 0.0; // Target value for PID controller.
    SemaphoreHandle_t _target_mutex; // Mutex for target access.

    // Parameters.
    Parameters _params; // Parameters for the robot brain.
    PIDController _pid;
    SemaphoreHandle_t _params_mutex; // Mutex for parameters and pid access.
};

#endif // ROBOT_BRAIN_H