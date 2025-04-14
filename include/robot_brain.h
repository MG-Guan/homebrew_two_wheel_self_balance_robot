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
    int mode = 0; // Mode of operation, 0 for stop, 1 for work, 2 for debug.
    float fps = 50.0; // Frames per second to update robot brain
    int max_value = 10.0; // Maximum control signal value
    int max_neg_value = 10.0; // Maximum negative control signal value
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

        _sensor_data.is_new_data = false; // Reset the flag

        if (IMU.accelerationAvailable()) {
            float current_time = millis() * 1e-3; // Convert to seconds
            IMU.readAcceleration(ax, ay, az);
            float roll_deg = atan2(ay, az) / M_PI * 180.0;  // In degrees.
            float delta_t = current_time - _last_sensor_time;

            _sensor_data.delta_t = delta_t;
            _sensor_data.roll_deg = roll_deg;
            _sensor_data.is_new_data = true;

            _last_sensor_time = current_time;
        }
        return _sensor_data;
    }

    // Convert gain to control signal in range [-max_neg_value, max_value].
    int gainToControl(float gain, float alpha, int max_value, int max_neg_value) {
        float norm_gain = tanh(gain * alpha);  // Normalize gain to [-1.0, 1.0].
        if (norm_gain > 0) {
            return int(norm_gain * max_value);
        } else if (norm_gain < 0.0) {
            return int(norm_gain * max_neg_value);
        } else {
            return 0.0;
        }
    }

    // Get control signal based on target value, PID gain, and max value.
    int getControlSignal(double target, double alpha, int max_value, int max_neg_value) {
        SensorData sensor_data;
        Parameters params = safe_getParam();
        if (params.mode == 0) {
            Serial.println("Stop mode");
            return 0.0; // Stop mode, return 0.
        } else if (params.mode == 2) { // Debug mode.
            sensor_data = getNextMockSensorData();
        } else { // Work mode.
            sensor_data = getSensorData();
        }

        // float delta_t = 1e-2; // Default delta_t to avoid division by zero.
        // if (!sensor_data.is_new_data) {
        //     delta_t = sensor_data.delta_t;
        // }
        float pid_gain = _pid.compute(target, sensor_data.roll_deg, sensor_data.delta_t, params.kp, params.ki, params.kd);
        int control_signal = gainToControl(pid_gain, alpha, max_value, max_neg_value);
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
    int safe_getLastControlSignal() {
        int control_signal = 0;
        if(xSemaphoreTake(_control_signal_mutex, portMAX_DELAY)) {
             control_signal = _control_signal;
            xSemaphoreGive(_control_signal_mutex);
        }
        return control_signal;
    }

    // Set the control signal with mutex lock.
    int safe_setControlSignal(int control_signal) {
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

    SensorData getNextMockSensorData() {
        SensorData sensor_data;
        sensor_data.is_new_data = true;
        sensor_data.roll_deg = sin(_mock_roll_idx++) * _mock_max_roll; // Mock roll value.
        sensor_data.delta_t = 0.02; // Mock delta time.
        if (_mock_roll_idx > 100) {
            _mock_roll_idx = 0;
        }
        return sensor_data;
    }

    void step() {
        float target = safe_getTarget();
        Parameters params = safe_getParam();

        // Get the control signal based on the target value and parameters.
        int control_signal = getControlSignal(target, params.alpha, params.max_value, params.max_neg_value);

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
    int _control_signal = 0.0; // Control signal.
    SemaphoreHandle_t _control_signal_mutex; // Mutex for control signal access.

    // The target value for the PID controller.
    float _target = 0.0; // Target value for PID controller.
    SemaphoreHandle_t _target_mutex; // Mutex for target access.

    // Parameters.
    Parameters _params; // Parameters for the robot brain.
    PIDController _pid;
    SemaphoreHandle_t _params_mutex; // Mutex for parameters and pid access.

    // Debug variables.
    float _mock_max_roll = 20.0; // Mock roll value for testing.
    int _mock_roll_idx = 0;
};

#endif // ROBOT_BRAIN_H