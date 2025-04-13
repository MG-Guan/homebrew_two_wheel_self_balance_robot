/* The main program for the robot brain.
It initializes the IMU, sets up the web server, and handles incoming requests.

Compile and upload the code to the ESP32 sensor board with IMU.
*/
#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>

#include "robot_brain.h"
#include "ssid.h"

// Global variables for web server.
AsyncWebServer server(port);

// Global robot brain object.
RobotBrain robot_brain;

void setup() {
  Wire.begin();
  Serial.begin(9600);

  // Initialize robot brain.
  if (!robot_brain.Begin(LSM9DS1_ACCELRANGE_4G)) {
    Serial.println("Failed to initialize robot brain!");
    while (1);
  }

  // Setup Access point (AP) with service set identifier (SSID) and Password
  WiFi.softAP(ssid, password);
  // Get the softAP interface IP address
  IPAddress IP = WiFi.softAPIP();
  // Printeh the IP address in serial monitor
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // Service to get the control signal.
  server.on("/motor", HTTP_GET, [](AsyncWebServerRequest *request) {
    float control_pwm = robot_brain.safe_getLastControlSignal();
    // String json_response = "{\"pwm\": " + String(control_pwm) + "}";
    String control_pwm_str = String(control_pwm);
    request->send(200, "text/plain", control_pwm_str);
    // Serial.println(control_pwm_str);
  });

  // Service to set the target or params.
  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("target")) {
      String target_str = request->getParam("target")->value();
      float target = target_str.toFloat();
      robot_brain.safe_setTarget(target);
    }

    if (request->hasParam("mode")) {  // Make sure to pass the mode parameter as an indicator.
      // Parse the parameters from the string and set them in the robot brain.
      // This is a placeholder; actual parsing logic should be implemented.
      Parameters params = robot_brain.safe_getParam();

      if (request->hasParam("mode")) {
        String mode_str = request->getParam("mode")->value();
        params.mode = mode_str.toInt();
      }
      if (request->hasParam("alpha")) {
        params.alpha = request->getParam("alpha")->value().toFloat();
      }
      if (request->hasParam("fps")) {
        params.fps = request->getParam("fps")->value().toFloat();
      } 
      if (request->hasParam("max_value")) {
        params.max_value = request->getParam("max_value")->value().toFloat();
      }
      if (request->hasParam("kp")) {
        params.kp = request->getParam("kp")->value().toFloat();
      }
      if (request->hasParam("ki")) {
        params.ki = request->getParam("ki")->value().toFloat();
      }
      if (request->hasParam("kd")) {
        params.kd = request->getParam("kd")->value().toFloat();
      }
      robot_brain.safe_setParam(params);
    }
    request->send(200, "text/plain", "Parameters set.");
  });
  
  // Start the async web server.
  server.begin();
}

void loop() {
  robot_brain.step(); // Set control signal to robot brain with delay.
}
