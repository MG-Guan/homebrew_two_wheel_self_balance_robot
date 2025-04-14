/* The main program for the robot leg.

Compile and upload the code to the ESP32 motor drive board (left leg)
*/

#include <Arduino.h> //Always needed
#include <AsyncHTTPRequest_Generic.h>
#include <WiFi.h> //For WiFi functions
#include <HTTPClient.h> //For HTTP functions

#include "ssid.h"
#include "robot_leg.h"

#define MOTOR_SIDE 1 // 1 for left, -1 for right

unsigned long previousMillis = 0;
String server_temp;

const unsigned long FPS_UPDATE = 50;  // Asks for update pwm.
const unsigned long INTERVAL_UPDATE = 1000 / FPS_UPDATE;  // Asks for control signal every 100ms.

RobotLeg robot_leg(MOTOR_SIDE); // Create a robot leg object.

int control_signal = 0;
unsigned long lastRequestTime = 0;
unsigned long currTime = millis();

void sendHttpRequest(const char* url) {  
  HTTPClient http;
    
  // Your IP address with path or Domain name with URL path 
  http.begin(url);
  
  // Send HTTP POST request
  int httpResponseCode = http.GET();
  
  String payload = "--"; 
  
  if (httpResponseCode>0) {
    payload = http.getString();
    control_signal = payload.toInt();
    //Serial.println("Get control signal: " + String(control_signal));
    robot_leg.setControlSignalAndDrive(control_signal);
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();
} 

void setup() {
  Serial.begin(9600); 
  delay(1000);

  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) { 
    delay(500);
    Serial.print(".");
    Serial.println("");
    Serial.print("Connected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());
  }

  robot_leg.Begin(); // Initialize the robot leg.
  Serial.println("initialized");
}

void loop() {
  currTime = millis();
  if (currTime - lastRequestTime > INTERVAL_UPDATE) {
    sendHttpRequest(motor_url);
    lastRequestTime = currTime;
    int pwm = robot_leg.setControlSignalAndDrive(control_signal);
    float angular_velocity = robot_leg.getAngularVelocity();
    Serial.println(String(pwm) + "," + String(angular_velocity));
  }
}
