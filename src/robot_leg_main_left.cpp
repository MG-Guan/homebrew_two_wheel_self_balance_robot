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

const unsigned long FPS_UPDATE = 20;  // Asks for update pwm.
const unsigned long INTERVAL_UPDATE = 1000 / FPS_UPDATE;  // Asks for control signal every 100ms.

RobotLeg robot_leg(MOTOR_SIDE); // Create a robot leg object.

AsyncHTTPRequest async_http;  
String control_signal_str = "0";

void asyncHandleResponse(void* arg, AsyncHTTPRequest* request, int readyState) {  
  if (readyState == 4) { // Request is completed  
    if (request->responseHTTPcode() > 0) {  
      control_signal_str = request->responseText();  

      int control_signal = control_signal_str.toInt();
      robot_leg.setControlSignalAndDrive(control_signal);
    }
  }  
}  

void asyncHttpGETRequest(const char* url) {  
  if (async_http.readyState() == 0 || async_http.readyState() == 4) { // Ready for a new request  
    async_http.onReadyStateChange(asyncHandleResponse); // Set callback function  
    async_http.open("GET", url);  
    async_http.send();  
  }
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
  // Send a http request to the server, to get the temperature measured by the server.
  asyncHttpGETRequest(motor_url);
  delay(INTERVAL_UPDATE);
}
