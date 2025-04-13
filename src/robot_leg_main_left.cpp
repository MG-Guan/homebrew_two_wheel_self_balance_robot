/* The main program for the robot leg.

Compile and upload the code to the ESP32 motor drive board (left leg)
*/

#include <Arduino.h> //Always needed
#include <ESP32Encoder.h>
#include <WiFi.h> //For WiFi functions
#include <HTTPClient.h> //For HTTP functions

#define MOTOR_SIDE 1 // 1 for left, -1 for right

unsigned long previousMillis = 0;
const char* ssid = "ESP32-Access-Point-SEP783_P3G2";
const char* password = "jian_leo";
int port = 80;
const char* url = "http://192.168.4.1/motor";
String server_temp;

const unsigned long FPS = 10;  // Asks for control signal every 100ms.
const unsigned long interval = 1000 / FPS; // Interval in milliseconds to request data from the server.

ESP32Encoder encoder;
#define SLEEP 16  // used to idle the motor and make sure the drive is working, no need to change
#define PMODE 27  // used to set the drive mode for the motor controller, leave as defined below
#define EN_PWM 32 // PWM to set motor "torque/speed, etc..."
#define DIR 33    // Change the direction of motor travel

#define PWM_RESOLUTION 8         // PWM resolution, 0-255


String httpGETRequest(const char* url) {
  HTTPClient http;
    
  // Your IP address with path or Domain name with URL path 
  http.begin(url);
  
  // Send HTTP POST request
  int httpResponseCode = http.GET();
  
  String payload = "--"; 
  
  if (httpResponseCode>0) {
   // Serial.print("HTTP Response code: ");
   // Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}


void setup() {
  Serial.begin(9600); 
  delay(1000);

  /* setup the pins for the motor control */
  pinMode(SLEEP, OUTPUT);
  pinMode(PMODE, OUTPUT);
  pinMode(DIR, OUTPUT);
  digitalWrite(SLEEP, HIGH);
  digitalWrite(PMODE, LOW);
  digitalWrite(DIR, LOW);
  ledcSetup(0, 10000, PWM_RESOLUTION);
  ledcAttachPin(EN_PWM, 0);

  encoder.attachHalfQuad(25, 26);
  encoder.setCount(0);

  // set starting count value after attaching
  Serial.println("Encoder Start = " + String((int32_t)encoder.getCount()));

  WiFi.begin(ssid, password);
  Serial.println("Connecting");
  while(WiFi.status() != WL_CONNECTED) { 
    delay(500);
    Serial.print(".");
    Serial.println("");
    Serial.print("Connected to WiFi network with IP Address: ");
    Serial.println(WiFi.localIP());
  }
  Serial.println("initialized");
}

int control_signal = 0;

void loop() {
  // Send a http request to the server, to get the temperature measured by the server.
  String control_signal_str = "0";
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis >= interval) {
     // Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED ){ 
       control_signal_str = httpGETRequest(url);
       control_signal = control_signal_str.toInt();

       previousMillis = currentMillis;
    }
  }

  // Change the signal sign based on the motor side.
  control_signal *= MOTOR_SIDE;

  // Control the motor with the control signal.
  if (control_signal > 0) {
    digitalWrite(DIR, HIGH); // Forward direction
  } else {
    digitalWrite(DIR, LOW); // Reverse direction
  }
  // The control_signle should between [1, 100], 
  // where 1 means the minimum value of the rotation,
  // and 100 means the maximum value of the rotation.
  // The motor need to map the control signal to the PWM value.
  int control_signal_value = abs(control_signal);
  const int MIN_PWM_TO_ROTATE = 10; // Minimum PWM to rotate the motor.
  int max_pwm = pow(2, PWM_RESOLUTION) - 1; // Maximum PWM value.
  int pwm = map(control_signal_value, 0, 100, 0, max_pwm);

  if (pwm < MIN_PWM_TO_ROTATE) {
    pwm = MIN_PWM_TO_ROTATE;
  }

  float ratio = pwm * 1.0 / max_pwm;
  ledcWrite(0, pwm); // Set the PWM signal to the motor.
  Serial.println("Control signal: " + control_signal_str + " PWM: " + String(pwm) + " ratio: " + String(ratio));
  delay(20);

  /////////////////////////////
  // Test motor.
  /////////////////////////////
  // for (int i = -100; i < 100; i+=5) {
  //   if (i < 0) {
  //     digitalWrite(DIR, HIGH); // Forward direction
  //   } else {
  //     digitalWrite(DIR, LOW); // Reverse direction
  //   }
  //   float ratio = abs(i) * 1.0 / pow(2, PWM_RESOLUTION);
  //   Serial.println("PWM to apply: " + String(i) + " ratio: " + String(ratio));
  //   ledcWrite(0, abs(i)); // Set the PWM signal to the motor.
  //   delay(1000);
  // }
}
