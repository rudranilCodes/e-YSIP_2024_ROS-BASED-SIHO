#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "FreeRTOSConfig.h"


// Task handles
TaskHandle_t Task1Handle;
TaskHandle_t Task2Handle;

//Buffer datas
byte buffer1[4*4];  
char incomingPacket[255]; 

float linear_speed=0.0;
float angular_speed=0.0;

//UDP ports
const int UDP_PORT_1= 4210;
const int UDP_PORT_2=8888;

//Connection Setup
const char* ssid = "Oneplus";  // Replace with your WiFi SSID
const char* password = "hello@123";  // Replace with your WiFi password
const char* udpAddress = "192.168.80.67"; // Replace with your laptop's IP address


//Initalisation of UDP
WiFiUDP udp1;
WiFiUDP udp2;


// Define the encoder pins for Motor 1
const int encoder1PinA = 13;  // Change to the pin connected to encoder 1 output A
const int encoder1PinB = 39;  // Change to the pin connected to encoder 1 output B

// Define the encoder pins for Motor 2
const int encoder2PinA = 14;  // Change to the pin connected to encoder 2 output A
const int encoder2PinB = 15;  // Change to the pin connected to encoder 2 output B

// Define the encoder pins for Motor 3
const int encoder3PinA = 35;  // Change to the pin connected to encoder 3 output A
const int encoder3PinB = 34;  // Change to the pin connected to encoder 3 output B

// Define the encoder pins for Motor 4
const int encoder4PinA = 18;  // Change to the pin connected to encoder 4 output A
const int encoder4PinB = 19;  // Change to the pin connected to encoder 4 output B

// Define the motor control pins
const int motor1PwmPin = 26;   // PWM pin for Motor 1
const int motor1DirPin = 2;   // Direction pin for Motor 1
const int motor2PwmPin = 27;   // PWM pin for Motor 2
const int motor2DirPin = 4;   // Direction pin for Motor 2
const int motor3PwmPin = 5;   // PWM pin for Motor 3
const int motor3DirPin = 32;   // Direction pin for Motor 3
const int motor4PwmPin = 23;   // PWM pin for Motor 4
const int motor4DirPin = 33;   // Direction pin for Motor 4

// Variables to store the encoder counts
volatile long encoder1Counts = 0;
volatile int lastEncoded1 = 0;

volatile long encoder2Counts = 0;
volatile int lastEncoded2 = 0;

volatile long encoder3Counts = 0;
volatile int lastEncoded3 = 0;

volatile long encoder4Counts = 0;
volatile int lastEncoded4 = 0;


// Interrupt service routine for encoder 1
void IRAM_ATTR handleEncoder1() {
  int MSB = digitalRead(encoder1PinA);  // Most significant bit
  int LSB = digitalRead(encoder1PinB);  // Least significant bit
  
  int encoded = (MSB << 1) | LSB;      // Convert the 2 pin value to a single number
  int sum  = (lastEncoded1 << 2) | encoded;  // Combine previous encoded value with current to get the direction

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder1Counts++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder1Counts--;

  lastEncoded1 = encoded;  // Store current encoded value for the next iteration
}

// Interrupt service routine for encoder 2
void IRAM_ATTR handleEncoder2() {
  int MSB = digitalRead(encoder2PinA);  // Most significant bit
  int LSB = digitalRead(encoder2PinB);  // Least significant bit
  
  int encoded = (MSB << 1) | LSB;      // Convert the 2 pin value to a single number
  int sum  = (lastEncoded2 << 2) | encoded;  // Combine previous encoded value with current to get the direction

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder2Counts++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder2Counts--;

  lastEncoded2 = encoded;  // Store current encoded value for the next iteration
}

// Interrupt service routine for encoder 3
void IRAM_ATTR handleEncoder3() {
  int MSB = digitalRead(encoder3PinA);  // Most significant bit
  int LSB = digitalRead(encoder3PinB);  // Least significant bit
  
  int encoded = (MSB << 1) | LSB;      // Convert the 2 pin value to a single number
  int sum  = (lastEncoded3 << 2) | encoded;  // Combine previous encoded value with current to get the direction

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder3Counts++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder3Counts--;

  lastEncoded3 = encoded;  // Store current encoded value for the next iteration
}

// Interrupt service routine for encoder 4
void IRAM_ATTR handleEncoder4() {
  int MSB = digitalRead(encoder4PinA);  // Most significant bit
  int LSB = digitalRead(encoder4PinB);  // Least significant bit
  
  int encoded = (MSB << 1) | LSB;      // Convert the 2 pin value to a single number
  int sum  = (lastEncoded4 << 2) | encoded;  // Combine previous encoded value with current to get the direction

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder4Counts++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder4Counts--;

  lastEncoded4 = encoded;  // Store current encoded value for the next iteration
}





void setup() {
  Serial.begin(115200);

  // Set encoder pins as inputs for Motor 1
  pinMode(encoder1PinA, INPUT);
  pinMode(encoder1PinB, INPUT);

  // Set encoder pins as inputs for Motor 2
  pinMode(encoder2PinA, INPUT);
  pinMode(encoder2PinB, INPUT);

  // Set encoder pins as inputs for Motor 3
  pinMode(encoder3PinA, INPUT);
  pinMode(encoder3PinB, INPUT);

  // Set encoder pins as inputs for Motor 4
  pinMode(encoder4PinA, INPUT);
  pinMode(encoder4PinB, INPUT);

  // Attach interrupts to the encoder pins for Motor 1
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), handleEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), handleEncoder1, CHANGE);

  // Attach interrupts to the encoder pins for Motor 2
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), handleEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinB), handleEncoder2, CHANGE);

  // Attach interrupts to the encoder pins for Motor 3
  attachInterrupt(digitalPinToInterrupt(encoder3PinA), handleEncoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder3PinB), handleEncoder3, CHANGE);

  // Attach interrupts to the encoder pins for Motor 4
  attachInterrupt(digitalPinToInterrupt(encoder4PinA), handleEncoder4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder4PinB), handleEncoder4, CHANGE);

  // Set motor control pins as outputs
  pinMode(motor1PwmPin, OUTPUT);
  pinMode(motor1DirPin, OUTPUT);
  pinMode(motor2PwmPin, OUTPUT);
  pinMode(motor2DirPin, OUTPUT);
  pinMode(motor3PwmPin, OUTPUT);
  pinMode(motor3DirPin, OUTPUT);
  pinMode(motor4PwmPin, OUTPUT);
  pinMode(motor4DirPin, OUTPUT);

  Serial.println("Pins allocation done");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  Serial.println("Wifi Connected");

  udp1.begin(UDP_PORT_1);
  udp2.begin(UDP_PORT_2);

  Serial.println("UDP Connection began");

  xTaskCreatePinnedToCore(
    Task1,        // Task function
    "Task1",      // Name of the task
    2048*10,         // Stack size
    NULL,         // Parameter
    1,            // Priority
    &Task1Handle, // Task handle
    1             // Core
  );

  // // Create Task 2
  xTaskCreatePinnedToCore(
    Task2,        // Task function
    "Task2",      // Name of the task
    2048*5,         // Stack size
    NULL,         // Parameter
    1,            // Priority
    &Task2Handle, // Task handle
    1             // Core
  );

  
}
void Task1(void *pvParameters) {
  for(;;) {
    float dataArray[4]={encoder1Counts,encoder2Counts,encoder3Counts,encoder4Counts};
    int arrayLength1 = sizeof(dataArray) / sizeof(dataArray[0]);
    memcpy(buffer1, dataArray, arrayLength1 * sizeof(float));

    udp1.beginPacket(udpAddress, UDP_PORT_1); // Change "destination_ip_address" to the IP address of the destination
    udp1.write(buffer1, arrayLength1 * sizeof(float));
    udp1.endPacket();
  }
  delay(50);
}

// Task 2: Print run-time stats
void Task2(void *pvParameters) {
  for(;;) {
  int packetSize = udp2.parsePacket();
  if (packetSize) {
    int len = udp2.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;
    }

    
    // Convert bytes back to float array
    int floatCount = packetSize / sizeof(float);
    float floats[floatCount];
    memcpy(floats, incomingPacket, packetSize);
    linear_speed=floats[0];
    angular_speed=floats[1];


  }

  float linear_x = linear_speed;
  float angular_z = angular_speed;

  int speed1 = 255 * (linear_x - angular_z);
  int speed2 = 255 * (linear_x + angular_z);
  int speed3 = 255 * (linear_x - angular_z);
  int speed4 = 255 * (linear_x + angular_z);

  // Ensure speed values are within valid PWM range
  speed1 = constrain(speed1, -255, 255);
  speed2 = constrain(speed2, -255, 255);
  speed3 = constrain(speed3, -255, 255);
  speed4 = constrain(speed4, -255, 255);


  if (speed1 >= 0) {
    digitalWrite(motor1DirPin, HIGH);
    analogWrite(motor1PwmPin, speed1);
  } else {
    digitalWrite(motor1DirPin, LOW);
    analogWrite(motor1PwmPin, -speed1);
  }

  // Set motor 2 direction and speed
  if (speed2 >= 0) {
    digitalWrite(motor2DirPin, HIGH);
    analogWrite(motor2PwmPin, speed2);
  } else {
    digitalWrite(motor2DirPin, LOW);
    analogWrite(motor2PwmPin, -speed2);
  }

  // Set motor 3 direction and speed
  if (speed3 >= 0) {
    digitalWrite(motor3DirPin, HIGH);
    analogWrite(motor3PwmPin, speed3);
  } else {
    digitalWrite(motor3DirPin, LOW);
    analogWrite(motor3PwmPin, -speed3);
  }

  // Set motor 4 direction and speed
  if (speed4 >= 0) {
    digitalWrite(motor4DirPin, HIGH);
    analogWrite(motor4PwmPin, speed4);
  } else {
    digitalWrite(motor4DirPin, LOW);
    analogWrite(motor4PwmPin, -speed4);
  }
  delay(50);

  }

}


void loop() {

}
