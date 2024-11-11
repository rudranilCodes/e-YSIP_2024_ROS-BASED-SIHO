#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "FreeRTOSConfig.h"
#include "thijs_rplidar.h"
#include "mpu9250.h"
#include <MadgwickAHRS.h>

//FreeRTOS variables
TaskHandle_t Task1Handle;
TaskHandle_t Task2Handle;
TaskHandle_t Task3Handle;
TaskHandle_t Task4Handle;


//UDP variables
const int UDP_PORT_1= 4210;
const int UDP_PORT_2=8888;
const int UDP_PORT_3=6666;
const int UDP_PORT_4=7777;


const char* ssid = "Oneplus";  // Replace with your WiFi SSID
const char* password = "hello@123";  // Replace with your WiFi password
const char* udpAddress = "192.168.241.200"; // Replace with your laptop's IP address

WiFiUDP udp1;
WiFiUDP udp2;
WiFiUDP udp3;
WiFiUDP udp4;

//Buffer datas
byte buffer1[4*4];  
byte buffer2[360*4];
byte buffer3[9*4]; 
char incomingPacket[255]; 

/* Mpu9250 object */
bfs::Mpu9250 imu;
Madgwick filter;
float acc_x;
float acc_y;
float acc_z;

float gyr_x;
float gyr_y;
float gyr_z;
unsigned long millisOld;
float roll, pitch, heading;

//Lidar variables
#define lidarDebugSerial Serial
float lidar_scan[360]={0.0};
struct lidarMotorHandler {  // not really needed (and (currently) very ESP32-bound) but somewhat futureproof
  const uint8_t pin;
  const uint32_t freq; //Hz
  const uint8_t channel; // an ESP32 ledc specific thing
  const bool activeHigh; // depends on your specific hardware setup (CTRL_MOTO should be driven to the same voltage as 5V_MOTO (which can range from 5 to 9V), i think)
  lidarMotorHandler(const uint8_t pin, const bool activeHigh=true, const uint32_t freq=500, /*const uint8_t res=8,*/ const uint8_t channel=0) : 
                    pin(pin), freq(freq), /*res(res),*/ channel(channel), activeHigh(activeHigh) {}
  void init() {
    ledcSetup(channel, freq, 8);
    ledcAttachPin(pin, channel);
    setPWM(0);
  }
  inline void setPWM(uint8_t newPWMval) {ledcWrite(channel, activeHigh ? newPWMval : (255-newPWMval));}
};


lidarMotorHandler motorHandler(25);
RPlidar lidar(Serial2);

bool keepSpinning = true;

void dataHandler(RPlidar* lidarPtr, uint16_t dist, uint16_t angle_q6, uint8_t newRotFlag, int8_t quality) {
  float distFloat = dist; // unit is mm directly
  int angleDegreesFloat = angle_q6 * 0.015625;
    lidar_scan[angleDegreesFloat]=distFloat/1000;
    // Serial.println(angleDegreesFloat);
}


//cmd_vel variables

float linear_speed=0.0;
float angular_speed=0.0;

// Define the encoder pins for Motor 1
const int encoder1PinA = 39;  // Change to the pin connected to encoder 1 output A
const int encoder1PinB = 13;  // Change to the pin connected to encoder 1 output B

// Define the encoder pins for Motor 2
const int encoder2PinA = 15;  // Change to the pin connected to encoder 2 output A
const int encoder2PinB = 14;  // Change to the pin connected to encoder 2 output B

// Define the encoder pins for Motor 3
const int encoder3PinA = 34;  // Change to the pin connected to encoder 3 output A
const int encoder3PinB = 35;  // Change to the pin connected to encoder 3 output B

// Define the encoder pins for Motor 4
const int encoder4PinA = 19;  // Change to the pin connected to encoder 4 output A
const int encoder4PinB = 18;  // Change to the pin connected to encoder 4 output B

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
  udp3.begin(UDP_PORT_3);
  udp4.begin(UDP_PORT_4);

  Serial.println("UDP Connection began");

  motorHandler.init();
  lidar.init(16, 17);
  lidar.postParseCallback = dataHandler; // set dat handler function

  lidar.printLidarInfo();
  Serial.println();
  motorHandler.setPWM(255);

  while(!lidar.connectionCheck()) { Serial.println("connectionCheck() failed"); }

  delay(10);
  motorHandler.setPWM(255);
  bool startSuccess = lidar.startStandardScan();
  
  Serial.println("LiDAR Started!!");

  Wire.begin();
  Wire.setClock(400000);
  /* I2C bus,  0x68 address */
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  /* Set the sample rate divider */
  if (!imu.ConfigSrd(9)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }
  
  //Madgwick setup

  bool status = imu.ConfigAccelRange(bfs::Mpu9250::ACCEL_RANGE_2G);
  if (!status) {
  Serial.println("Error configured acceleration_range");
  }

  bool status2 = imu.ConfigGyroRange(bfs::Mpu9250::GYRO_RANGE_250DPS);
  if (!status2) {
     Serial.println("Error configured gyro_range");
  }
  millisOld=millis();

  Serial.println("IMU Started!!");


  //Create Task 1
  xTaskCreatePinnedToCore(
    Task1,        // Task function
    "Task1",      // Name of the task
    2048*10,         // Stack size
    NULL,         // Parameter
    1,            // Priority
    &Task1Handle, // Task handle
    1             // Core
  );

  //Create Task 2
  xTaskCreatePinnedToCore(
    Task2,        // Task function
    "Task2",      // Name of the task
    2048*5,         // Stack size
    NULL,         // Parameter
    1,            // Priority
    &Task2Handle, // Task handle
    1             // Core
  );

  //Create Task 3
  xTaskCreatePinnedToCore(
    Task3,        // Task function
    "Task3",      // Name of the task
    2048*10,      // Stack size
    NULL,         // Parameter
    1,            // Priority
    &Task3Handle, // Task handle
    1           // Core
  );

  Create Task 4
  xTaskCreatePinnedToCore(
    Task4,        // Task function
    "Task4",      // Name of the task
    2048*5,      // Stack size
    NULL,         // Parameter
    1,            // Priority
    &Task4Handle, // Task handle
    1           // Core
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
    delay(100);
  }
  
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

    Serial.print("Linear speed: ");
    Serial.print(linear_speed);
    Serial.print(" Angular speed: "); 
    Serial.println(angular_speed);
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
    analogWrite(motor1PwmPin,- speed1);
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

void Task3( void * pvParameters ){

  for(;;){
    if(keepSpinning) {
      uint32_t extraSpeedTimer = micros();
      int8_t datapointsProcessed = lidar.handleData(false, false); // read lidar data and send it to the callback function. Parameters are: (includeInvalidMeasurements, waitForChecksum)

      if(datapointsProcessed < 0) { keepSpinning = false; lidar.stopScan(); } // handleData() returns -1 if it encounters an error
        } 
      else {
        motorHandler.setPWM(0);
      }

    int arrayLength2 = sizeof(lidar_scan) / sizeof(lidar_scan[0]);
    memcpy(buffer2, lidar_scan, arrayLength2 * sizeof(float));
    udp3.beginPacket(udpAddress, UDP_PORT_3); // Change "destination_ip_address" to the IP address of the destination
    udp3.write(buffer2, arrayLength2 * sizeof(float));
    udp3.endPacket();
      
  } 
}

void Task4(void *pvParameters) {
  for(;;) {

    if (imu.Read()) {

    acc_x = imu.accel_x_mps2();
    acc_y = -imu.accel_y_mps2();
    acc_z = -imu.accel_z_mps2();

    gyr_x=imu.gyro_x_radps();
    gyr_y=imu.gyro_y_radps();
    gyr_z=imu.gyro_z_radps();

    filter.updateIMU(gyr_x*57.2958,gyr_y*57.2958,gyr_z*57.2958,acc_x/9.81,acc_y/9.81,acc_z/9.81);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();

    }
     if(millis()-millisOld>100){
      float imuArray[9]={roll,pitch,heading,acc_x,acc_y,acc_z,gyr_x,gyr_y,gyr_z};
      int arrlen = sizeof(imuArray) / sizeof(imuArray[0]);
      memcpy(buffer3, imuArray, arrlen * sizeof(float));
      udp4.beginPacket(udpAddress, UDP_PORT_4); // Change "destination_ip_address" to the IP address of the destination
      udp4.write(buffer3, arrlen * sizeof(float));
      udp4.endPacket();
      Serial.println(heading);
      }
  
}
}


void loop() {

}
