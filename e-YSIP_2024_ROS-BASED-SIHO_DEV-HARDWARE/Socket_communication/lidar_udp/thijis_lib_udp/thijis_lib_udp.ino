#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <WiFi.h>
#include <WiFiUdp.h>

#define lidarDebugSerial Serial

#include "thijs_rplidar.h"

TaskHandle_t Task1;
TaskHandle_t Task2;

byte buffer2[360*4];    // Buffer to hold data to be sent


const int UDP_PORT_1= 8888;
const char* ssid = "Oneplus";  // Replace with your WiFi SSID
const char* password = "hello@123";  // Replace with your WiFi password

const char* udpAddress = "192.168.113.67"; // Replace with your laptop's IP address
WiFiUDP udp1;

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
uint32_t debugPrintTimer;
const uint32_t dubugPrintInterval = 5000; // micros between prints

void dataHandler(RPlidar* lidarPtr, uint16_t dist, uint16_t angle_q6, uint8_t newRotFlag, int8_t quality) {
  float distFloat = dist; // unit is mm directly
  float angleDegreesFloat = angle_q6 * 0.015625; // angle comes in 'q6' format, so divide by (1<<6)=64 (or multiply by 1/64) (or bitshift to the right by 6) to get angle in degrees

  if((micros()-debugPrintTimer) >= dubugPrintInterval) {  // (debugPrintCounter >= (lidarPtr->lidarSerial.available())) {  // dynamic?
    debugPrintTimer = micros();

    lidar_scan[int(angleDegreesFloat)]=distFloat;
    Serial.println(distFloat);
  }
}

void setup() {
  Serial.begin(115200);

  Serial.println("Esp started");
  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }

  Serial.print("WiFi connected with IP: ");

  udp1.begin(UDP_PORT_1);

  motorHandler.init();

  lidar.init(16, 17);
  lidar.postParseCallback = dataHandler; // set dat handler function

  lidar.printLidarInfo();
  Serial.println();

  while(!lidar.connectionCheck()) { Serial.println("connectionCheck() failed"); }

  delay(10);
  motorHandler.setPWM(200);

  bool startSuccess = lidar.startExpressScan(EXPRESS_SCAN_WORKING_MODE_BOOST);

  xTaskCreatePinnedToCore(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 0 */                  
  delay(500); 

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
                    Task2code,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 
}

void Task1code( void * pvParameters ){


  for(;;){
    if(keepSpinning) {
    uint32_t extraSpeedTimer = micros();
    int8_t datapointsProcessed = lidar.handleData(false, false); // read lidar data and send it to the callback function. Parameters are: (includeInvalidMeasurements, waitForChecksum)

    if(datapointsProcessed < 0) { keepSpinning = false; lidar.stopScan(); } // handleData() returns -1 if it encounters an error
  } else {
    motorHandler.setPWM(0);
  }
      
  } 
}


void Task2code( void * pvParameters ){

  for(;;){

  int arrayLength2 = sizeof(lidar_scan) / sizeof(lidar_scan[0]);
  memcpy(buffer2, lidar_scan, arrayLength2 * sizeof(float));
  udp1.beginPacket(udpAddress, UDP_PORT_1); // Change "destination_ip_address" to the IP address of the destination
  udp1.write(buffer2, arrayLength2 * sizeof(float));
  udp1.endPacket();

  }
}
 

void loop() {

  
  
}
