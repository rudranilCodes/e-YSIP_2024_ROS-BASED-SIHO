#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include "FreeRTOSConfig.h"


#include "mpu9250.h"

/* Mpu9250 object */
bfs::Mpu9250 imu;


// Task handles
TaskHandle_t Task1Handle;
TaskHandle_t Task2Handle;
TaskHandle_t Task3Handle;


byte buffer1[9*4];  
byte buffer2[360*4];    // Buffer to hold data to be sent
char incomingPacket[255]; 

const int UDP_PORT_1= 4210;
const int UDP_PORT_2=8888;
const int UDP_PORT_3=6666;


const char* ssid = "Oneplus";  // Replace with your WiFi SSID
const char* password = "hello@123";  // Replace with your WiFi password

const char* udpAddress = "192.168.113.67"; // Replace with your laptop's IP address


WiFiUDP udp1;
WiFiUDP udp2;
WiFiUDP udp3;


void setup() {
  Serial.begin(115200);
  // Serial.println();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }


  udp1.begin(UDP_PORT_1);
  udp2.begin(UDP_PORT_2);
  udp3.begin(UDP_PORT_3);

  while(!Serial) {}
  /* Start the I2C bus */
  Wire.begin();
  Wire.setClock(400000);
  /* I2C bus,  0x68 address */
  imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
  /* Initialize and configure IMU */
  while (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");

  }
  /* Set the sample rate divider */
  while (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");

  }

  xTaskCreatePinnedToCore(
    Task1,        // Task function
    "Task1",      // Name of the task
    1024*5,         // Stack size
    NULL,         // Parameter
    1,            // Priority
    &Task1Handle, // Task handle
    0             // Core
  );

  // Create Task 2
  xTaskCreatePinnedToCore(
    Task2,        // Task function
    "Task2",      // Name of the task
    2048*5,         // Stack size
    NULL,         // Parameter
    1,            // Priority
    &Task2Handle, // Task handle
    1             // Core
  );

   xTaskCreatePinnedToCore(
    Task3,        // Task function
    "Task3",      // Name of the task
    2048*2,         // Stack size
    NULL,         // Parameter
    1,            // Priority
    &Task3Handle, // Task handle
    1             // Core
  );

}
void Task1(void *pvParameters) {
  for(;;) {
  if (imu.Read()) {
  float dataArray[9] = {imu.accel_x_mps2() ,imu.accel_y_mps2() ,imu.accel_z_mps2() ,imu.gyro_x_radps() ,imu.gyro_y_radps(),imu.gyro_z_radps(),imu.mag_x_ut(),imu.mag_y_ut(),imu.mag_z_ut()};
  int arrayLength1 = sizeof(dataArray) / sizeof(dataArray[0]);
  // Pack array data into the buffer
  memcpy(buffer1, dataArray, arrayLength1 * sizeof(float));
  // Send data via UDP
  udp1.beginPacket(udpAddress, UDP_PORT_1); // Change "destination_ip_address" to the IP address of the destination
  udp1.write(buffer1, arrayLength1 * sizeof(float));
  udp1.endPacket();
  }

  }
}

// Task 2: Print run-time stats
void Task2(void *pvParameters) {
  for(;;) {
  float dataArray2[360] = {0.0};
  int arrayLength2 = sizeof(dataArray2) / sizeof(dataArray2[0]);
  memcpy(buffer2, dataArray2, arrayLength2 * sizeof(float));

  udp2.beginPacket(udpAddress, UDP_PORT_2); // Change "destination_ip_address" to the IP address of the destination
  udp2.write(buffer2, arrayLength2 * sizeof(float));
  udp2.endPacket();

  }
}

void Task3(void *pvParameters) {
  for(;;) {
    int packetSize = udp3.parsePacket();
    if (packetSize) {
    // Read the packet into the buffer
    int len = udp3.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;
    }
    Serial.printf("UDP packet contents: %s\n", incomingPacket);
  }
  delay(10);

  }
}

void loop() {
}
