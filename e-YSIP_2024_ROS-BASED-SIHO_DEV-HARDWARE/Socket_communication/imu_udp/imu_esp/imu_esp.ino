#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <WiFi.h>
#include <WiFiUdp.h>


#include "mpu9250.h"
/* Mpu9250 object */
bfs::Mpu9250 imu;



byte buffer1[9*4];  
byte buffer2[360*4];    // Buffer to hold data to be sent


const int UDP_PORT_1= 4210;
const int UDP_PORT_2=8888;
const char* ssid = "Oneplus";  // Replace with your WiFi SSID
const char* password = "hello@123";  // Replace with your WiFi password

const char* udpAddress = "192.168.113.67"; // Replace with your laptop's IP address


WiFiUDP udp1;
WiFiUDP udp2;

void setup() {
  Serial.begin(115200);
  Serial.println("Esp started");
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  
  WiFi.mode(WIFI_STA);
  Serial.println("Wifi going to start");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi...");
  }

  Serial.print("WiFi connected with IP: ");

  udp1.begin(UDP_PORT_1);
  udp2.begin(UDP_PORT_2);

  while(!Serial) {}
  /* Start the I2C bus */
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
}

void loop() {
  // Generate or read array data
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

  float dataArray2[360] = {0.0};
  int arrayLength2 = sizeof(dataArray2) / sizeof(dataArray2[0]);
  memcpy(buffer2, dataArray2, arrayLength2 * sizeof(float));

  

  udp2.beginPacket(udpAddress, UDP_PORT_2); // Change "destination_ip_address" to the IP address of the destination
  udp2.write(buffer2, arrayLength2 * sizeof(float));
  udp2.endPacket();

  // Serial.println(xPortGetCoreID());

  // delay(500);
}
