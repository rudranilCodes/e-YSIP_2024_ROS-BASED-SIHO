#include "mpu9250.h"
#include <math.h>
#include <MadgwickAHRS.h>

Madgwick filter;


/* Mpu9250 object */
bfs::Mpu9250 imu;

float acc_x;
float acc_y;
float acc_z;

float gyr_x;
float gyr_y;
float gyr_z;

float mag_x;
float mag_y;
float mag_z;

unsigned long millisOld;
float roll, pitch, heading;

void setup() {
  Serial.begin(115200);
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

}

void loop() {

  

  if (imu.Read()) {



    acc_x = imu.accel_x_mps2();
    acc_y = -imu.accel_y_mps2();
    acc_z = -imu.accel_z_mps2();

    gyr_x=imu.gyro_x_radps();
    gyr_y=imu.gyro_y_radps();
    gyr_z=imu.gyro_z_radps();

    mag_x =imu.mag_x_ut();
    mag_y =imu.mag_y_ut();
    mag_z =imu.mag_z_ut();


    filter.updateIMU(gyr_x*57.2958,gyr_y*57.2958,gyr_z*57.2958,acc_x/9.81,acc_y/9.81,acc_z/9.81);

    // print the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();

  }

  if(millis()-millisOld>100){
  Serial.print("Orientation: ");
  Serial.print(heading-180.0);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.println(roll);

  
  
  millisOld=millis();
  }
  
}
