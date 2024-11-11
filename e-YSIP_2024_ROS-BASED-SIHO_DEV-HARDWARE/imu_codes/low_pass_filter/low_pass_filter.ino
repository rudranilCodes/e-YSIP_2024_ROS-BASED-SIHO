#include "mpu9250.h"
#include <math.h>

/* Mpu9250 object */
bfs::Mpu9250 imu;


float curr_time=millis();
float prev_time=millis();
float dt;
unsigned long millisOld;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
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
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configured SRD");
    while(1) {}
  }
  millisOld=millis();
}

float acc_x;
float acc_y;
float acc_z;

float gyr_x;
float gyr_y;
float gyr_z;

float theta;
float phi;

float thetaFold=0.0;
float thetaFNew=0.0;
float phiFold=0.0;
float phiFNew=0.0;

float thetaG=0;
float phiG=0;



void loop() {
  curr_time=millis();
  // put your main code here, to run repeatedly:
  if (imu.Read()) {
  acc_x = imu.accel_x_mps2();
  acc_y = imu.accel_y_mps2();
  acc_z = imu.accel_z_mps2();
  gyr_x=imu.gyro_x_radps();
  gyr_y=imu.gyro_y_radps();

  // theta =  atan2(acc_x, sqrt(acc_y*acc_y + acc_z*acc_z))*180 /PI;
  // phi   =  atan2(acc_y, sqrt(acc_x*acc_x + acc_z*acc_z))*180 /PI;

  // theta= atan2(acc_x,sqrt(acc_y*acc_y + acc_z*acc_z))*180.0/3.141592654;
  // phi = atan2(-acc_y , acc_z) * 180.0 / 3.141592654;
  theta=degrees(atan2 (acc_y ,( sqrt ((acc_x * acc_x) + (acc_z * acc_z)))));
  phi=degrees(atan2(-acc_x ,( sqrt((acc_y * acc_y) + (acc_z * acc_z)))));

  phiFNew =0.95*phiFold+0.05*phi;
  thetaFNew =0.95*thetaFold+0.05*theta;

  dt = (millis()-millisOld)/1000.0;
  millisOld=millis();
  // Serial.println(dt);

  thetaG=thetaG + 57.2958*gyr_x*dt;
  phiG=phiG - 57.2958*gyr_y*dt;

  thetaFold=thetaFNew;
  phiFold=phiFNew;



  if(curr_time-prev_time>1000){
  // Serial.print(acc_x);
  // Serial.print(", ");
  // Serial.print(acc_x);
  // Serial.print(", ");
  // Serial.print(acc_z);
  // Serial.println("");
  

  Serial.print(theta);
  Serial.print(",");

  // // Serial.print("Phi ");
  Serial.print(phi);
  Serial.print(",");

  Serial.print(-thetaG);
  // // Serial.print(thetaFNew);
  Serial.print(",");

  Serial.print(phiG);
  // // Serial.print(phiFNew);
  // // Serial.print(",");
  Serial.println("");

  prev_time=millis();
  }



}
