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
  while (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
   
  }
  /* Set the sample rate divider */
  if (!imu.ConfigSrd(9)) {
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

float mag_x;
float mag_y;
float mag_z;


float thetaM;
float phiM;


float theta=0.0 ;
float phi=0.0;

float Xm;
float Ym;
float psi;



void loop() {
  curr_time=millis();
  // put your main code here, to run repeatedly:
  if (imu.Read()) {
  acc_x = imu.accel_x_mps2();
  acc_y = imu.accel_y_mps2();
  acc_z = imu.accel_z_mps2();

  gyr_x=imu.gyro_x_radps();
  gyr_y=imu.gyro_y_radps();
  gyr_z=imu.gyro_z_radps();

  mag_x=imu.mag_x_ut();
  mag_y=imu.mag_y_ut();
  mag_z=imu.mag_z_ut();

  
  phiM=(atan2 (acc_y ,( sqrt ((acc_x * acc_x) + (acc_z * acc_z)))));
  thetaM=(atan2(-acc_x ,( sqrt((acc_y * acc_y) + (acc_z * acc_z)))));


  dt = (millis()-millisOld)/1000.0;
  millisOld=millis();

  theta=(theta- gyr_y*dt)*0.95 + 0.05*thetaM;
  phi=(phi - gyr_x*dt)*0.95 +0.05*phiM;

  Xm=mag_x*cos(theta)-mag_y*sin(phi)*sin(theta)+mag_z*cos(phi)*sin(theta);
  Ym=mag_y*cos(phi)+mag_z*sin(phi);
  // Xm=mag_x;
  // Ym=mag_y;
  psi=(psi - gyr_z*dt)*0.95+atan2(Ym,Xm)*0.05;
  // psi=(psi - gyr_z*dt);
  // psi=atan2(Ym,Xm)
  
  }

  if(curr_time-prev_time>200){
  Serial.print(-90);
  Serial.print(",");
  Serial.print(degrees(theta));
  Serial.print(",");
  Serial.print(degrees(phi));
  Serial.print(",");
  Serial.print(degrees(psi));
  Serial.print(",");
  Serial.println(90);

  
  prev_time=millis();
  }

}
