#define lidarDebugSerial Serial
#include "thijs_rplidar.h"

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/laser_scan.h>

rcl_publisher_t publisher;
sensor_msgs__msg__LaserScan laser_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

float arr[360] = {0.0};

#define LED_PIN LED_BUILTIN

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

  char frame[12] = "laser_frame";

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &laser_msg, NULL));
  }
}


TaskHandle_t Task1;

float lidar_scan[360]={0.0};
struct lidarMotorHandler {
  const uint8_t pin;
  const uint32_t freq; 
  const uint8_t channel; 
  const bool activeHigh; 
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
  float angleDegreesFloat = angle_q6 * 0.015625; 

    laser_msg.ranges.data[int(angleDegreesFloat)] = distFloat/1000;
}

void Task1code( void * pvParameters ){
  Serial.println();

  motorHandler.init();

  lidar.init(16, 17);
  lidar.postParseCallback = dataHandler; 

  lidar.printLidarInfo();

  Serial.println();
  motorHandler.setPWM(255);
  while(!lidar.connectionCheck()) { Serial.println("connectionCheck() failed"); }

  delay(10);
  motorHandler.setPWM(255);
  //bool startSuccess = lidar.startStandardScan();
  //bool startSuccess = lidar.startExpressScan(EXPRESS_SCAN_WORKING_MODE_LEGACY);
  bool startSuccess = lidar.startExpressScan(EXPRESS_SCAN_WORKING_MODE_BOOST);

  for(;;){

  if(keepSpinning) {
    uint32_t extraSpeedTimer = micros();
    int8_t datapointsProcessed = lidar.handleData(false, false);

    if(datapointsProcessed < 0) { keepSpinning = false; lidar.stopScan(); }
  } else {
    motorHandler.setPWM(0);
  }

  }
}

void setup() {
  Serial.begin(115200);
  set_microros_wifi_transports("Aditya", "1010101010", "192.168.169.47", 8888);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  laser_msg.scan_time = 0.0;
  laser_msg.time_increment = 0.0;

    laser_msg.header.frame_id.data = frame;
    laser_msg.angle_max = 3.141592653589793*2;
    laser_msg.angle_increment = 3.141592653589793 / 180;
    // laser_msg.scan_time = 1.0;
    laser_msg.range_min = 0.5;
    laser_msg.range_max = 12.0;

  laser_msg.ranges.capacity = 360; 
  laser_msg.ranges.size = 360;
  laser_msg.ranges.data = (float*) malloc(laser_msg.ranges.capacity * sizeof(float));

  // laser_msg.ranges.data = arr;

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "scan_pub", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    "scan_pub"));

  // create timer,
  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  xTaskCreatePinnedToCore(
                    Task1code,
                    "Task1",
                    10000,
                    NULL,
                    1,
                    &Task1,
                    1);

}


void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}