# 1_ROS_based_SIHO

**Libraries Required for IMU library**
* Install library Bolder Flight Systems MPU9250 from Arduino IDE
* Install library Bolder Flight Systems Unit Conversions from Arduino IDE
* Install library Bolder Flight Systems Eigen from Arduino IDE

Reference GitHub repo https://github.com/bolderflight/invensense-imu

**Instructions for Madgwick filter**
* Install library Madgwick by Arduino from Arduino IDE
* Go to Arduino libraries from documents (in windows)/ from Home (in linux) and open the Magwick library folder
* Go inside the src folder and open MadgwickAHRS and change from #define sampleFreqDef   512.0f to #define sampleFreqDef   100.0f
* This is needed because the IMU update rate is 100 Hz, but the Madgwick filter default rate is 512 Hz.
