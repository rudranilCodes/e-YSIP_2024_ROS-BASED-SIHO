This folder contains a python script to emulate a quadrature encoder for simulation. The ticks are published as pulses onto the following topics:

/front_left_wheel/channel_a
/front_left_wheel/channel_b
/front_right_wheel/channel_a
/front_right_wheel/channel_b
/back_left_wheel/channel_a
/back_left_wheel/channel_b
/back_right_wheel/channel_a
/back_right_wheel/channel_b

To run the file use the following command:
$ ros2 run diff_drive_bot encoder_emulator_vel.py

The python script quad_odom.py may be used for calculation of raw odometry from the quadrature encoder pulses. The odometry has been published as enc_odom--->base_link.
To run this script use the following terminal command:

$ ros2 run diff_drive_bot quad_odom.py