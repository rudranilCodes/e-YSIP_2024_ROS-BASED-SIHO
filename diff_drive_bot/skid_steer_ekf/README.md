** This code models the kinematics of a skid-steer drive robot and designs an Extended Kalman Filter algorithm which uses wheel slip estimation for accurate modeling of the state-space. Wheel slip plays a critical role in kinematic and dynamic modeling of skid-steered mobile robots. The slip information provides a connection between the wheel rotation velocity and the linear motion of the robot platform. Understanding the slip information is important for robot localization applications, such as dead reckoning.

** All logical steps have been verified and the code has been based on the following research paper: https://ieeexplore.ieee.org/document/5175357

** However the process noise and measurement noise covariance matrices are to be tuned properly which could not be completed due to lack of time. 

** Rigorous testing and tuning methods may be used for the tuning of these matrices one of the most basic methods being Mahalanobis distance optimisation.