Control software
====

This directory contains code for control software which is used by the vehicle to participate in the competition, it contains a mix of a self made LIDAR library for detection of objects using the LDROBOT LD19 lidar and the current code for the robot in main.ino

The code is mostly written in c++ and handles the PWM output for servos, engines, and inputs such as the ultrasonic sensors, the color sensors and the gyroscope which is an essential part of the code as it used a PID to autocorrect steep angles and keep itself straight.
