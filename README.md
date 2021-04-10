# CPP Anti Collision

## About
This program provides an obstacle avoidance functionality to the DiAvEn Labfly drones with the use of four VL53L1X sensors that are connected to a Raspberry Pi 4.
The communication between the sensors and the Raspberry Pi 4 is done through I2C and the communication between the Raspberry Pi 4 and the Pixhawk Flight Controller is done through UART. To enable the simple obstacle avoidance in Ardupilot, MAVLink messages will be used.

## How to use
This program was coded specifically to be used on a Raspberry Pi 4. It might work on another Raspberry Pis with 40 GPIO Pins, but that capability has not been tested yet.
Before running the program, everything needs to be compiled and made first. In the folder `scripts`, there is a python script that generate a makefile based on all libraries in the program. The python script is named `pi_generate_makefile.py`. After generating the necessary makefile, the `pi_make.sh` shell script needs to be called to build the program. After the build, running the program is as easy as calling the `pi_run.sh` script.