# Robolegs

At the beginning of the hackathon we had an idea to turn the couple of myorobotic motors, its control electronics, support structure and microcontrollers into a robot which can stand and stabilize itself.

The first challenge was getting readings from the joints angular sensors with CAN communication protocol. The Genuino board which we planned to use had incompatible pinout with the CAN shield. After making it work we connected the Genuino bord to the PC and transmitted the data via a ROS node written in python. Then we used an existing GUI-control program of the motors as a baseline for our control and extended it with the PID control which stabilizes the legs using the information from the joints angle sensors and spring displacement sensors available on the motors.

The main challenge was to understand the sensors, its calibration and implement an efficient PID control.

At the very end of the hackathon our robot can already stabilize itself and counteract small disturtions.

We learned about myorobotics, how to exchange tendons after failed attempts as well as about robot control algorithms.

Here is a quick pick on the mechanism being control in a position where ith out control was unstable: https://youtu.be/iNIt2GMIuko

And more media here: https://devpost.com/software/robolegs
