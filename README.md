# mapping-robot
**Self-Driving Robot with Obstacle Avoidance**

This project aimed to develop a two-wheeled, self-driving robot capable of navigating its a maze using ultrasonic and infrared sensors. The robot was designed and prototyped using an Arduino Uno microcontroller programmed in the Arduino IDE.

**Key Features:**

Obstacle detection and avoidance using ultrasonic and infrared sensors.
Autonomous navigation through pathfinding algorithms (development not fully implemented due to time constraints).

** Hardware **

* Arduino Uno microcontroller
* Gyroscope & Accelerometer (MPU-6050)
* Motor driver (TB6612FNG)
* Ultrasonic sensor & infrared sensors
* Chassis and wheels
* Power Supply
* Expansion board (connects parts and power supply together)
* RGB LEDs


* Slides for presentation can be found here: https://1drv.ms/p/s!Av1Mr2INJ3D5iTg51S3YhZ4rr0eI?e=uo5Da9
* The navigation algorithm can be found in nav_algorithm.pdf. This algorithm is found in Project > Ultrasonic.h > obstacleAvoidanceMode().
* Some of this code was provided to us from the manufacturer by the robot. Due to limited time and team members, we didn't have time to do some things, like learn about the calculations associated with the gyroscope and sensors.
* Group members include Trevor Gibb, and Adam Foster. 
