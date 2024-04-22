# mapping-robot
**Self-Driving Robot with Obstacle Avoidance**
This project aimed to develop a two-wheeled, self-driving robot capable of navigating its a maze using ultrasonic and infrared sensors. The robot was designed and prototyped using an Arduino Uno microcontroller programmed in the Arduino IDE. Group members include Trevor Gibb and Adam Foster. Team Leader: Trevor Gibb

**Key Features:**
Obstacle detection and avoidance using ultrasonic and infrared sensors.
Autonomous navigation through pathfinding algorithms (development not fully implemented due to time constraints).

**Hardware**
* Arduino Uno microcontroller
* Gyroscope & Accelerometer (MPU-6050)
* Motor driver (TB6612FNG)
* Ultrasonic sensor & infrared sensors
* Chassis and wheels
* Power Supply
* Expansion board (connects parts and power supply together)
* RGB LEDs

**Software:**
* Arduino IDE for programming
* I2C communication for gyroscope integration

**Status:**
This project was a learning experience focused on embedded systems and sensor integration. While a fully functional, self-driving robot wasn't achieved due to time limitations, the project successfully explored:

* Ultrasonic and infrared sensor interfacing with Arduino Uno
* Basic obstacle detection logic
* I2C communication for gyroscope data acquisition
* Timers

**Key Learnings:**
* Progressed beyond writing code to understanding its execution on embedded systems
* Developed proficiency in obstacle detection and avoidance using ultrasonic and infrared sensors
* Gained hands-on experience interfacing an embedded system with various peripherals, including sensors and a gyroscope
* Developed a grasp of how embedded systems function by effectively integrating hardware and software components
* I2C communication protocol for sensor integration

**Etc.**
* Slides for presentation can be found here: https://1drv.ms/p/s!Av1Mr2INJ3D5iTg51S3YhZ4rr0eI?e=uo5Da9
* The navigation algorithm can be found in nav_algorithm.pdf. This algorithm is found in Project > Ultrasonic.h > obstacleAvoidanceMode().
* Some of this code was provided to us from the manufacturer by the robot. Due to limited time and team members, we didn't have time to do some things, like learn about the calculations associated with the gyroscope and sensors.
