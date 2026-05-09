Bluetooth Controlled Autonomous Robot with Obstacle Avoidance

Overview

This project presents an Arduino-based autonomous robot that supports three operational modes: Bluetooth manual control, obstacle avoidance, and line following. The robot is built using Arduino Uno R3 and integrates multiple sensors and modules to perform intelligent movement and navigation.

Features

- Bluetooth manual control using an Android phone
- Autonomous obstacle avoidance using an ultrasonic sensor
- Line following using IR sensors
- OLED display showing face animations
- Passive buzzer alert for warnings
- Rear safety sensor
- Multiple operation modes (Manual / Autonomous / Line Following)

Hardware Components

- Arduino Uno R3
- HC-05 Bluetooth Module
- HC-SR04 Ultrasonic Sensor
- IR Sensors (for line following)
- L298N Motor Driver
- SG90 Servo Motor
- OLED Display
- Passive Buzzer
- Li-ion Battery Pack

Working Principle

The robot works in three different modes:

1. Bluetooth Mode
   The robot receives commands from a smartphone through the HC-05 Bluetooth module. Commands such as forward, backward, left, and right control the movement of the robot.

2. Obstacle Avoidance Mode
   The ultrasonic sensor mounted on a servo motor scans the environment. If an obstacle is detected within a certain distance, the robot stops, reverses, and turns to avoid collision.

3. Line Following Mode
   Two IR sensors detect a black line on the ground. The robot automatically adjusts its movement to stay on the line.

Project Structure

code/ – Arduino firmware (.ino file)
circuit-diagram/ – Circuit diagram of the robot
images/ – Robot and project images
report/ – Project report document
docs/ – Presentation slides

Skip to content
Akash-chakrabartty
AI-robotics
Repository navigation
Code
Issues
Pull requests
Agents
Actions
Projects
Wiki
Security and quality
Insights
Settings
Commit 807c309
Akash-chakrabartty
Akash-chakrabartty
authored
on Apr 4
Verified
edit
Removed team members section from the README.
main
1 parent 
8c3e708
 commit 
807c309
1 file changed

-6
Lines changed: 0 additions & 6 deletions
File tree
Filter files…
README.md
Search within code
 
‎README.md‎
-6
Lines changed: 0 additions & 6 deletions

Team Members

- Akash Chakrabartty
- Md Sarwar Islam
- MD Sajid Tousif
- Bikram Sardar
- Bimochan Tiwari

Future Improvements

- Mobile app interface for better control
- Camera-based navigation
- AI based object detection
- Improved battery monitoring system

License

This project is developed for academic purposes.
