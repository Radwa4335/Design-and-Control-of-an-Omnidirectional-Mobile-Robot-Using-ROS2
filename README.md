# Design-and-Control-of-an-Omnidirectional-Mobile-Robot-Using-ROS2

## Overview
This project focuses on the design, simulation, and control of an Omnidirectional Mobile Robot using ROS2 and Arduino. The robot utilizes Mecanum wheels, allowing for smooth omnidirectional movement in all directions.

The system integrates ROS2 for communication, Gazebo for simulation, and real-time joystick control, making it a versatile and scalable robotic solution.

## Features
- Omnidirectional Motion with Mecanum Wheels: Enables movement in any direction with independent wheel control.
- ROS2-Based Modular Architecture: Provides efficient, scalable robot control and simulation.
- Joystick-Based Remote Control: Allows real-time manual navigation via a joystick.
- Gazebo Simulation: Enables testing in a realistic virtual environment.
- Real-Time Motor Speed Calculation: Uses kinematic equations to adjust wheel speeds dynamically.
- Arduino-Based Motor Control: Receives ROS2 commands and controls motor speeds accordingly.

## System Components
Hardware
- Microcontroller & Motor Control:
  - Arduino UNO – Processes motor speed commands and communicates with ROS2.
  - Motor Drivers (L298) – Controls the motors based on ROS2 commands. 
  - Mecanum Wheels (4x) – Provides omnidirectional movement capabilities.

- Power Supply:
  - Lithium Battery Pack – Powers motors and electronics.

- Motion & Steering:
  - Timing Belts & Pulleys – Ensures efficient power transmission.

Software
- Python-Based ROS2 Nodes:
  - Publisher-Subscriber Node: Subscribes to /cmd_vel, computes wheel speeds, and publishes motor commands.
  - Serial Communication: Interfaces with Arduino via UART serial communication.

- Arduino Firmware:
  - Listens to ROS2 messages via Serial Communication.
  - Converts speed commands to PWM signals for precise motor control.

- Gazebo Simulation:
  - Provides a virtual testing environment for the robot.
  - Uses URDF (Unified Robot Description Format) for accurate simulation.


 ## Installation & Setup
1. **Clone the repository**:
   ```sh
   git clone https://github.com/yourusername/omnidirectional-robot-ros2.git
   cd omnidirectional-robot-ros2
   ```
2. Build the ROS2 Workspace
   - Install dependencies:
     ```sh
     colcon build
     source install/setup.bash
     ```
4. Launch Gazebo Simulation
     ```sh
   ros2 launch omnidirectional_robot gazebo.launch.py
   ```
5. Enable Joystick Control
   ```sh
   ros2 run teleop_twist_joy teleop_node
   ```
6. Flash Arduino Firmware
   - Compile and upload the firmware using Arduino IDE or PlatformIO.
--------------------------------------------------------------------------------------------------------------------------
📌 For more details, please visit the Project Report (PDF) and Project Presentation (PPT) available in the repository.
