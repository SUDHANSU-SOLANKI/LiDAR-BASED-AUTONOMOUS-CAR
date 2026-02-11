==> 🚗 LiDAR-Based Self-Driving Car (ROS 2)

Final Year Project – Autonomous Mobile Robot using LiDAR, ROS 2, SLAM, and Navigation Stack.


## 📖 Overview

This project focuses on building a small-scale autonomous vehicle using:

- LiDAR for environment perception
- ROS 2 (Humble) 
- SLAM for mapping
- Nav2 for autonomous navigation
- Differential drive control with encoder feedback

The robot is capable of:
- Mapping unknown environments
- Localizing itself
- Planning paths
- Avoiding obstacles
- Autonomous navigation to goal points

## 🛠️ Hardware Used

- Raspberry Pi 4
- 12V High Torque DC Motors with Quadrature Encoders
- Motor Driver (L298N )
- LiDAR Sensor (RPLIDAR C1)
- Power Bank / Battery Pack
- Custom Wooden / Aluminum Chassis

## 💻 Software Stack

- Ubuntu 22.04
- ROS 2 Humble
- SLAM Toolbox
- Nav2
- RViz2
- Gazebo (for simulation)



---

## 🚀 Setup Instructions

###  Install ROS 2 Humble
Follow the official ROS 2 installation guide for Ubuntu 22.04.

###  Clone the Repository
   --> git clone https://github.com/SUDHANSU-SOLANKI/LiDAR-BASED-AUTONOMOUS-CAR
       cd LiDAR-BASED-AUTONOMOUS-CAR


###  Build Workspace
  --> colcon build 
      source install/setup.bash

###  Run the launch file for simulation
  --> ros2 launch my_robot_bringup my_robot_gazebo.launch.xml

### Run SLAM (slam_toolbox)
  --> ros2 launch slam_toolbox online_async_launch.py params_file:=./src/my_robot_bringup/config/mapper_params_online_async.yaml use_sim_time:=true

### Run Navigation stack (nav2_stack)
  --> ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true


---

## 🎯 Features

- Real-time LiDAR mapping
- Autonomous navigation
- Obstacle avoidance
- Encoder-based odometry
- Differential drive control
- Simulation + Hardware support

## 👨‍💻 Author

Sudhansu Solanki  
Final Year Engineering Student  
Robotics & Autonomous Systems Enthusiast

## 📜 License

This project is for academic and research purposes.
