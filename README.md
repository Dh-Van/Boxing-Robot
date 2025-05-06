# Boxing Bot: Vision-Guided Task-Space Planning on a 5-DOF Mobile Manipulator

This repository documents our final project for the *Fundamentals of Robotics (Spring 2025)* course at Olin College. We developed a vision-guided trajectory planner for a boxing robot using a HiWonder 5-DOF arm. The system detects an ARUCO marker using a webcam, estimates its 6-DOF pose, transforms this into the robot's base frame, generates task-space waypoints around the marker, and uses inverse kinematics to execute a basic boxing motion.

<p align="center">
  <img src="media/hiwonder.png" height="300" alt="Hiwonder Arm Setup">
</p>

## Project Video

🎥 [Click to view the demo video](https://www.youtube.com/watch?v=REPLACE_ME)

## Repository Overview

This repository contains:

- All Python code used to run our vision and motion pipeline on a Raspberry Pi 4B  
- Instructions to connect and deploy code on the HiWonder robot platform  
- Our technical report documenting system architecture, implementation, and insights  

## How to Run the System

### Prerequisites

- Raspberry Pi 4B with RasAdapter V3.6  
- SSH access to the Pi (use `ssh funrobot@funrobot#.local`)  
- A connected camera module on `/dev/video0`  
- ARUCO marker placed in view  

### 1. SSH into the Raspberry Pi

```bash
ssh funrobot@funrobot#.local
# Default password is: FunR0b0!
```

### 2. Create a Python virtual environment

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

### 3. Initialize pigpio daemon

```bash
sudo pigpiod
```

### 4. Run the system

```bash
sudo venv/bin/python main.py
```

> Make sure the `main.py` script points to the correct path for your Python virtual environment.

## System Summary

The system is composed of:

- **Vision Module**: Captures one frame, detects a 36h11 ARUCO marker, and computes the 5-DOF pose using camera intrinsics and distortion correction. Pose is transformed to robot base frame.  
- **Trajectory Planning**: Generates task-space waypoints around the marker pose.  
- **Execution Loop**: Uses an inverse kinematics solver to sequentially move the end-effector to each waypoint.  

<p align="center">
  <img src="media/jstick-manual-1.png" height="260">
  <img src="media/jstick-manual-2.png" height="260">
</p>

## Known Limitations

- The ARUCO marker was oversized, reducing precision.  
- System is limited to static, one-shot motion (not real-time responsive).  
- No drivetrain or mobile base integration implemented.  

## Future Work

- Replace marker with smaller fiducials for higher accuracy  
- Integrate dynamic camera updates for live trajectory correction  
- Add drivetrain control to enable mobile sparring behavior  

## Technical Report

📄 [Download Full Report (PDF)](media/FunRobo_Final.pdf)

## ROS Camera Node Troubleshooting

If you encounter errors accessing the camera, kill ROS processes:

```bash
rosnode kill -a
pkill -f roscore
pkill -f roslaunch
pkill -f rosmaster
pkill -f rosout
```
