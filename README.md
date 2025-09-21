# quadrotor_path_planning

Path planning for quadrotor UAVs using the **A* Hybrid algorithm** in a simulated Gazebo environment.  
This repository sets up a 3D labyrinth world, models a quadrotor (3DR Iris), and computes collision-free trajectories from start to goal.

---

## Overview

The project demonstrates how to:
- Install and configure ROS Noetic, Gazebo, and MAVROS.
- Simulate a quadrotor drone (Iris) inside a 3D environment with obstacles.
- Implement **A* Hybrid path planning** for global trajectory generation.
- Export an optimal trajectory as reference input for tracking controllers.

---

## Features

- **Simulation setup** with ROS + Gazebo + MAVROS.  
- **Quadrotor model**: 3DR Iris with sensors (IMU, GPS, barometer).  
- **A* Hybrid planner** in 2D/3D grid environments.  
- **Labyrinth environment** (6 × 6 × 6 m).  
- **Trajectory export** to `path.txt` for controllers.  

---

## Repository structure

```
quadrotor_path_planning/
├─ worlds/                 # Gazebo labyrinth world (3D obstacles)
├─ scripts/                # Python/C++ planning scripts
├─ launch/                 # ROS launch files
├─ config/                 # Parameters for planner, robot dimensions
└─ README.md
```

---

## Setup

1. Install **Ubuntu 20.04** and **ROS Noetic**.  
2. Install dependencies:
   ```bash
   sudo apt install ros-noetic-desktop-full ros-noetic-mavros ros-noetic-mavros-extras
   ```
3. Clone this repo inside your catkin workspace:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/Anaskherro/quadrotor_path_planning.git
   cd ~/catkin_ws && catkin build
   ```
4. Source workspace:
   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```

---

## Usage

- Launch Gazebo with the labyrinth world:
  ```bash
  roslaunch quadrotor_path_planning lidar.launch
  ```
- Run planner:
  ```bash
  rosrun quadrotor_path_planning planning.py
  ```
- Output trajectory is saved in:
  ```
  path.txt
  ```

---

## References

- A* Hybrid algorithm for UAV navigation.  
- MAVROS + Gazebo integration.  
- [ArduPilot](https://github.com/ArduPilot/ardupilot) SITL for autopilot integration.

---

## License

MIT (see LICENSE).
