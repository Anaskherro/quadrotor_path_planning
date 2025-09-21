# quadrotor_path_planning

Path planning for quadrotor UAVs using the **Hybrid A*** algorithm in Gazebo simulation.  
This repo generates collision-free paths in a labyrinth-like environment, later used by the tracking controllers.

---

## Step-by-step setup

### 1. Prepare environment
- Install **Ubuntu 20.04** and update packages:
  ```bash
  sudo apt update && sudo apt upgrade
  ```
- Install **ROS Noetic** with Gazebo + Rviz:
  ```bash
  sudo apt install ros-noetic-desktop-full
  ```
- Initialize rosdep:
  ```bash
  sudo apt install python3-rosdep
  sudo rosdep init
  rosdep update
  ```

### 2. Simulation dependencies
- Create a catkin workspace and install `catkin_tools`:
  ```bash
  mkdir -p ~/catkin_ws/src
  cd ~/catkin_ws
  catkin init
  ```
- Clone quadrotor simulation models (e.g. iq_sim) for drone + sensors:
  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/Intelligent-Quads/iq_sim.git
  ```

### 3. MAVROS + autopilot integration
- Install MAVROS:
  ```bash
  sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
  ```
- Clone ArduPilot SITL:
  ```bash
  cd ~
  git clone https://github.com/ArduPilot/ardupilot.git
  ```

### 4. Launch simulation
- Build workspace:
  ```bash
  cd ~/catkin_ws && catkin build
  source ~/catkin_ws/devel/setup.bash
  ```
- Launch labyrinth world:
  ```bash
  roslaunch quadrotor_path_planning lidar.launch
  ```

---

## 5. Run Hybrid A* Planner
- Configure start, goal, and map in `config/`.
- Run planner script:
  ```bash
  python3 planner/planning.py
  ```
- The optimal path is saved in:
  ```
  paths/path.txt
  ```

---

## Output
- Planned trajectory visualized in Rviz (blue line).  
- Path file exported to be consumed by **quadrotor_path_tracking**.

---

## Repo structure

```
quadrotor_path_planning/
├─ paths/            # Output trajectories
├─ planner/          # Hybrid A* implementation
├─ utils/            # Collision checking, grid map tools
├─ config/           # Environment & quadrotor configs
├─ launch/           # Launch files for Gazebo
└─ README.md
```

---

## License
MIT (see LICENSE)
