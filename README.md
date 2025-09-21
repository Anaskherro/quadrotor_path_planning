# quadrotor_path_planning

Path planning for quadrotor UAVs using the **Hybrid A*** algorithm in Gazebo simulation.  
This repo converts a **PGM occupancy grid** (derived from a 3D world model) into an optimal path file (`path.txt`) to be used by the tracking controllers.

---

## Step-by-step workflow

### 1) Setup
- Python ≥ 3.8  
- Install dependencies:
  ```bash
  pip install numpy scipy matplotlib opencv-python
  ```
- Clone the repo (with submodules for map/visualization tools):
  ```bash
  git clone --recurse-submodules https://github.com/Anaskherro/quadrotor_path_planning.git
  cd quadrotor_path_planning
  ```

### 2) Convert 3D world → 2D occupancy grid (PGM)
The planner works on a **2D occupancy grid**:
- Obstacles = **0 (black)**  
- Free = **255 (white)**  

Steps:
1. Export a top-down raster view of your 3D labyrinth or world.  
2. Threshold to binary (0/255).  
3. Save as `map.pgm` with resolution + origin.  
4. Inflate obstacles according to quadrotor footprint.

Place it in:
```
maps/map.pgm
```

### 3) Configure planner
In `config/` or script headers set:
- `MAP_PGM`: path to occupancy grid  
- `RESOLUTION`: meters/pixel  
- `ORIGIN`: map origin (x0, y0)  
- `ROBOT_SIZE_M`: e.g. 0.72 m square  
- `START_M`: start (x,y)  
- `GOAL_M`: goal (x,y)  
- (Optional) `HEURISTIC_WEIGHT`, `GRID_RES`, `TURNING_RADIUS`

### 4) Run Hybrid A*
Generate optimal path:
```bash
python3 planner/planning.py
```
Outputs:
```
paths/path.txt   # waypoints (x,y) in meters
```

### 5) Visualize
Use provided visualization utilities to display path.  
Expected: a **blue line** path through the maze.

---

## Repository structure

```
quadrotor_path_planning/
├─ maps/             # Input occupancy grids (.pgm)
├─ paths/            # Output trajectories (path.txt)
├─ planner/          # Hybrid A* algorithm
├─ utils/            # Collision checking, map handling
├─ config/           # Config params (start, goal, resolution, etc.)
├─ launch/           # Optional launch files
└─ README.md
```

---

## License
MIT (see LICENSE)
