# quadrotor_path_planning

Path planning for quadrotor UAVs using the **Hybrid A*** algorithm.  
This repo implements global path planning in a 3D/2D discretized space, suitable for use in simulation or real‑time systems. The planned paths are intended to be consumed by a tracking controller (PID / MPC) in downstream repositories.

---

## Overview

Features include:

- Hybrid A* global planner capable of navigating in grid‐based 2D/3D spaces with obstacles.  
- Support for varying heuristics and grid resolutions.  
- Configuration of quadrotor dimensions (e.g. size, turning radius).  
- Generation of paths saved in the `paths/` directory to be used later by path tracking modules.

---

## Repository structure

```
quadrotor_path_planning/
├─ paths/                     # Saved path files (e.g. .txt or .csv) from planner outputs
├─ planner/                   # Hybrid A* implementation & helper modules
├─ utils/                     # Utility functions: collision checking, grid maps, heuristics
├─ config/                    # Config files (grid resolution, obstacle maps, start / goal positions)
├─ launch/ (if applicable)    # Launch or script runners to produce paths
└─ README.md
```

---

## Requirements

- Python 3.8+  
- Packages: `numpy`, `scipy`, `matplotlib`  
- If using ROS: `rospy`, `ros‑launch` (if planner is wrapped in ROS)  
- Obstacle maps or environment descriptions (e.g. occupancy grid)  

---

## Usage

- Adjust configuration in `config/` (start, goal, grid resolution, map) and obstacle definitions.  
- Run planner via a command in the `planner/` module, e.g.:

  ```python
  # in planner/main_planner.py
  plan = hybrid_astar_planner(grid_map, start, goal, heuristic, resolution)
  plan_path(plan, output_path_file)
  ```

- Output path files are stored in `paths/`. These contain waypoints (x,y,z) or (x,y).

---

## Configuration variables

These are set inside code / configuration files:

- Planner: `GRID_RESOLUTION`, `TURNING_RADIUS`, `HEURISTIC_WEIGHT`  
- Map: path to obstacle map, obstacle inflation margin  
- Start / Goal: coordinates in map frame  
- Output path filename  

---

## How it connects to tracking

The generated paths (in `paths/`) are input to the **quadrotor_path_tracking** repo, where tracking controllers (PID or MPC) follow these waypoints.

---

## Tips & Troubleshooting

- For dense obstacle maps, increase resolution but expect longer planning time.  
- Heuristic weight: too high → possibly non‑optimal path; too low → long runtimes.  
- Validate that start/goal are in free space.  
- Visualize the planned path (with `matplotlib`) before using with tracking controllers.  

---

## License

MIT License (see `LICENSE`).
