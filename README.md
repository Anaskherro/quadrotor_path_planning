# Quadrotor Path Planning

## Overview

**quadrotor_path_planning** is a project focused on calculating optimal paths for quadrotors navigating dense environments. It uses a `.pgm` map generated from a Gazebo maze (see the `plywood_mazes` submodule) and employs the hybrid A* algorithm to determine the shortest route through complex obstacle fields.

---

## Features

- **Hybrid A\* Algorithm:** Efficient, grid-based path planning suitable for environments with obstacles.
- **PGM Map Integration:** Uses occupancy grid maps exported from Gazebo.
- **Dense Environment Navigation:** Designed for challenging, cluttered spaces.
- **Visualizations:** Plots generated paths and obstacles for easy interpretation.
- **Modular:** Easily adapt to new maps by replacing the `.pgm` input.

---

## Installation

1. **Clone the repository (with submodules!):**
   ```bash
   git clone --recurse-submodules https://github.com/Anaskherro/quadrotor_path_planning.git
   ```
2. **Install Python dependencies:**
   ```bash
   pip install matplotlib numpy
   ```
3. **(Optional) Setup your Gazebo environment**  
   Generate your maze and export the `.pgm` map using the tools in the `plywood_mazes` submodule.

---

## Usage

1. **Generate your `.pgm` map** from your custom maze in Gazebo and place it in the repository directory (e.g., `cropped_map_1.pgm`).

2. **Run the path planning script:**
   ```bash
   python scripts/path_planning/planning_a_hybrid.py
   ```

   This will:
   - Read your `.pgm` map
   - Compute the shortest path using the hybrid A* algorithm
   - Save the path to `path2.txt`
   - Display a plot of the environment and the planned path

3. **Customize start/end points and drone dimensions:**  
   Edit `scripts/path_planning/planning_a_hybrid.py` to set your desired parameters.

---

## Example

```python
# Example usage in the script
file_path = 'cropped_map_1.pgm'
start = (0, 0)
end = (70, 70)
drone_dimensions = (9, 9)
```

---

## Contributing

Contributions are welcome! Please open an issue or submit a pull request with improvements or bug fixes.  
See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines (create this file if needed).

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

## Contact & Support

Questions, suggestions, or need help?  
Open an issue or contact [Anaskherro](https://github.com/Anaskherro).

---

## Acknowledgements

- [Gazebo](http://gazebosim.org/)
- [Hybrid A\* Algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm)
