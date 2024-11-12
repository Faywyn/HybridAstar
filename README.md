# Hybrid A* Pathfinding with Reeds-Shepp Paths

This project implements a **Hybrid A*** pathfinding algorithm with **Reeds-Shepp** paths for planning
smooth, collision-free paths in a 2D environment. The algorithm is designed for robotic path
planning applications where a non-holonomic robot (one that cannot move directly sideways)
navigates from a start to a goal position, avoiding obstacles along the way.

## Features
- **Hybrid A*** pathfinding algorithm for non-holonomic path planning.
- **Reeds-Shepp** paths for smooth turning and backward movement, allowing the planner
to generate realistic paths.
- **Collision detection** to ensure paths avoid obstacles.
- **SFML** graphics integration for visualization of the map, the search process, and the final path.
- **Precomputed heuristic** for faster convergence, enhancing the performance of the pathfinding algorithm. 

## Dependencies
- **SFML**: For graphical rendering of the environment and visualization of the pathfinding process.
- **OMPL**: For computing Reeds-Shepp paths and collision checking, (see [here](https://ompl.kavrakilab.org/download.html) to install).
- **Boost**: Required by OMPL for serialization and other utilities.

> [!WARNING]  
> This code is macOS (silicon) compatible. If you are using a different OS, you may need to modify the build
script to link the correct libraries.

## Usage
1. Clone the repository:
```bash
git clone https://github.com/Faywyn/HybridAstar.git
```

2. Build the project:
```bash ./build.sh release```

3. Run the executable:
```bash ./build.sh run```
or
```bash ./build/bin/HybridAstar```

4. Press SPACE to restart the search process with a new random start and goal position.

## Screenshots
