# CS183-Pathfinding-demonstrator
A visual pathfinding algorithm simulator built with Processing.  
Supports BFS, Dijkstra, and A* algorithms with weighted terrain and multi-agent pathfinding.

---

## Project Overview
This interactive visualization tool demonstrates three classic pathfinding algorithms in action:
- **Breadth-First Search (BFS)** – Unweighted, finds shortest path by exploring all neighbors equally
- **Dijkstra's Algorithm** – Weighted shortest path algorithm
- **A* Algorithm** – Heuristic-guided pathfinding with optimal efficiency

The project includes weighted terrain (normal/grass/desert) and multi-agent pathfinding modes, with real-time step-by-step visualization.

---

## Features
- Real-time visualization of open/closed lists and final paths
- Adjustable simulation speed (1x–20x, with doubled max speed in this version)
- Support for obstacles, grass, and desert terrain with different movement weights
- Multi-agent / multi-goal pathfinding
- Interactive grid editing (add/remove obstacles, start/end points, terrain)
- Statistics panel showing visited nodes, path length, and CPU cycles
- Algorithm comparison mode

---

## Version History

### Version 1 (Single-Path Mode)
- Core implementation of BFS, Dijkstra, and A*
- Single start/goal pathfinding
- Basic weighted terrain support
- Adjustable simulation speed

### Version 2 (Enhanced Multi-Path Mode)
- Multi-agent / multi-goal parallel pathfinding
- Improved speed control (doubled maximum speed, instant slider updates)
- Enhanced terrain drag-and-drop editing
- Automatic balancing of start/goal counts
- Detailed multi-path statistics and visualization

---

## How to Run
1. Open the project in the Processing IDE
2. Click "Run" to start the simulator
3. Use the tools to add start points, goals, obstacles, and terrain
4. Select an algorithm and click "Start" to begin the simulation
5. Adjust the speed slider to control the simulation rate

---

## Source Code
Full project source code (including Version 1 and Version 2) is available on GitHub:  
[Click to visit Version 1](https://github.com/xuan20070819/CS183-Pathfinding-demonstrator/blob/main/main_program_1.pde)
[Click to visit Version 2](https://github.com/xuan20070819/CS183-Pathfinding-demonstrator/blob/main/main_program_2.pde)

---

## Acknowledgments
- Inspired by classic pathfinding algorithm visualizations
- Built with Processing
