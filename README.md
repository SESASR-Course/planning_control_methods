# Planning and Control Methods

Python examples of grid-based path planning and robot control methods for educational use.

## Content organization

### Path planners

[`Path_Planners/`](Path_Planners/) contains implementations built on a shared `Planner2D` class:

| Method | File | Description |
| --- | --- | --- |
| Breadth-first search (BFS) | [`bfs.py`](Path_Planners/bfs.py) | Explores reachable grid cells in queue order. |
| Dijkstra | [`dijkstra.py`](Path_Planners/dijkstra.py) | Searches using accumulated movement and map costs. |
| Greedy best-first search | [`greedy.py`](Path_Planners/greedy.py) | Selects cells using a heuristic estimate to the goal. |
| A* | [`a_star.py`](Path_Planners/a_star.py) | Combines accumulated cost and a goal heuristic. |

The [`planner_comparison.ipynb`](Path_Planners/planner_comparison.ipynb) notebook compares the four planners with four- and eight-connected movement on a sample map. [`utils.py`](Path_Planners/utils.py) provides map loading, movement definitions, and plotting helpers. Sample map images and occupancy-map data are in [`2D_maps/`](2D_maps/).

<p align="center">
  <img src=./readme_images/Movements4Connectivity_algorithms.gif alt="Grid Planners" width="400">
</p>


### Controllers

| Method | Location | Example |
| --- | --- | --- |
| Dynamic Window Approach (DWA) | [`Controllers/DWA/`](Controllers/DWA/) | [`controller_main.py`](Controllers/DWA/controller_main.py) simulates navigation toward a goal among point obstacles and plots the path and velocities. |
| Pure Pursuit | [`Controllers/PurePursuit/`](Controllers/PurePursuit/) | [`pure_pursuit_simulation.py`](Controllers/PurePursuit/pure_pursuit_simulation.py) simulates path following and saves an animation. |

Example outputs: [planner animations](Path_Planners/images/) · [Pure Pursuit animation](Controllers/PurePursuit/output_images/pure_pursuit.gif).

<p align="center">
  <img src=./readme_images/pure_pursuit.gif alt="Pure Pursuit" width="400">
</p>

## Getting started

With a working Python 3 environment, install the dependencies from the repository root:

```bash
python3 -m pip install -r requirements.txt
```

This installs the dependencies declared in [`requirements.txt`](requirements.txt).

To run the planner comparison, start Jupyter **from `Path_Planners/`** and open `planner_comparison.ipynb`. Its map path is relative to that directory:

```bash
cd Path_Planners
jupyter notebook planner_comparison.ipynb
```

Run the controller examples from the repository root:

```bash
python3 Controllers/DWA/controller_main.py
```
or
```bash
python3 Controllers/PurePursuit/pure_pursuit_simulation.py
```

## References
To be added.

## Acknowledgements
This work has been realized thanks to a joint effort by researchers at PIC4SeR Centre for Service Robotics at Politecnico di Torino (https://pic4ser.polito.it/). It supports the didactic activity of the course (Sensors, embedded systems and algorithms for Service Robotics) offered from 2023/24 in the M.Sc. in Mechatronic Engineering at Politecnico di Torino.

## License

See the [GNU General Public License v3.0](LICENSE).
