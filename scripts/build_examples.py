"""Run the main-branch algorithms and export compact data for the static site."""

import argparse
import ast
import json
import math
import sys
import types
from pathlib import Path

import numpy as np


def load_planner_utils(source):
    # The planner utility file also imports ROS and plotting packages. Its
    # algorithmic classes only need NumPy, so load those classes in isolation.
    tree = ast.parse((source / "Path_Planners" / "utils.py").read_text())
    names = {"Node", "Movements", "Movements4Connectivity", "Movements8Connectivity"}
    classes = [node for node in tree.body if isinstance(node, ast.ClassDef) and node.name in names]
    module = types.ModuleType("utils")
    module.__dict__["np"] = np
    exec(compile(ast.Module(body=classes, type_ignores=[]), "planner_utils", "exec"), module.__dict__)
    sys.modules["utils"] = module
    return module


def grid_data(source):
    utils = load_planner_utils(source)
    sys.path.insert(0, str(source / "Path_Planners"))
    from a_star import AStar
    from bfs import BFS
    from dijkstra import Dijkstra
    from greedy import Greedy

    grid = np.zeros((18, 24), dtype=int)
    grid[3:16, 8] = 1
    grid[3, 8:17] = 1
    grid[3:13, 16] = 1
    grid[12, 16:22] = 1
    grid[6:10, 3:6] = 1
    grid[14:17, 19:22] = 1
    grid[11, 8] = grid[3, 12] = grid[9, 16] = grid[12, 19] = 0
    start, goal = np.array([15, 2]), np.array([2, 22])
    results = {}
    for connectivity, movement in ((4, utils.Movements4Connectivity), (8, utils.Movements8Connectivity)):
        for label, cls in (("bfs", BFS), ("dijkstra", Dijkstra), ("greedy", Greedy), ("astar", AStar)):
            planner = cls(grid, movement())
            path = planner.plan(start, goal)
            results[f"{label}-{connectivity}"] = {
                "path": [point.astype(int).tolist() for point in path],
                "visited": [node.position.astype(int).tolist() for node in planner.came_from],
                "cost": round(sum(movement().cost(a, b) for a, b in zip(path, path[1:])), 2),
            }
    return {"grid": grid.tolist(), "start": start.tolist(), "goal": goal.tolist(), "results": results}


def clear_algorithm_modules():
    sys.modules.pop("utils", None)


def dwa_data(source):
    clear_algorithm_modules()
    sys.path.insert(0, str(source / "Controllers" / "DWA"))
    from dwa import DWA

    obstacles = np.array([[6, 8], [2, 5], [8.5, 5], [3, 3], [5, 6], [4.5, 1.5],
                          [5, 4], [6.5, 3.5], [6, 7], [3.5, 2.5], [7, 3], [3, 7.5],
                          [7.5, 5], [7, 6.5], [3.5, 7], [8.4, 8]])
    start = np.array([1.0, 2.0, 0.0])
    goal = np.array([8.5, 6.0])
    controller = DWA(dt=0.1, sim_time=2.0, time_granularity=0.1,
                     v_samples=10, w_samples=20, goal_dist_tol=0.2,
                     weight_angle=0.06, weight_vel=0.2, weight_obs=0.04,
                     obstacles_map=obstacles, init_pose=start.copy(),
                     max_linear_acc=0.5, max_ang_acc=math.pi,
                     max_lin_vel=0.5, min_lin_vel=0.0,
                     max_ang_vel=2.82, min_ang_vel=-2.82, radius=0.2)
    candidates, velocities = controller.get_trajectories(start)
    chosen = int(controller.evaluate_paths(candidates, velocities, goal, start, obstacles))
    success, trajectory = controller.go_to_pose(goal)
    return {"start": start.tolist(), "goal": goal.tolist(), "obstacles": obstacles.tolist(),
            "success": bool(success), "trajectory": np.round(trajectory, 3).tolist(),
            "candidates": np.round(candidates[:, ::3, :2], 3).tolist(), "chosen": chosen}


def pure_pursuit_data(source):
    clear_algorithm_modules()
    sys.path.insert(0, str(source / "Controllers" / "PurePursuit"))
    from pure_pursuit import PurePursuitController
    from utils import DifferentialDriveRobot, interpolate_waypoints, proportional_control

    waypoints = np.array([[0, 0], [2.5, 2.5], [0, 5], [5, 5],
                          [2.5, 2.5], [5, 0], [0, 0]], dtype=float)
    path = interpolate_waypoints(waypoints, resolution=0.01)
    robot = DifferentialDriveRobot(np.array([0.0, 0.0, 1.570]))
    controller = PurePursuitController(robot, path, 0, 0.5, 0.22)
    samples = []
    a = w = 0.0
    for i in range(1200):
        if i % 5 == 0:
            a = proportional_control(controller.target_velocity(), robot.v, kp=0.5)
            w = controller.angular_velocity()
        robot.update_state([a, w], 0.1)
        if i % 5 == 0:
            samples.append([round((i + 1) * 0.1, 2), *np.round(robot.pose, 3).tolist(),
                            round(float(robot.v), 3), int(controller.pind)])
    return {"waypoints": waypoints.tolist(), "path": np.round(path[::10], 3).tolist(),
            "lookahead_path": np.round(path, 3).tolist(), "samples": samples}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    source = args.source.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    for name, build in (("grid", grid_data), ("dwa", dwa_data), ("pure-pursuit", pure_pursuit_data)):
        data = build(source)
        (output / f"{name}.json").write_text(json.dumps(data, separators=(",", ":")))
        print(f"Generated {name}.json")


if __name__ == "__main__":
    main()
