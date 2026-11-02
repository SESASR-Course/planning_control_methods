"""Run the main-branch algorithms and export compact data for the static site."""

import argparse
import ast
import json
import math
import struct
import sys
import types
import zlib
from collections import deque
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


def read_grayscale_png(path):
    """Decode the repository's 1-bit and 8-bit grayscale PNG maps."""
    payload = path.read_bytes()
    if not payload.startswith(b"\x89PNG\r\n\x1a\n"):
        raise ValueError(f"Not a PNG: {path}")
    chunks = []
    pos = 8
    while pos < len(payload):
        length = struct.unpack_from(">I", payload, pos)[0]
        kind = payload[pos + 4:pos + 8]
        body = payload[pos + 8:pos + 8 + length]
        chunks.append((kind, body))
        pos += length + 12
        if kind == b"IEND":
            break
    header = next(body for kind, body in chunks if kind == b"IHDR")
    width, height, depth, color, compression, filtering, interlace = struct.unpack(">IIBBBBB", header)
    if color != 0 or depth not in (1, 8) or compression or filtering or interlace:
        raise ValueError(f"Unsupported map PNG format: {path}")
    raw = zlib.decompress(b"".join(body for kind, body in chunks if kind == b"IDAT"))
    stride = (width * depth + 7) // 8
    previous = bytearray(stride)
    pixels = np.empty((height, width), dtype=np.uint8)
    offset = 0
    for row in range(height):
        filter_type = raw[offset]
        scan = bytearray(raw[offset + 1:offset + 1 + stride])
        offset += stride + 1
        for col in range(stride):
            left = scan[col - 1] if col else 0
            up = previous[col]
            upper_left = previous[col - 1] if col else 0
            if filter_type == 1:
                scan[col] = (scan[col] + left) & 255
            elif filter_type == 2:
                scan[col] = (scan[col] + up) & 255
            elif filter_type == 3:
                scan[col] = (scan[col] + ((left + up) // 2)) & 255
            elif filter_type == 4:
                prediction = left + up - upper_left
                distances = (abs(prediction - left), abs(prediction - up), abs(prediction - upper_left))
                predictor = (left, up, upper_left)[distances.index(min(distances))]
                scan[col] = (scan[col] + predictor) & 255
            elif filter_type != 0:
                raise ValueError(f"Unsupported PNG filter {filter_type}: {path}")
        if depth == 8:
            pixels[row] = np.frombuffer(scan, dtype=np.uint8)
        else:
            pixels[row] = np.unpackbits(np.frombuffer(scan, dtype=np.uint8))[:width] * 255
        previous = scan
    return pixels


def make_grid(path):
    image = read_grayscale_png(path)
    blocked = image < 128 if path.stem == "corridor" else image >= 128
    factor = 16 if path.stem == "corridor" else 3
    height, width = blocked.shape
    grid = np.zeros(((height + factor - 1) // factor, (width + factor - 1) // factor), dtype=int)
    for row in range(grid.shape[0]):
        for col in range(grid.shape[1]):
            grid[row, col] = int(blocked[row * factor:(row + 1) * factor,
                                         col * factor:(col + 1) * factor].any())
    return grid


def endpoints(grid):
    """Choose distant points in the largest four-connected free region."""
    height, width = grid.shape
    free = {(r, c) for r in range(height) for c in range(width) if grid[r, c] == 0}
    components = []
    while free:
        seed = min(free)
        component = {seed}
        queue = deque([seed])
        free.remove(seed)
        while queue:
            row, col = queue.popleft()
            for point in ((row + 1, col), (row - 1, col), (row, col + 1), (row, col - 1)):
                if point in free:
                    free.remove(point)
                    component.add(point)
                    queue.append(point)
        components.append(component)
    largest = max(components, key=len)

    def farthest(seed):
        queue = deque([seed])
        distance = {seed: 0}
        while queue:
            row, col = queue.popleft()
            for point in ((row + 1, col), (row - 1, col), (row, col + 1), (row, col - 1)):
                if point in largest and point not in distance:
                    distance[point] = distance[(row, col)] + 1
                    queue.append(point)
        return max(distance, key=lambda point: (distance[point], point))

    start = farthest(min(largest))
    return np.array(start), np.array(farthest(start))


def grid_data(source):
    utils = load_planner_utils(source)
    sys.path.insert(0, str(source / "Path_Planners"))
    from a_star import AStar
    from bfs import BFS
    from dijkstra import Dijkstra
    from greedy import Greedy

    maps = {}
    for image in sorted((source / "2D_maps").glob("*.png")):
        grid = make_grid(image)
        start, goal = endpoints(grid)
        results = {}
        for connectivity, movement in ((4, utils.Movements4Connectivity), (8, utils.Movements8Connectivity)):
            for label, cls in (("bfs", BFS), ("dijkstra", Dijkstra), ("greedy", Greedy), ("astar", AStar)):
                moves = movement()
                planner = cls(grid, moves)
                path = planner.plan(start, goal)
                results[f"{label}-{connectivity}"] = {
                    "path": [point.astype(int).tolist() for point in path],
                    "visited": [node.position.astype(int).tolist() for node in planner.came_from],
                    "cost": round(sum(moves.cost(a, b) for a, b in zip(path, path[1:])), 2),
                }
        maps[image.name] = {"grid": grid.tolist(), "costmap": Dijkstra(grid, utils.Movements4Connectivity()).costmap.tolist(),
                            "start": start.tolist(), "goal": goal.tolist(), "results": results}
    return {"default": "map1.png", "maps": maps}


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
            "candidates": np.round(candidates[:, :, :2], 3).tolist(), "chosen": chosen,
            "config": {"dt": controller.dt, "timeGranularity": 0.1,
                       "vSamples": controller.v_samples, "wSamples": controller.w_samples,
                       "goalDistTol": controller.goal_dist_tol, "collisionTol": controller.collision_tol,
                       "obstacleMaxDist": controller.obstacle_max_dist, "maxSteps": controller.max_num_steps,
                       "maxLinearAcc": controller.robot.max_linear_acc,
                       "maxAngAcc": controller.robot.max_ang_acc,
                       "maxLinVel": controller.robot.max_lin_vel, "minLinVel": controller.robot.min_lin_vel,
                       "maxAngVel": controller.robot.max_ang_vel, "minAngVel": controller.robot.min_ang_vel,
                       "radius": controller.robot.radius},
            "defaults": {"heading": controller.weight_angle, "velocity": controller.weight_vel,
                         "obstacle": controller.weight_obs, "simTime": 2.0}}


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
