import numpy as np
import math
from utils import Differential_drive_robot, residual, normalize_angle, normalize, sigmoid

# DWA
class DWA():
    def __init__(self,
                 dt = 0.1,
                 sim_time = 2.0,
                 time_granularity = 0.1,
                 v_samples = 20,
                 w_samples = 20,
                 goal_dist_tol = 0.3,
                 weight_angle = 0.04,
                 weight_vel = 0.2,
                 weight_obs = 0.1,
                 obstacles_fn = None,
                 obstacles_map = None, # A list of obstacles poses in the map # TO DO: accept a gridmap and use local map
                 **kwargs
                 ):

        # define robot kinematics model and simulation parameters
        self.dt = dt
        self.sim_step = math.ceil(sim_time / time_granularity)
        self.robot = Differential_drive_robot(**kwargs)

        # define number of velocity samples to simulate
        self.v_samples = v_samples
        self.w_samples = w_samples

        # define weight
        self.weight_angle = weight_angle
        self.weight_vel = weight_vel
        self.weight_obs = weight_obs

        # function to detect near obstacles, can be lidar-based on map-based
        self.detect_obstacles = obstacles_fn
        self.obstacle_map = obstacles_map

        # tolerances
        self.goal_dist_tol = goal_dist_tol
        self.obstacle_max_dist = 3 # like a local costmap size
        self.obst_tolerance = 0.75

        # max number of steps to reach goal and feedback rate
        self.max_num_steps = 300
        self.feedback_rate = 50

    def go_to_pose(self, goal_pose):
        """
        Assign a target goal to the DWA controller.
        Compute commands until goal is reached.
        Provide intermediate feedback on the navigation task.
        """
        if goal_pose is list:
            goal_pose = np.array(goal_pose)

        success = False
        dist_to_goal = np.linalg.norm(self.robot.pose[0:2] - goal_pose)

        print("Initial distance to goal: ", dist_to_goal)
        print("Initial Robot pose: ", self.robot.pose)

        steps = 1
        while steps <= self.max_num_steps:
            # 1. Check if Goal reached
            dist_to_goal = np.linalg.norm(self.robot.pose[0:2] - goal_pose)
            if dist_to_goal < self.goal_dist_tol:
                success = True
                print("Goal reached!")
                break

            # 2. Get new observations for obstacles is needed
            # no lidar used here
            obstacles = self.detect_obstacles(self.robot.pose, self.obstacle_map, self.obstacle_max_dist)

            # 3. Compute command for the robot with DWA controller
            u = self.compute_cmd(goal_pose, self.robot.pose, obstacles)

            # 4. Send the command / or update the robot pose
            pose = self.robot.update_state(u, self.dt)
            
            # 5. Provide intermediate task feedback
            if steps % self.feedback_rate == 0:
                dist_to_goal = np.linalg.norm(self.robot.pose[0:2] - goal_pose)
                print("Current distance to goal ", dist_to_goal, " at step ", steps)
                print("Current Robot pose: ", pose)

            steps += 1

        if steps > self.max_num_steps:
            print("Timeout! Goal not reached.")

        return success, self.robot.trajectory
    
    def compute_cmd(self, goal_pose, robot_state, obstacles):
        """
        Compute the next velocity command u=(v,w) according to the DWA algorithm.
        The velocity leading to the highest scored trajectory is selected.
        """
        # create path
        paths, velocities = self.get_trajectories(robot_state)

        # evaluate path
        opt_idx = self.evaluate_path(paths, velocities, goal_pose, robot_state, obstacles)
        u = velocities[opt_idx]
        return u

    def get_trajectories(self, robot_pose): 
        """
        Get all the reachable velocities inside the dynamic window
        Simulate all trajectory with velocity values u=(v,w)
        Return: 
        - simulated paths : np.ndarray,  shape: (n_paths, sim_step, state_dim)
        - velocities : np.ndarray, shape (n_paths, 2)
        """
        # calculate reachable range of velocity and angular velocity in the dynamic window
        min_lin_vel, max_lin_vel, min_ang_vel, max_ang_vel = self.compute_dynamic_window(self.robot.vel)
        
        v_values = np.linspace(min_lin_vel, max_lin_vel, self.v_samples)
        w_values = np.linspace(min_ang_vel, max_ang_vel, self.w_samples)

        # list of all paths and velocities
        n_paths = w_values.shape[0]*v_values.shape[0]
        sim_paths = np.zeros((n_paths, self.sim_step, robot_pose.shape[0]))
        velocities = np.zeros((n_paths, 2))

        # evaluate all velocities and angular velocities combinations
        i = 0
        path = np.zeros((self.sim_step, robot_pose.shape[0]))
        for w in w_values:
            for v in v_values:
                u = np.array([v,w])
                velocities[i] = u.copy()
                path = self.simulate_path(robot_pose, u)
                sim_paths[i] = path
                i += 1

        return sim_paths, velocities
    
    def simulate_path(self, pose, u):
        """
        Simulate trajectory at constant velocity u=(v,w)
        """
        sim_trajectory = np.zeros((self.sim_step, 3))
        sim_trajectory[0] = pose.copy()
    
        for i in range(1, self.sim_step):
            sim_trajectory[i, 0] = sim_trajectory[i-1, 0] + u[0] * np.cos(sim_trajectory[i-1, 2]) * self.dt
            sim_trajectory[i, 1] = sim_trajectory[i-1, 1] + u[0] * np.sin(sim_trajectory[i-1, 2]) * self.dt
            sim_trajectory[i, 2] = sim_trajectory[i-1, 2] + u[1] * self.dt

        return sim_trajectory

    def compute_dynamic_window(self, robot_vel): 
        """
        Calculate the dynamic window composed of reachable linear velocity and angular velocity according to robot's kinematic limits.
        """
        # linear velocity
        min_vel = robot_vel[0] - self.dt * self.robot.max_linear_acc
        max_vel = robot_vel[0] + self.dt * self.robot.max_linear_acc
        # minimum
        if min_vel < self.robot.min_lin_vel:
            min_vel = self.robot.min_lin_vel
        # maximum
        if max_vel > self.robot.max_lin_vel:
            max_vel = self.robot.max_lin_vel

        # angular velocity
        min_ang_vel = robot_vel[1] - self.dt * self.robot.max_ang_acc
        max_ang_vel = robot_vel[1] + self.dt * self.robot.max_ang_acc
        # minimum
        if min_ang_vel < self.robot.min_ang_vel:
            min_ang_vel = self.robot.min_ang_vel
        # maximum
        if max_ang_vel > self.robot.max_ang_vel:
            max_ang_vel = self.robot.max_ang_vel

        return min_vel, max_vel, min_ang_vel, max_ang_vel


    def evaluate_path(self, paths, velocities, goal_pose, robot_pose, obstacles):
        """
        Evaluate the simulated paths using the objective function.
        J = w_h * heading + w_v * vel + w_o * obst_dist
        """
        # detect nearest obstacle
        nearest_obs = self.calc_nearest_obs(robot_pose, obstacles)

        score_heading_angles = np.zeros(paths.shape[0])
        score_heading_vel = np.zeros(paths.shape[0])
        score_obstacles = np.zeros(paths.shape[0])

        # evaluate all pathes
        for i, path in enumerate(paths):
            # (1) heading_angle
            score_heading_angles[i] = self.score_heading_angle(path, goal_pose)
            # (2) heading_vel
            score_heading_vel[i] = self.score_heading_vel(velocities[i], path, goal_pose)
            # (3) obstacle
            score_obstacles[i] = self.score_obstacle(path, nearest_obs)

        # normalization
        score_heading_angles = normalize(score_heading_angles)
        score_heading_vel = normalize(score_heading_vel)
        score_obstacles = normalize(score_obstacles)

        score = 0.0

        for k in range(paths.shape[0]):
            temp_score = 0.0
            temp_score = self.weight_angle * score_heading_angles[k] + \
                         self.weight_vel * score_heading_vel[k] + \
                         self.weight_obs * score_obstacles[k]
  
            if temp_score > score:
                opt_idx = k
                score = temp_score

        try:
            return opt_idx
        except:
            ret = print("Cannot calculate optimal path!")

    def score_heading_angle(self, path, goal_pose):

        last_x = path[-1, 0]
        last_y = path[-1, 1]
        last_th = path[-1, 2]

        # calculate angle
        angle_to_goal = math.atan2(goal_pose[1]-last_y, goal_pose[0]-last_x)

        # calculate score
        score_angle = angle_to_goal - last_th
        score_angle = abs(normalize_angle(score_angle))
        score_angle = math.pi - score_angle

        return score_angle

    def score_heading_vel(self, u, path, goal_pose):

        vel = u[0]
        dist_to_goal = np.linalg.norm(path[-1, 0:2] - goal_pose)

        return vel + np.exp(-dist_to_goal / self.goal_dist_tol)

    def score_obstacle(self, path, nearest_obs):
        # obstacle avoidance
        score_obstacle = 2.0
        temp_dist_to_obs = 0.0

        for i in range(self.sim_step):
            for obs in nearest_obs: 
                temp_dis_to_obs = np.linalg.norm(path[i, 0:2] - obs)

                if temp_dis_to_obs < score_obstacle:
                    score_obstacle = temp_dist_to_obs # the nearest obstacle

                # collision with obstacle
                if temp_dis_to_obs < self.robot.size + self.obst_tolerance: # safety margin distance
                    score_obstacle = -100
                    break
            else:
                continue
            break
        return score_obstacle

    def calc_nearest_obs(self, robot_pose, obstacles):
        """
        Filter obstacles: find the ones in the local map considered for obstacle avoidance.
        """
        nearest_obs = []

        for obs in obstacles:
            temp_dist_to_obs = np.linalg.norm(robot_pose[0:2]-obs)

            if temp_dist_to_obs < self.obstacle_max_dist :
                nearest_obs.append(obs)

        return np.array(nearest_obs)




