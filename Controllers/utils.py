import numpy as np
import sympy
from sympy import symbols, Matrix


def normalize(arr: np.ndarray):
    """normalize vector for plots"""
    return (arr - np.min(arr)) / (np.max(arr) - np.min(arr))


def normalize_angle(theta):
    """
    Normalize angles between [-pi, pi)
    """
    theta = np.remainder(theta, 2.0 * np.pi)  # force in range [0, 2 pi)
    if np.isscalar(theta):
        if theta > np.pi:  # move to [-pi, pi)
            theta -= 2.0 * np.pi
    else:
        theta_ = theta.copy()
        theta_[theta > np.pi] -= 2.0 * np.pi
        return theta_

    return theta


class RobotStates:
    """
    Class to store robot states (x, y, yaw, v, t)
    """

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []
        self.pind = []
        self.a = []

    def append(self, t, pind, state, a):
        """
        Append a state to the list of states

        Parameters
        ----------
        t : float
            time
        pind : int
            index of the waypoint
        state : object
            state of the robot
        a : float
            acceleration
        """
        self.x.append(state.x.flatten()[0])
        self.y.append(state.x.flatten()[1])
        self.yaw.append(state.x.flatten()[2])
        self.v.append(state.u.flatten()[0])
        self.a.append(a)
        self.t.append(t)
        self.pind.append(pind)

    def __len__(self):
        return len(self.x)


def interpolate_waypoints(waypoints, resolution=0.01):
    """
    Interpolate the waypoints to add more points along the path

    Args:
    waypoints: array of waypoints
    resolution: distance between two interpolated points

    Return:
    interpolated_waypoints: array of interpolated waypoints
    """
    interpolated_waypoints = []
    for i in range(len(waypoints) - 1):
        p1 = waypoints[i]
        p2 = waypoints[i + 1]
        dist = np.linalg.norm(p2 - p1)
        n_points = int(dist / resolution)
        x = np.linspace(p1[0], p2[0], n_points)
        y = np.linspace(p1[1], p2[1], n_points)
        interpolated_waypoints += list(zip(x, y))
    return np.array(interpolated_waypoints)


# decorator
def squeeze_sympy_out(func):
    # inner function
    def squeeze_out(*args):
        out = func(*args).squeeze()
        return out

    return squeeze_out


def velocity_mm_simpy():
    """
    Define Jacobian Gt w.r.t state x=[x, y, theta] and Vt w.r.t command u=[v, w]
    """
    x, y, theta, v, w, dt = symbols("x y theta v w dt")

    gux = Matrix(
        [
            [x + v * dt * sympy.cos(theta)],
            [y + v * dt * sympy.sin(theta)],
            [theta + w * dt],
        ]
    )

    eval_gux = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w, dt), gux, "numpy"))

    Gt = gux.jacobian(Matrix([x, y, theta]))
    eval_Gt = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w, dt), Gt, "numpy"))

    Vt = gux.jacobian(Matrix([v, w]))
    eval_Vt = squeeze_sympy_out(sympy.lambdify((x, y, theta, v, w, dt), Vt, "numpy"))

    return eval_gux, eval_Gt, eval_Vt


def acceleration_mm_simpy():
    """Define the acceleration law"""
    a, vt_1, dt = symbols(r"a v_{t-1} dt")
    fva = vt_1 + a * dt
    eval_fva = squeeze_sympy_out(sympy.lambdify((vt_1, a, dt), fva, "numpy"))

    return eval_fva


def proportional_control(v_target, v_current, kp=3.0):
    """
    Compute the control input using proportional control law
    a = kp * (v_target - v_current)

    Args:
    v_target: target velocity
    v_current: current velocity

    Return:
    a: control input computed using proportional control
    """
    a = kp * (v_target - v_current)
    return a


class Robot:
    def __init__(self, dim_x=1, dim_u=1, eval_gux=None, eval_fva=None):
        """
        Initialize the robot creating the necessary matrices
        """
        self.x = np.zeros((dim_x, 1))  # state variables
        self.u = np.zeros((dim_u, 1))  # control variables

        self.eval_gux = eval_gux
        self.eval_fva = eval_fva

        # Robot properties
        self.max_linear_velocity = 0.22  # m/s
        self.max_angular_velocity = 1.5  # rad/s
        self.max_acceleration = 0.5  # m/s^2

    @property
    def v(self):
        return self.u[0, 0]

    @property
    def w(self):
        return self.u[1, 0]

    def velocity_update(self, a, dt):
        """
        Update the state of the robot using the control input u

        Modified variables:
        self.u: updated input
        """
        # Update the state evauating the motion model
        args = (*self.u[0].flatten(), a, dt)
        # Saturate the velocities with np.clip(value, min, max)
        a = np.clip(a, -self.max_acceleration, self.max_acceleration)
        self.u[0] = self.eval_fva(*args)  # evaluate the motion model
        self.u[0] = np.clip(
            self.u[0], -self.max_linear_velocity, self.max_linear_velocity
        )
        return self.u[0]

    def update_state(self, a, w, dt):
        """
        Update the state of the robot using the control input u

        Modified variables:
        self.x: updated state
        """
        w_ = np.clip(w, -self.max_angular_velocity, self.max_angular_velocity)
        v_ = self.velocity_update(a, dt)
        # Update the state evauating the motion model
        args = (*self.x.flatten(), v_.flatten(), w_.flatten(), dt)
        self.x = self.eval_gux(*args)  # evaluate the motion model
        self.x[2] = normalize_angle(
            self.x[2]
        )  # normalize the orientation between [-pi, pi]
        return self.x
