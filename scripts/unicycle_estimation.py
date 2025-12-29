from kalman_filter import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

import pyfiglet


FPS = 90  # frames per second
N_LANDMARKS = 40
AREA_LOWER_BOUND = -15.0
AREA_UPPER_BOUND = 15.0
HEADING_ARROW_LENGTH = 0.5


# ----------------------------
# Unicycle model
# ----------------------------
class Unicycle:
    def __init__(self, x=0.0, y=0.0, v=0.0, theta=0.0, dt=0.02):
        self.state = np.array([x, y, v, theta], float)
        
        self.dt = dt
        self.a = 0.0
        self.v = v
        self.omega = 0.0

        self.v_cmd = 0.0      # linear acceleration command
        self.theta_cmd = 0.0   # angular velocity command

        self.u_cmd = np.array([self.a, self.omega], dtype=float)
        self.previous_u_cmd = self.u_cmd.copy()

        self.u = self.u_cmd.copy()

        self.states_log = [self.state.copy()]

        self.K_p = np.array([0.5, 0.5], dtype=float)  # Proportional gains for v and theta

    
    def dynamics(self, state, u_cmd):

        # Higher order dynamics for control inputs
        
        x, y, v, theta = state

        a, omega = self.u


        

        dx = v * np.cos(theta)
        dy = v * np.sin(theta)
        dv = a
        dtheta = omega

        return np.array([dx, dy, dv, dtheta], dtype=float)

    def step(self, delta_t= None):
        

        u_cmd = np.array([self.v_cmd, self.theta_cmd], dtype=float)

        v = self.state[2]
        theta = self.state[3]

        error = u_cmd - np.array([v, theta], dtype=float)


        self.u = self.K_p * error # Acceleration and angular velocity commands are obtained proportionally

        dstate = self.dynamics(self.state, self.u)

        if delta_t is None:
            delta_t = self.dt

        self.state += dstate * delta_t
        self.state[3] = (self.state[3] + np.pi) % (2 * np.pi) - np.pi  # wrap theta to [-pi, pi]
        self.states_log.append(self.state.copy())



# ----------------------------
# Keyboard controller
# ----------------------------
class KeyboardController:
    def __init__(self, uni):
        self.uni = uni

        self.v_cmd = 0.0      # linear acceleration increment
        self.theta_cmd = 0.0   # angular velocity increment

        self.delta_v = 1.0   # linear acceleration step
        self.delta_theta = np.pi / 6  # angular velocity step

    def on_key(self, event):
        # key press: apply command and keep it until release (like an accelerator)
        if event.key == "up":
            self.uni.v_cmd += self.delta_v
        elif event.key == "down":
            self.uni.v_cmd -= self.delta_v
        elif event.key == "left":
            self.uni.theta_cmd += self.delta_theta
            self.uni.theta_cmd = (self.uni.theta_cmd + np.pi) % (2 * np.pi) - np.pi
        elif event.key == "right":
            self.uni.theta_cmd -= self.delta_theta
            self.uni.theta_cmd = (self.uni.theta_cmd + np.pi) % (2 * np.pi) - np.pi
        elif event.key == " ":
            # immediate stop / brake
            self.uni.v_cmd = 0.0
            self.uni.theta_cmd = 0.0
            

    
        

class Simulation: 

    def __init__(self):
        text = "Unicycle Simulation"

        print(pyfiglet.figlet_format(text, font='slant'))

        print("-Up/Down to increase/decrease forward acceleration\n-Left/Right to increase/decrease angular speed\n-Space to stop\n-Close the plot with 'q' window to exit.")
        self.uni = Unicycle()

        self.ctrl = KeyboardController(self.uni)

        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect("equal")
        self.ax.set_xlim(AREA_LOWER_BOUND-1, AREA_UPPER_BOUND+1)
        self.ax.set_ylim(AREA_LOWER_BOUND-1, AREA_UPPER_BOUND+1)
        self.ax.grid(True)

        # robot body marker
        self.body, = self.ax.plot([], [], "bo", markersize=8)
        # heading arrow
        self.head, = self.ax.plot([], [], "r-", linewidth=2)

        self.trajectory, = self.ax.plot([], [], "g--", linewidth=1)

    def init_sim(self):
        self.body.set_data([], [])
        self.head.set_data([], [])
        return self.body, self.head
    
    def update(self, frame): 

        self.uni.step()

        x, y, v, th = self.uni.state

        self.body.set_data([x], [y])

        # heading arrow length
        hx = x + HEADING_ARROW_LENGTH * np.cos(th)
        hy = y + HEADING_ARROW_LENGTH * np.sin(th)
        self.head.set_data([x, hx], [y, hy])

        # update trajectory
        self.trajectory.set_data(*zip(*[state[:2] for state in self.uni.states_log]))

        return self.body, self.head, self.trajectory
    
    def run(self): 

        self.fig.canvas.mpl_connect("key_press_event", self.ctrl.on_key)
        self.ani = FuncAnimation(self.fig, self.update, init_func=self.init_sim, interval=20, blit=True, cache_frame_data=False)

        plt.show()


# ----------------------------
# Planar IMU model
# ----------------------------
class IMU:
    """
    Planar IMU:
      - gyro_z
      - accel_body (ax, ay)
    Using finite differences of world velocity to get world acceleration.
    """
    def __init__(self, dt,
                 gyro_bias=0.001, accel_bias=(0.005, -0.003),
                 gyro_noise_std=0.002, accel_noise_std=0.002,
                 gyro_bias_rw_std=0.0005, accel_bias_rw_std=0.0002,
                 seed=0):
        self.dt = dt
        self.rng = np.random.default_rng(seed)

        self.bg = float(gyro_bias)
        self.ba = np.array(accel_bias, dtype=float)

        self.gyro_noise_std = float(gyro_noise_std)
        self.accel_noise_std = float(accel_noise_std)

        self.gyro_bias_rw_std = float(gyro_bias_rw_std)
        self.accel_bias_rw_std = float(accel_bias_rw_std)

        self.prev_vw = None  # previous world velocity (2,)
        self.log = []        # list of dicts

    def step(self, state, u, delta_t=None):

        x,y,v,th = state
        a, omega = u

        if delta_t is None:
            delta_t = self.dt

        # bias random walk (optional, but realistic)
        self.bg += self.gyro_bias_rw_std * np.sqrt(delta_t) * self.rng.normal()
        self.ba += self.accel_bias_rw_std * np.sqrt(delta_t) * self.rng.normal(size=2)

        # world velocity
        vw = np.array([v*np.cos(th), v*np.sin(th)])

        a_w = a * np.array([np.cos(th), np.sin(th)]) + omega * v * np.array([-np.sin(th), np.cos(th)])

        # rotate to body frame
        R = rot2(th)
        ab = R.T @ a_w

        # measurements with bias + noise
        gyro_z = (omega + self.bg) + self.gyro_noise_std * self.rng.normal()
        accel_b = (ab + self.ba) + self.accel_noise_std * self.rng.normal(size=2)
        
        # accel_b = ab 
        # gyro_z = omega 

        meas = np.array([accel_b[0], accel_b[1], gyro_z]).reshape((3,1))
        self.log.append(meas)
        return meas


class Camera: 
    """
    Simple pinhole camera model for 2D landmarks.
    """
    def __init__(self, intrinsics=np.array([0, 1]), dt=1.5):
        self.intrinsics = intrinsics  # Placeholder for camera intrinsics
        self.fov = np.pi / 4  # 45 degrees field of view
        self.min_range = 1.0
        self.max_range = 10.0
        self.noise = 0.001  # pixel noise standard deviation

        self.dt = dt

    def project(self, landmark_pos, robot_pos, R_BW):

        landmark_body = R_BW @ (landmark_pos - robot_pos)
        landmark_body = landmark_body.reshape((2, 1))
        # Simple projection ignoring distortion etc.
        l_hat_b = landmark_body / landmark_body[0]  # Normalize by x (assuming pinhole at x=1)
        u = self.intrinsics @ l_hat_b  # focal_length * y + principal_point
        return u  # Placeholder projection

    def check_fov(self, landmark_pos, robot_pos, robot_theta):
        # Placeholder for field of view check

        dist = landmark_pos - robot_pos
        angle = np.arctan2(dist[1], dist[0])

        l_b = rot2(-robot_theta) @ dist
        angle = np.arctan2(l_b[1], l_b[0])

        if l_b[0] <= self.min_range or l_b[0] >= self.max_range:
            return False 
        if angle > self.fov / 2 or angle < -self.fov / 2:
            return False
        
        

        return True
    
    def step(self, landmarks, robot_pos, robot_theta):
        measurements =  {'landmarks': [], 'pixels': []}
        R_BW = rot2(robot_theta).T  # Rotation from world to body frame
        for lm in landmarks:
            if self.check_fov(lm, robot_pos, robot_theta):
                z = self.project(lm, robot_pos, R_BW)
                z += self.noise * np.random.normal(size=1)  # Add noise
                measurements['landmarks'].append(lm)
                measurements['pixels'].append(z)
        return measurements
    
# ----------------------------
# Simulation with EKF
# ----------------------------
class SimulationWithEstimation():

    def __init__(self):
        text = "Unicycle Simulation with Estimation"

        print(pyfiglet.figlet_format(text, font='slant'))

        print("-Up/Down to increase/decrease forward speed\n-Left/Right to increase/decrease angular speed\n-Space to stop\n-Close the plot with 'q' window to exit.")
        self.delta_t = 1/FPS
        self.interval = 1000 / FPS  # in milliseconds

        self.uni = Unicycle(dt=self.delta_t)

        self.imu = IMU(dt=self.delta_t)
        self.camera = Camera(dt=self.delta_t)
        self.ctrl = KeyboardController(self.uni)

        rng = np.random.default_rng(42)
        num_landmarks = N_LANDMARKS
        coords = rng.uniform(AREA_LOWER_BOUND, AREA_UPPER_BOUND, size=(num_landmarks, 2))
        landmark_positions = [tuple(coord) for coord in coords]
        self.landmarks = [np.array(pos, dtype=float) for pos in landmark_positions]
        
        self.ekf = ExtendedKalmanFilter()

        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect("equal")
        self.ax.set_xlim(AREA_LOWER_BOUND-1, AREA_UPPER_BOUND+1)
        self.ax.set_ylim(AREA_LOWER_BOUND-1, AREA_UPPER_BOUND+1)
        self.ax.grid(True)

        self.visible_landmarks = self.ax.scatter(*zip(*landmark_positions), marker='x', color='grey')

        self.colors = np.full(len(self.landmarks), 'red')

        # robot body marker
        self.body, = self.ax.plot([], [], "bo", markersize=8)
        # heading arrow
        self.head, = self.ax.plot([], [], "r-", linewidth=2)

        self.trajectory, = self.ax.plot([], [], "g--", linewidth=1)
        self.estimated_trajectory, = self.ax.plot([], [], "m--", linewidth=1)
        self.predicted_trajectory, = self.ax.plot([], [], "b--", linewidth=1)
        # self.visible_landmarks = self.ax.scatter([], [], marker='o', color='cyan', s=50, label='Visible Landmarks')

    def init_sim(self):
        self.body.set_data([], [])
        self.head.set_data([], [])
        return self.body, self.head
    
    def update(self, frame): 

        """
        We take the duration of one frame and we split it so that we have one IMU step at the beginning and one in the middle, then a camera step at 0.75 of the frame.
        """
        
        # One frame means we run 1 IMU and 1 camera step
        meas = self.imu.step(self.uni.state, self.uni.u, delta_t=self.delta_t) 

        self.ekf.predict(imu_measurement=meas)

        

        camera_measurements = self.camera.step(self.landmarks, self.uni.state[0:2], self.uni.state[3]) 

        kalman_estimate = self.ekf.update(camera_measurements, delta_t=self.delta_t)

        colors =  np.repeat(np.array([[0.5, 0.5, 0.5, 1.0]]), len(self.landmarks), axis=0) # grey for not visible
        for landmark in camera_measurements['landmarks']:
            index =  np.where(self.landmarks == landmark)

            colors[index[0]] = [0, 1, 0, 1]  # green for visible

        self.visible_landmarks.set_facecolors(colors)
        self.visible_landmarks.set_edgecolors(colors)

        

        x, y, v, th = self.uni.state

        def plot_robot():
            self.body.set_data([x], [y])
            # heading arrow length
            hx = x + HEADING_ARROW_LENGTH * np.cos(th)
            hy = y + HEADING_ARROW_LENGTH * np.sin(th)
            self.head.set_data([x, hx], [y, hy])
            # update trajectory
            self.trajectory.set_data(*zip(*[state[:2] for state in self.uni.states_log]))
            self.estimated_trajectory.set_data(*zip(*[state[:2].flatten() for state in self.ekf.estimated_states_log]))
            self.predicted_trajectory.set_data(*zip(*[state[:2].flatten() for state in self.ekf.predicted_states_log]))

        plot_robot()

        self.uni.step(delta_t=self.delta_t) 
        return self.body, self.head, self.trajectory, self.estimated_trajectory, self.predicted_trajectory, self.visible_landmarks

   

    def run(self):

        self.fig.canvas.mpl_connect("key_press_event", self.ctrl.on_key)
        self.ani = FuncAnimation(self.fig, self.update, init_func=self.init_sim, interval=self.interval, blit=True, cache_frame_data=False)

        plt.show()


def main():
    sim  = SimulationWithEstimation()

    sim.run()



if __name__ == "__main__":
    main()


