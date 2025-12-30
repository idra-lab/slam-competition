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

        self.u = np.array([self.a, self.omega], dtype=float)

        self.states_log = [self.state.copy()]

        self.K_p = np.array([0.5, 2.0], dtype=float)  # Proportional gains for v and theta

    
    def dynamics(self, state, u):

        # Higher order dynamics for control inputs
        
        x, y, v, theta = state

        a, omega = u


        

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

        error[1] = (error[1] + np.pi) % (2 * np.pi) - np.pi  # wrap angle error to [-pi, pi]


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

        self.v_cmd = 0.0      # linear velocity command
        self.theta_cmd = 0.0   # angular velocity command

        self.delta_v = 1.0   # linear velocity step
        self.delta_theta = np.pi / 6  # heading step

    def on_key(self, event):

        if event.key == "up":
            self.uni.v_cmd += self.delta_v
        elif event.key == "down":
            self.uni.v_cmd -= self.delta_v
        elif event.key == "left":
            self.uni.theta_cmd += self.delta_theta
        elif event.key == "right":
            self.uni.theta_cmd -= self.delta_theta
        elif event.key == " ":
            # immediate stop / brake
            self.uni.v_cmd = 0.0
            self.uni.theta_cmd = 0.0
            


# ----------------------------
# Planar IMU model
# ----------------------------
class IMU:
    """
    Planar IMU:
      - gyro_z
      - accel_body (ax, ay)
    Compute acceleration and angular velocity in body frame with noise.
    """
    def __init__(self, dt,
                 gyro_bias=0.001, accel_bias=(0.005, -0.003),
                 gyro_noise_std=0.002, accel_noise_std=0.002,
                 gyro_bias_rw_std=0.0005, accel_bias_rw_std=0.0002,
                 seed=0):
        
        self.dt = dt # time step
        self.rng = np.random.default_rng(seed) # random number generator for reproducibility

        self.bg = float(gyro_bias) # Initial gyroscope bias
        self.ba = np.array(accel_bias, dtype=float) # Initial accelerometer bias


        self.gyro_noise_std = float(gyro_noise_std) # Std deviation of gyroscope noise
        self.accel_noise_std = float(accel_noise_std) # Std deviation of accelerometer noise

        self.gyro_bias_rw_std = float(gyro_bias_rw_std) # Std deviation of gyro bias random walk
        self.accel_bias_rw_std = float(accel_bias_rw_std) # Std deviation of acceleration bias random walk

        self.log = []        # log of measurements

    def step(self, state, u, delta_t=None):

        x,y,v,th = state
        a, omega = u

        if delta_t is None:
            delta_t = self.dt

        # bias random walk update
        self.bg +=  delta_t * self.rng.normal(scale=self.gyro_bias_rw_std)
        self.ba +=  delta_t * self.rng.normal(scale=self.accel_bias_rw_std, size=2)

        # world acceleration
        a_w = a * np.array([np.cos(th), np.sin(th)]) + omega * v * np.array([-np.sin(th), np.cos(th)])

        # rotate to body frame
        R = rot2(th)
        ab = R.T @ a_w

        # measurements with bias + noise
        gyro_z = (omega + self.bg) + self.rng.normal(scale=self.gyro_noise_std)
        accel_b = (ab + self.ba) + self.rng.normal(scale=self.accel_noise_std, size=2)

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

        # Transform landmark to body frame
        landmark_body = R_BW @ (landmark_pos - robot_pos)

        # Fix numpy shape to have awkward matrix multiplication
        landmark_body = landmark_body.reshape((2, 1))

        # Simple projection ignoring distortion etc.
        l_hat_b = landmark_body / landmark_body[0]  # Normalize by x (assuming pinhole at x=1)
        z = self.intrinsics @ l_hat_b  # focal_length * y + principal_point

        return z  # 1 Dimensional pixel measurement

    def check_fov(self, landmark_pos, robot_pos, robot_theta):
        """
        Fied of view check for a landmark given robot position and orientation."""
    
        # Compute distance vector 
        dist = landmark_pos - robot_pos

        # Transform to body frame
        l_b = rot2(-robot_theta) @ dist
        
        # Compute angle wrt robot heading
        angle = np.arctan2(l_b[1], l_b[0])

        # Check depth 
        if l_b[0] <= self.min_range or l_b[0] >= self.max_range:
            return False 
        
        # Check angle within FOV
        if angle > self.fov / 2 or angle < -self.fov / 2:
            return False
        
        # Landmark is visible
        return True
    

    def step(self, landmarks, robot_pos, robot_theta):
        """
        Based on robot pose, simulate camera measurements of visible landmarks.
        """

        # Dictionary of measurements
        measurements =  {'landmarks': [], 'pixels': []}

        # Compute rotation matrix from world to body frame
        R_BW = rot2(robot_theta).T  # Rotation from world to body frame is nothing more than the transpose of the rotation from body frame to world (What we use to describe the robot's orientation)

        # Iterate over all landmarks and check visibility
        for lm in landmarks:

            # Check if landmark is in FOV
            if self.check_fov(lm, robot_pos, robot_theta):

                # Project landmark to pixel coordinates
                z = self.project(lm, robot_pos, R_BW)

                # Introduce fictitious noise
                z += np.random.normal(size=1, scale=self.noise) 

                # Append measurements and associated landmark
                measurements['landmarks'].append(lm)
                measurements['pixels'].append(z)
                
        return measurements
    
# ----------------------------
# Simulation with EKF
# ----------------------------
class SimulationWithEstimation():
    """
    Simulate a unicycle robot with IMU and camera, and estimate its state using an Extended Kalman Filter.
    """

    def __init__(self):

        
        text = "Unicycle Simulation with Estimation"

        print(pyfiglet.figlet_format(text, font='slant'))

        print("Commands:\n-Up/Down to increase/decrease commanded speed\n"
        "-Left/Right to increase/decrease heading\n"
        "-Space to stop the robot\n"
        "-Close the plot with 'q' window to exit.")

        print("\n\n\nExplanation:\n\nGreen landmarks are visible, grey are not."
              "The robot is blue, its heading is red." \
              "The true trajectory is green dashed, the EKF estimated trajectory is red dashed.")


        # ----------------------------
        # Simulation parameters: 
        # ----------------------------
        self.delta_t = 1/FPS # simulation time step
        self.interval = 1000 / FPS  # visualization interval in milliseconds

        self.uni = Unicycle(dt=self.delta_t) # Instantiate the robot unicycle model

        self.imu = IMU(dt=self.delta_t) # Instantiate the planar IMU model
        self.camera = Camera(dt=self.delta_t) # Instantiate the camera model
        self.ctrl = KeyboardController(self.uni) # Instantiate the keyboard controller

        # Generate random landmarks
        rng = np.random.default_rng(42) # Seed generator for reproducibility
        num_landmarks = N_LANDMARKS # Select number of landmarks from above
        coords = rng.uniform(AREA_LOWER_BOUND, AREA_UPPER_BOUND, size=(num_landmarks, 2)) # Uniformly distribute landmarks in the area
        landmark_positions = [tuple(coord) for coord in coords] # Convert to list of tuples for easier handling
        self.landmarks = [np.array(pos, dtype=float) for pos in landmark_positions] # Convert to list of numpy arrays
        
        self.ekf = ExtendedKalmanFilter() # Instantiate the Extended Kalman Filter for estimation

        # ----------------------------
        # Visualization setup
        # ----------------------------

        self.fig, self.ax = plt.subplots() # Create matplotlib figure and axis for visualization
        self.ax.set_aspect("equal")
        self.ax.set_xlim(AREA_LOWER_BOUND-1, AREA_UPPER_BOUND+1)
        self.ax.set_ylim(AREA_LOWER_BOUND-1, AREA_UPPER_BOUND+1)
        self.ax.grid(True)

        # Position some text box for commands and info
        self.cmd_text = self.ax.text(
        0.02, 0.98, "",                 # position in axes coords
        transform=self.ax.transAxes,    # axes-relative coordinates
        fontsize=10,
        verticalalignment="top",
        bbox=dict(boxstyle="round", facecolor="white", alpha=0.8), usetex=True
        )

        # Plot some landmark markers
        self.visible_landmarks = self.ax.scatter(*zip(*landmark_positions), marker='x', color='grey')

        # Instantiate colors so that we can update visible landmarks in green
        self.colors = np.full(len(self.landmarks), 'red')

        # Robot body marker to draw 
        self.body, = self.ax.plot([], [], "bo", markersize=8)
        # Heading direction marker
        self.head, = self.ax.plot([], [], "r-", linewidth=2)

        # Instantiate a true and estimated robot trajectory to draw
        self.trajectory, = self.ax.plot([], [], "g--", linewidth=1)
        self.estimated_trajectory, = self.ax.plot([], [], "m--", linewidth=1)

        # Count frames to refresh command text periodically
        self.frame_i = 0


    def init_sim(self):
        # Initialize the simulation visualization
        self.body.set_data([], [])
        self.head.set_data([], [])

        return self.body, self.head
    
    def update(self, frame): 

        """
        Update function for each simulation frame.

        The update takes the current visualization frame as input and performs the IMU measurement, the EKF prediction
        the camera measurement and the EKF update. 
        Later, it updates the robot and landmark visualization and pushes the physics of the unicycle forward.
        """
        
        # One frame means we run 1 IMU and 1 camera step

        # IMU measurement from current state, input and delta_t
        meas = self.imu.step(self.uni.state, self.uni.u, delta_t=self.delta_t) 

        # We use the measurement to predict the next state with the EKF
        self.ekf.predict(imu_measurement=meas)

        # Camera measurement from current state and landmarks
        camera_measurements = self.camera.step(self.landmarks, self.uni.state[0:2], self.uni.state[3]) 

        # Use the camera measurements to update the EKF, correcting the prediction
        kalman_estimate = self.ekf.update(camera_measurements, delta_t=self.delta_t)

        # Update landmark colors based on visibility
        colors =  np.repeat(np.array([[0.5, 0.5, 0.5, 1.0]]), len(self.landmarks), axis=0) # grey for not visible
        
        # Each visible landmark will be colored green
        for landmark in camera_measurements['landmarks']:
            index =  np.where(self.landmarks == landmark)

            colors[index[0]] = [0, 1, 0, 1]  # green for visible

        # Set the scatter plot colors
        self.visible_landmarks.set_facecolors(colors) #Face
        self.visible_landmarks.set_edgecolors(colors) #Edge 

        # Extract current true state for visualization
        x, y, v, th = self.uni.state

        # Helper function to plot the robot, it is called immediately below
        def plot_robot():
            
            # update robot body position
            self.body.set_data([x], [y])

            # heading arrow length computation and update
            hx = x + HEADING_ARROW_LENGTH * np.cos(th)
            hy = y + HEADING_ARROW_LENGTH * np.sin(th)
            self.head.set_data([x, hx], [y, hy])
            
            # update trajectory
            self.trajectory.set_data(*zip(*[state[:2] for state in self.uni.states_log]))
            self.estimated_trajectory.set_data(*zip(*[state[:2].flatten() for state in self.ekf.estimated_states_log]))

        # Actually plot the robot
        plot_robot()

        # Update command text every 60 frames as it would slow down the simulation otherwise
        if self.frame_i % 60 == 0:
            plt.pause(0.001)  # small pause to update the plot
            self.cmd_text.set_text(
            r"$v_{cmd}$" f"= {self.uni.v_cmd:+.2f}" r"$ \frac{m}{s²}$" f"\n"
            r"$\theta_{cmd}$" f"={self.uni.theta_cmd/np.pi*180:+.2f}" r"$deg$" f"\n"
            r"$v$" f"= {v:.2f}" r"$\frac{m}{s}$" f"\n"
            r"$\theta$" f"= {th/np.pi*180:.2f} deg"
            )

        # Step the unicycle physics forward
        self.uni.step(delta_t=self.delta_t) 

        # Update the frame counter
        self.frame_i += 1

        return self.body, self.head, self.trajectory, self.estimated_trajectory, self.visible_landmarks, self.cmd_text

   

    def run(self):
        """
        Call this method to effectively start the simulation with estimation.
        It sets up the matplotlib animation and shows the plot.
        """

        # Connect the key press event to the KeyboardController function on_key
        self.fig.canvas.mpl_connect("key_press_event", self.ctrl.on_key)

        # Set up the animation function
        self.ani = FuncAnimation(
            self.fig,  # Figure object we define in __init__
            self.update,  # Update function defined above
            init_func=self.init_sim,  # Initialization function defined above
            interval=self.interval,  # Interval between frames defined in __init__
            blit=True, cache_frame_data=False)

        plt.show() # Show the plot and start the animation loop


def main():
    """
    This is the main function body that runs the simulation
    """
    sim  = SimulationWithEstimation() # Instantiate the simulation with estimation. This means that the constructor SimulationWithEstimation.__init__() is called here.

    # Now we call the method that runs the simulation as we have defined it above.
    sim.run()



if __name__ == "__main__": # This string allows us to run the main function only if this script is executed directly (python3 <script_name.py>), not imported as a module in other scripts.
    

    main() # Execute the main function defined above.


