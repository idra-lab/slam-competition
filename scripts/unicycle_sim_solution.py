import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from kalman_filter import *

import pyfiglet


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

        self.u = np.array([self.a, self.omega], dtype=float)

        self.states_log = [self.state.copy()]

    
    def dynamics(self, state, u):
        x, y, v, theta = state
        a, omega = u

        

        dx = v * np.cos(theta)
        dy = v * np.sin(theta)
        dv = a
        dtheta = omega

        return np.array([dx, dy, dv, dtheta], dtype=float)

    def step(self, delta_t= None):
        

        u = np.array([self.a, self.omega], dtype=float)
        self.u = u
        dstate = self.dynamics(self.state, u)

        if delta_t is None:
            delta_t = self.dt

        self.state += dstate * delta_t
        self.state[3] = self.state[3] % (2 * np.pi)  # wrap theta to [0, 2*pi]
        self.states_log.append(self.state.copy())



# ----------------------------
# Keyboard controller
# ----------------------------
class KeyboardController:
    def __init__(self, uni):
        self.uni = uni

        self.da = 1.0       # linear acceleration increment
        self.domega = 0.3   # angular velocity increment

    def on_key(self, event):
        if event.key == "up":
            self.uni.a += self.da
        elif event.key == "down":
            self.uni.a -= self.da
        elif event.key == "left":
            self.uni.omega += self.domega
        elif event.key == "right":
            self.uni.omega -= self.domega
        elif event.key == " ":
            self.uni.a = 0.0
            self.uni.state[2] = 0.0  # reset v
            self.uni.omega = 0.0
        

class Simulation: 

    def __init__(self):
        text = "Unicycle Simulation"

        print(pyfiglet.figlet_format(text, font='slant'))

        print("-Up/Down to increase/decrease forward acceleration\n-Left/Right to increase/decrease angular speed\n-Space to stop\n-Close the plot with 'q' window to exit.")
        self.uni = Unicycle()

        self.ctrl = KeyboardController(self.uni)

        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect("equal")
        self.ax.set_xlim(-5, 5)
        self.ax.set_ylim(-5, 5)
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
        L = 0.4
        hx = x + L * np.cos(th)
        hy = y + L * np.sin(th)
        self.head.set_data([x, hx], [y, hy])

        # update trajectory
        self.trajectory.set_data(*zip(*[state[:2] for state in self.uni.states_log]))

        return self.body, self.head, self.trajectory
    
    def run(self): 

        self.fig.canvas.mpl_connect("key_press_event", self.ctrl.on_key)
        self.ani = FuncAnimation(self.fig, self.update, init_func=self.init_sim, interval=20, blit=True, cache_frame_data=False)

        plt.show()




# ----------------------------
# Visualization + Animation
# ----------------------------
# def run_sim():

#     text = "Unicycle Simulation"

#     print(pyfiglet.figlet_format(text, font='slant'))

#     print("-Up/Down to increase/decrease forward speed\n-Left/Right to increase/decrease angular speed\n-Space to stop\n-Close the plot with 'q' window to exit.")
#     uni = Unicycle()

#     imu = IMU(dt=uni.dt)
#     ctrl = KeyboardController(uni)

#     fig, ax = plt.subplots()
#     ax.set_aspect("equal")
#     ax.set_xlim(-5, 5)
#     ax.set_ylim(-5, 5)
#     ax.grid(True)

#     # robot body marker
#     body, = ax.plot([], [], "bo", markersize=8)
#     # heading arrow
#     head, = ax.plot([], [], "r-", linewidth=2)

#     trajectory, = ax.plot([], [], "g--", linewidth=1)

#     def init():
#         body.set_data([], [])
#         head.set_data([], [])
#         return body, head

#     def update(frame):
#         uni.step()

#         x, y, th = uni.state

#         # IMU measurement at same rate as sim (can be higher later)
#         meas = imu.step(x, y, th, uni.v, uni.omega)

#         body.set_data([x], [y])

#         # heading arrow length
#         L = 0.4
#         hx = x + L * np.cos(th)
#         hy = y + L * np.sin(th)
#         head.set_data([x, hx], [y, hy])

#         # update trajectory
#         trajectory.set_data(*zip(*[state[:2] for state in uni.states_log]))

#         return body, head, trajectory

#     fig.canvas.mpl_connect("key_press_event", ctrl.on_key)
#     ani = FuncAnimation(fig, update, init_func=init, interval=20, blit=True, cache_frame_data=False)

#     plt.show()


if __name__ == "__main__":
    # run_sim()

    sim = Simulation()

    sim.run()
