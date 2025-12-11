import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 'pip install pyfiglet' if not already installed
import pyfiglet


# ----------------------------
# Unicycle model
# ----------------------------
class Unicycle:
    def __init__(self, x=0.0, y=0.0, theta=0.0, dt=0.02):
        self.state = np.array([x, y, theta], float)
        self.dt = dt
        self.v = 0.0
        self.omega = 0.0

        self.states_log = [self.state.copy()]

    
    def dynamics(self, state, u):
        x, y, theta = state
        v, omega = u

        """Compute state derivatives here"""
        # dx = ...
        # dy = ...
        # dtheta = ...

        return np.array([dx, dy, dtheta], dtype=float)

    def step(self):

        u = np.array([self.v, self.omega], dtype=float)
        dstate = self.dynamics(self.state, u)

        """Integrate the dynamics here (Euler integration)"""
        #self.state += ...

        self.states_log.append(self.state.copy())



# ----------------------------
# Keyboard controller
# ----------------------------
class KeyboardController:
    def __init__(self, uni):
        self.uni = uni

        self.dv = 0.2       # linear velocity increment
        self.domega = 0.3   # angular velocity increment

    def on_key(self, event):
        if event.key == "up":
            self.uni.v += self.dv
        elif event.key == "down":
            self.uni.v -= self.dv
        elif event.key == "left":
            self.uni.omega += self.domega
        elif event.key == "right":
            self.uni.omega -= self.domega
        elif event.key == " ":
            self.uni.v = 0.0
            self.uni.omega = 0.0
        


# ----------------------------
# Visualization + Animation
# ----------------------------
def run_sim():

    text = "Unicycle Simulation"

    print(pyfiglet.figlet_format(text, font='slant'))

    print("-Up/Down to increase/decrease forward speed\n-Left/Right to increase/decrease angular speed\n-Space to stop\n-Close the plot with 'q' window to exit.")
    uni = Unicycle()
    ctrl = KeyboardController(uni)

    fig, ax = plt.subplots()
    ax.set_aspect("equal")
    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.grid(True)

    # robot body marker
    body, = ax.plot([], [], "bo", markersize=8)
    # heading arrow
    head, = ax.plot([], [], "r-", linewidth=2)

    trajectory, = ax.plot([], [], "g--", linewidth=1)

    def init():
        body.set_data([], [])
        head.set_data([], [])
        return body, head

    def update(frame):
        uni.step()

        x, y, th = uni.state
        body.set_data([x], [y])

        # heading arrow length
        L = 0.4
        hx = x + L * np.cos(th)
        hy = y + L * np.sin(th)
        head.set_data([x, hx], [y, hy])

        # update trajectory
        trajectory.set_data(*zip(*[state[:2] for state in uni.states_log]))

        return body, head, trajectory

    fig.canvas.mpl_connect("key_press_event", ctrl.on_key)
    ani = FuncAnimation(fig, update, init_func=init, interval=20, blit=True, cache_frame_data=False)

    plt.show()


if __name__ == "__main__":
    run_sim()
