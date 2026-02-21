import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from main import main

time_steps, state_values, control_values, setpoint_values, L, dt = main()

thetas = [s[0] for s in state_values]
total_frames = len(thetas)

fig, ax = plt.subplots()
ax.set_xlim(-L*1.5, L*1.5)
ax.set_ylim(-L*1.5, L*1.5)
ax.set_aspect('equal')

line, = ax.plot([], [], 'o-', lw=2)

def update(frame):
    theta = thetas[frame]
    x = L * np.sin(theta)
    y = L * np.cos(theta)
    line.set_data([0,x],[0,y])
    return line,

ani = animation.FuncAnimation(fig, update, frames=len(thetas), interval=(10000/total_frames))
plt.show()
