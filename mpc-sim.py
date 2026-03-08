import importlib
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
import numpy as np

# mpc-main.py has a bare main() call at module level (no __name__ guard),
# so suppress plt.show during import to prevent it from blocking
_real_show = plt.show
plt.show = lambda *a, **kw: None
mpc_main = importlib.import_module("mpc-main")
plt.show = _real_show

init = mpc_main.init
mpc_step = mpc_main.mpc_step
plant_sim = mpc_main.plant_sim

g,L,m,theta,dot_theta,dt,horizon,total_steps,A,B,x,x_id,u,u_max,M,C,Q_m,R_m = init()

theta_hist = []
u_hist = []
time_hist = []

for i in range(total_steps):
    theta_hist.append(np.rad2deg(x[0, 0]))
    u_hist.append(u)
    time_hist.append(i * dt)
    u = mpc_step(x, horizon, u_max, M, C, Q_m, R_m)
    x = plant_sim(x, u, g, L, m, dt)

total_frames = len(theta_hist)

fig = plt.figure(figsize=(12, 5))
gs = gridspec.GridSpec(2, 2)

ax_anim = fig.add_subplot(gs[:, 0])
ax_anim.set_xlim(-L * 1.5, L * 1.5)
ax_anim.set_ylim(-L * 1.5, L * 1.5)
ax_anim.set_aspect('equal')
ax_anim.set_title('pendulum')
line, = ax_anim.plot([], [], 'o-', lw=2)

ax_theta = fig.add_subplot(gs[0, 1])
ax_theta.plot(time_hist, theta_hist)
ax_theta.set_ylabel('angle (deg)')
ax_theta.set_title('angle over time')
ax_theta.axhline(0, color='r', linestyle='--', linewidth=0.8)

ax_u = fig.add_subplot(gs[1, 1])
ax_u.plot(time_hist, u_hist)
ax_u.set_ylabel('torque (Nm)')
ax_u.set_xlabel('time (s)')
ax_u.set_title('control input')

plt.tight_layout()

def update(frame):
    theta_rad = np.deg2rad(theta_hist[frame])
    px = L * np.sin(theta_rad)
    py = L * np.cos(theta_rad)
    line.set_data([0, px], [0, py])
    return line,

ani = animation.FuncAnimation(fig, update, frames=total_frames, interval=(10000 / total_frames))
plt.show()
