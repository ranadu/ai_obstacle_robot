import math
import time
import random
import matplotlib.pyplot as plt

# Robot state
x, y, theta = 0.0, 0.0, 0.0
dt = 0.05

# Obstacle
obs_x, obs_y = 1.0, 0.0

plt.ion()
fig, ax = plt.subplots()

while True:
    # Fake sensor (replace with serial later)
    dx = obs_x - x
    dy = obs_y - y
    distance = math.hypot(dx, dy) + random.uniform(-0.01, 0.01)

    # Same decision logic as firmware
    if distance < 0.3:
        v, w = 0.0, 1.5
    elif distance > 0.5:
        v, w = 0.2, 0.0
    else:
        v, w = 0.0, 0.0

    # Kinematics (3 lines, no lies)
    x += v * math.cos(theta) * dt
    y += v * math.sin(theta) * dt
    theta += w * dt

    ax.clear()
    ax.set_xlim(-1, 2)
    ax.set_ylim(-1, 1)

    ax.plot(obs_x, obs_y, "rs")
    ax.plot(x, y, "bo")
    ax.arrow(x, y, 0.2*math.cos(theta), 0.2*math.sin(theta), head_width=0.05)

    ax.set_title(f"distance = {distance:.2f} m")
    plt.pause(dt)