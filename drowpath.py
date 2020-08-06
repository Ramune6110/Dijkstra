import numpy as np
import matplotlib.pyplot as plt

x, y = np.loadtxt("path.txt", comments='!', unpack=True)

sx = 5.0  # [m]
sy = 5.0  # [m]
gx = 50.0  # [m]
gy = 50.0  # [m]

ox, oy = [], []
for i in range(-10, 60):
    ox.append(i)
    oy.append(-10.0)
for i in range(-10, 60):
    ox.append(60.0)
    oy.append(i)
for i in range(-10, 61):
    ox.append(i)
    oy.append(60.0)
for i in range(-10, 61):
    ox.append(-10.0)
    oy.append(i)
for i in range(-10, 40):
    ox.append(20.0)
    oy.append(i)
for i in range(0, 40):
    ox.append(40.0)
    oy.append(60.0 - i)

plt.plot(x, y, color='red', linewidth = 1.5, label = "path")
plt.plot(ox, oy, ".k")
plt.plot(sx, sy, "og")
plt.plot(gx, gy, "xb")
plt.xlabel("x")
plt.ylabel("y")
plt.legend(loc="upper left")
plt.show()