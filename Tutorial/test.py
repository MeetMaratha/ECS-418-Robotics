import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from sympy import Point, Polygon

def _getDistance(point1 : list or tuple, point2 : list or tuple) -> float:
    return np.sqrt( (point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2 )

def findFoot(a, b, c, x1, y1):
    temp = (-1 * (a * x1 + b * y1 + c) //
                  (a * a + b * b))
    x = temp * a + x1
    y = temp * b + y1
    return (x, y)

w0 = [0.2, 0.2]
w1 = [2, 3]
p = [0.3, 0.1]
psi = -1.57
eps = 0.01
delta = 0.001
k = 0.1
u_max = 0.01
xs = []
ys = []
v = 0.001
dt = 0.1
theta = np.arctan2(w1[1] - w0[1], w1[0] - w0[0])
m, c = (w1[1] - w0[1])/(w1[0] - w0[0]), (w0[0]*w1[1] - w1[0]*w0[1])/(w0[0]-w1[0])
while _getDistance(p, w1) > eps : 
    d = findFoot(m, -1, c, p[0], p[1])
    x_vtp, y_vtp = p[0] + delta * np.cos(theta), p[1] + delta * np.sin(theta)
    psi_desired = np.arctan2(y_vtp - p[1], x_vtp - p[0])
    u = k * (psi_desired - psi)
    psi = psi + u * dt
    p[0] = p[0] + v * np.cos(psi) * dt
    p[1] = p[1] + v * np.sin(psi) * dt
    xs.append(p[0])
    ys.append(p[1])
    if p[0] > w1[0] and p[1] > w1[1] : break

fig, ax = plt.subplots()
ax.plot([w0[0], w1[0]], [w0[1], w1[1]], color = 'red')
ax.plot(p[0], p[1], color = 'green', marker = 'o')
for i in range(len(xs)):
    x, y = xs[i], ys[i]
    ax.plot(x ,y, color = "black", marker = 'o')
plt.show()