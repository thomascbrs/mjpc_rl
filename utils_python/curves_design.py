import matplotlib.pyplot as plt

#importing the bezier curve class
from ndcurves import (bezier)
from ndcurves.optimization import (problem_definition, setup_control_points)
from ndcurves.optimization import constraint_flag
import quadprog

import numpy as np

# N_int = 20
# degree = 8
# n_constraints  = 5
# weight_pos = np.diag([1.] * 3 * (degree - n_constraints))
# weight_vel = np.diag([0.1] * 3 * (degree - n_constraints))
# dim = 3

# x0 = [0.,0.,0.]
# v0 = [0.,0.,0.]
# x1 = [0.,0.,0.]
# v1 = [0.,0.,0.]

# # Results
# res_size = dim * (degree + 1)
# res = []

# pD = problem_definition(dim)
# pD.degree = degree

# # Reduce the size of the problem by fixing intial and final points
# pD.init_pos = np.array([x0[0], x0[1], x0[2]]).T
# pD.init_vel = np.array([v0]).T
# pD.init_acc = np.array([[0., 0., 0.]]).T

# pD.end_pos = np.array([x1]).T
# pD.end_vel = np.array([v1]).T
# pD.end_acc = np.array([[0., 0., 0.]]).T

P0 = [0.007, 0., 0.243]
P1 = [1.25, 0., -0.6]
P2 = [1.25, 0., 3.]
P3 = [4., 0., 0.173]
curve = bezier(np.array([P0, P1, P2, P3]).T)

plt.ion()
plt.figure()
T = np.linspace(0., 1., 100)
X = [curve(t)[0] for t in T]
Z = [curve(t)[2] for t in T]
plt.plot(X, Z, "x-")
plt.plot(P0[0], P0[2], "o")
plt.plot(P1[0], P1[2], "o")
plt.plot(P2[0], P2[2], "o")
plt.plot(P3[0], P3[2], "o")

plt.figure()
T = np.linspace(0., 1., 100)
dt = 0.002
pitch = []
for t in T:
    if (t + dt) <= 1.:
        dy = curve(t + dt)[2] - curve(t)[2]
        dx = curve(t + dt)[0] - curve(t)[0]
        if dx != 0.:
            pitch.append(dy / dx)
        else:
            pitch.append(dy / dx)
    else:
        pitch.append(pitch[-1])

plt.plot(T, pitch, "x-")
