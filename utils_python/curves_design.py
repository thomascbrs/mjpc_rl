import matplotlib.pyplot as plt

#importing the bezier curve class
from ndcurves import bezier, exact_cubic, curve_constraints, polynomial
from ndcurves.optimization import (problem_definition, setup_control_points)
from ndcurves.optimization import constraint_flag
import quadprog

import numpy as np


def update_coeffs(curve, new_pt, T=0.4):
    assert len(new_pt) == 3, "point should be size 3"
    a, b, c = curve.coeff().T
    a2 = a + b * T + c * (T**2)
    b2 = b + 2 * c * T
    c2 = (T**-2) * (new_pt - a2 - b2 * T)
    curve = polynomial(np.array([a2, b2, c2]).T, 0., 0.4)
    return curve


def update_coeffs_rebase(curve, new_pt, T, T2):
    assert len(new_pt) == 3, "point should be size 3"
    m = np.array([[1, 0., 0.], [0., 1., 0.], [1., (T2 - T), (T2 - T)**2]])
    minv = np.array([[1, 0., 0.], [0., 1., 0.], [-(T2 - T)**-2, -(T2 - T)**-1, (T2 - T)**-2]])
    # minv = np.linalg.inv(m)
    b = np.matrix([curve(T), curve.derivate(T, 1), new_pt])
    res = np.dot(minv, b)

    print("\n----")
    print(res.T)
    curve = polynomial(res.T, T, T2)
    return curve


def update_coeffs2(curve, new_pt, T=0.4):
    assert len(new_pt) == 3, "point should be size 3"
    a, b, c, d = [0., 0., 0., 0.]
    if curve.degree() == 2:
        a, b, c = curve.coeff().T
        d = 0.
    else:
        a, b, c, d = curve.coeff().T
    a2 = a + b * T + c * (T**2) + d * (T**3)
    b2 = b + 2 * c * T + 3 * d * (T**2)
    c2 = c + 3 * d * T
    d2 = (T**-3) * (new_pt - a2 - b2 * T - c2 * (T**2))
    curve = polynomial(np.array([a2, b2, c2, d2]).T, 0., 0.4)
    return curve


def plot_curve_x(curve, current_curve, T, title=False, color="b", marker="o"):
    ax = plt.subplot(3, 2, 1)
    X = [curve(t)[0] for t in T]
    ax.plot(T, X, "-", color=color)
    ax.plot(T[-1], current_curve[-1][0], color="r", marker=marker, markersize=3)
    ax.set_title("vx(t)") if title else None

    ax = plt.subplot(3, 2, 3)
    X = [curve.derivate(t, 1)[0] for t in T]
    ax.plot(T, X, "-", color=color)
    # ax.plot(T[-1],current_curve[-1][0], color="r",marker="o", markersize=10)
    ax.set_title("ax(t)") if title else None

    ax = plt.subplot(3, 2, 5)
    X = [curve.derivate(t, 2)[0] for t in T]
    ax.plot(T, X, "-", color=color)
    # ax.plot(T[-1],current_curve[-1][0], color="r",marker="o", markersize=10)
    ax.set_title("jx(t)") if title else None


def plot_curve_z(curve, current_curve, T, title=False, color="b"):
    ax = plt.subplot(3, 2, 2)
    X = [curve(t)[2] for t in T]
    ax.plot(T, X, "-", color=color)
    ax.plot(T[-1], current_curve[-1][2], color="r", marker="o", markersize=3)
    ax.set_title("vz(t)") if title else None

    ax = plt.subplot(3, 2, 4)
    X = [curve.derivate(t, 1)[2] for t in T]
    ax.plot(T, X, "-", color=color)
    # ax.plot(T[-1],current_curve[-1][0], color="r",marker="o", markersize=10)
    ax.set_title("az(t)") if title else None

    ax = plt.subplot(3, 2, 6)
    X = [curve.derivate(t, 2)[2] for t in T]
    ax.plot(T, X, "-", color=color)
    # ax.plot(T[-1],current_curve[-1][0], color="r",marker="o", markersize=10)
    ax.set_title("jz(t)") if title else None


timings = [0., 0.4, 0.8]
P0 = [0., 0.0, 0.]
P1 = [0., 0.0, 0.]
P2 = [0., 0.0, 0.]

current_curve = [P0, P1, P2]
added_points = [[0.5, 0.0, 0.], [1., 0.0, -0.], [0.2, 0.0, 0.], [0.6, 0.0, 0.]]
# added_points = [[0.5, 0.0, 0.]]
curve = polynomial(np.array(current_curve).T, 0., 0.4)

plt.ion()
plt.figure()

# Create a color gradient from dark blue to light blue
color_gradient = np.linspace(0.2, 1., len(added_points))
delta = 0.
T = np.linspace(delta, delta + 0.4, 100)
color = plt.cm.Blues(color_gradient[0])
plot_curve_x(curve, current_curve, T, True)
plot_curve_z(curve, current_curve, T, True)

for i, point in enumerate(added_points):
    delta += 0.4
    delta = np.around(delta, decimals=2)
    T = np.linspace(delta, delta + 0.4, 100)
    print("T[0], T[-1] : ", str([T[0], T[-1]]))
    print("[curve.min(), curve.max()]", str([curve.min(), curve.max()]))
    curve = update_coeffs_rebase(curve, point, T[0], T[-1])
    print("[curve.min(), curve.max()]", str([curve.min(), curve.max()]))
    current_curve.pop(0)
    current_curve.append(point)

    # Adjust color based on the gradient
    color = plt.cm.Blues(color_gradient[i])

    plot_curve_x(curve, current_curve, T, True, color)
    plot_curve_z(curve, current_curve, T, True, color)

# plt.figure()

# # Create a color gradient from dark blue to light blue

# n_points = 5
# Ntotal = 20
# color_gradient = np.linspace(0.15, 1., Ntotal)
# for k in range(Ntotal):
#     delta = 0.
#     T = np.linspace(delta, delta + 0.4, 100)
#     color = plt.cm.gist_rainbow(color_gradient[k])
#     plot_curve_x(curve, current_curve,T, True, color)
#     plot_curve_z(curve, current_curve,T, True, color)

#     point = np.zeros(3)
#     for i in range(n_points):
#         delta += 0.4
#         T = np.linspace(delta, delta + 0.4, 100)
#         a,b = -0.5, 0.5
#         point += a + (b-a)*np.random.rand(3)
#         # point += -1 + 2*np.random.rand(3)
#         curve = update_coeffs(curve, point, 0.4)
#         current_curve.pop(0)
#         current_curve.append(point)

#         # Adjust color based on the gradient
#         color = plt.cm.gist_rainbow(color_gradient[k])

#         plot_curve_x(curve, current_curve,T, True,color)
#         plot_curve_z(curve, current_curve,T, True, color)
