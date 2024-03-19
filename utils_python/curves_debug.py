import matplotlib.pyplot as plt

#importing the bezier curve class
from ndcurves import bezier,exact_cubic, curve_constraints, polynomial
from ndcurves.optimization import (problem_definition, setup_control_points)
from ndcurves.optimization import constraint_flag
import quadprog

import numpy as np

def update_coeffs_rebaseVel(curve, new_pt, T, T2):
    assert len(new_pt) == 1, "point should be size 3"

    # Velocity space
    m = np.array([[1, 0., 0.], [0.,1.,0.], [1.,(T2-T),(T2-T)**2]])
    minv = np.array([[1, 0., 0.], [0.,1.,0.], [-(T2-T)**-2,-(T2-T)**-1,(T2-T)**-2]])
    # minv = np.linalg.inv(m)
    b = np.matrix([curve(T), curve.derivate(T,1), new_pt])
    res = np.dot(minv, b)

    curve = polynomial(res.T, T, T2)
    return curve

def update_coeffs_rebaseAcc(curve, new_pt, T, T2):
    assert len(new_pt) == 1, "point should be size 3"

    # Accelerartion constraint
    a2 = curve(T)
    b2 =  curve.derivate(T,1)
    print("acc:",  curve.derivate(T,1))
    # c2 = (T*-2) * (new_pt - a2 - b2 * T)
    c2 = (new_pt - b2 ) / (2 * (T2-T))
    curve = polynomial(np.array([a2,b2,c2]).T, T, T2)

    return curve

def plot_curve_xVEl(curve,current_curve, T, title=False, color="b", marker="o"):
    ax = plt.subplot(3,1,1)
    X = [curve(t)[0] for t in T]
    ax.plot(T,X,"-", color=color)
    ax.plot(T[-1],current_curve[-1][0], color="r",marker=marker, markersize=3)
    ax.set_title("vx(t)") if title else None

    ax = plt.subplot(3,1,2)
    X = [curve.derivate( t,1)[0] for t in T]
    ax.plot(T,X,"-", color=color)
    # ax.plot(T[-1],current_curve[-1][0], color="r",marker="o", markersize=10)
    ax.set_title("ax(t)") if title else None

    ax = plt.subplot(3,1,3)
    X = [curve.derivate( t,2)[0] for t in T]
    ax.plot(T,X,"-", color=color)
    # ax.plot(T[-1],current_curve[-1][0], color="r",marker="o", markersize=10)
    ax.set_title("jx(t)") if title else None

def plot_curve_xACC(curve,current_curve, T, title=False, color="b", marker="o"):
    ax = plt.subplot(3,1,1)
    X = [curve(t)[0] for t in T]
    ax.plot(T,X,"-", color=color)
    ax.set_title("vx(t)") if title else None

    ax = plt.subplot(3,1,2)
    X = [curve.derivate( t,1)[0] for t in T]
    ax.plot(T,X,"-", color=color)
    ax.plot(T[-1],current_curve[-1][0], color="r",marker=marker, markersize=3)
    # ax.plot(T[-1],current_curve[-1][0], color="r",marker="o", markersize=10)
    ax.set_title("ax(t)") if title else None

    ax = plt.subplot(3,1,3)
    X = [curve.derivate( t,2)[0] for t in T]
    ax.plot(T,X,"-", color=color)
    # ax.plot(T[-1],current_curve[-1][0], color="r",marker="o", markersize=10)
    ax.set_title("jx(t)") if title else None


timings = [0.,0.4,0.8]
P0 = [0.]
P1 = [0.2]
P2 = [0.]

current_curve = [P0,P1,P2]
# added_points = [[0.5, 0.0, 0.],[1., 0.0, -0.],[0.2, 0.0, 0.],[0.6, 0.0, 0.] ]
added_points = [[0.5], [0.7], [1.1], [1.3], [1.5], [1.7], [1.9], [2.1], [2.3], [2.5], [2.7]]
curve =  polynomial(np.array(current_curve).T, 0. , 0.4)

plt.ion()
plt.figure()

# Create a color gradient from dark blue to light blue
color_gradient = np.linspace(0.2, 1., len(added_points))
delta = 0.
T = np.linspace(delta, delta + 0.4, 100)
color = plt.cm.Blues(color_gradient[0])
plot_curve_xVEl(curve, current_curve,T, True)

for i,point in enumerate(added_points):
    # print("--")
    delta += 0.4
    delta = np.around(delta, decimals=2)
    T = np.linspace(delta, delta + 0.400001, 100)
    # print("T[0], T[-1] : ", str([T[0], T[-1]]))
    # print("[curve.min(), curve.max()]", str([curve.min(), curve.max()]))
    curve = update_coeffs_rebaseVel(curve, point, T[0], T[-1])
    # print("[curve.min(), curve.max()]", str([curve.min(), curve.max()]))
    current_curve.pop(0)
    current_curve.append(point)

    # Adjust color based on the gradient
    color = plt.cm.Blues(color_gradient[i])

    plot_curve_xVEl(curve, current_curve,T, True,color)


plt.figure()

timings = [0.,0.4,0.8]
P0 = [0.]
P1 = [0.]
P2 = [0.]

current_curve = [P0,P1,P2]
# added_points = [[0.5, 0.0, 0.],[1., 0.0, -0.],[0.2, 0.0, 0.],[0.6, 0.0, 0.] ]
added_points = [[1.5], [1.5], [1.5], [1.5], [1.5], [0.], [0.], [0.], [0.], [0.], [0.]]
curve =  polynomial(np.array(current_curve).T, 0. , 0.4)

# Create a color gradient from dark blue to light blue
color_gradient = np.linspace(0.2, 1., len(added_points))
delta = 0.
T = np.linspace(delta, delta + 0.4, 100)
color = plt.cm.Blues(color_gradient[0])
plot_curve_xACC(curve, current_curve,T, True)

for i,point in enumerate(added_points):
    delta += 0.4
    delta = np.around(delta, decimals=2)
    T = np.linspace(delta, delta + 0.4000001, 100)
    print("T[0], T[-1] : ", str([T[0], T[-1]]))
    print("[curve.min(), curve.max()]", str([curve.min(), curve.max()]))
    curve = update_coeffs_rebaseAcc(curve, point, T[0], T[-1])
    print("[curve.min(), curve.max()]", str([curve.min(), curve.max()]))
    current_curve.pop(0)
    current_curve.append(point)

    # Adjust color based on the gradient
    color = plt.cm.Blues(color_gradient[i])

    plot_curve_xACC(curve, current_curve,T, True,color)
