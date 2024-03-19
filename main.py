# from build_release.libmjpc_rl_pywrap import MujocoSimulator
from build_release.libmjpc_rl_pywrap import MujocoSimulator,loadData
import example_robot_data
import numpy as np
from time import sleep
from time import perf_counter as clock
import os

# robot = example_robot_data.load("a1")
# robot.initViewer(windowName="mjpc_rl", loadModel=False)
# robot.loadViewerModel(rootNodeName="robot")

# Path to the parameter file.
current_dir = os.path.dirname(os.path.abspath(__file__))
relative_path = "../mjpc_rl/unitree_a1/task_hill.xml"
filename = os.path.join(current_dir, relative_path)

simulator = MujocoSimulator(2, False, True, filename)
# simulator.reset([1.5,0.,0.3,0.,0.,0.])
# simulator.reset([0.,0.,0.3,0.,0.,1.3])
obs = simulator.getObervation()
print("lfeet : ", obs.lfeet_pos)

# list_points = [[0.5, 0.0, 0.0, 0.0, -0.1, 0.0], [0.5, 0.0, 0.0, 0.0, -0.15, 0.0], [0.5, 0.0, 0.0, 0.0, -0.15, 0.0],
#                [0.8, 0.0, 0.0, 0.0, -0.15, 0.0], [0.8, 0.0, 0.0, 0.0, -0.15, 0.0], [0.8, 0.0, 0.0, 0.0, -0.15, 0.0],
#                [0.8, 0.0, 0.0, 0.0, -0.15, 0.0], [0.0, 0.0, 0.0, 0.0, -0.15, 0.0], [0.0, 0.0, 0.0, 0.0, -0.15, 0.0],
#                [0.0, 0.0, 0.0, 0.0, -0.15, 0.0]]
list_points = [[0.5, 0., 0.0, 0.0, -0.1, 0.],
               [0.2, 0., 0.0, 0.0, -0., 0.],
               [0.2, 0., 0.0, 0.0, -0., 0.],
               [0.1, 0., 0.0, 0.0, -0., 0.],
               [0., 0., 0.0, 0.0, -0., 0.],
               [0., 0., 0.0, 0.0, -0., 0.],
               [0., 0., 0.0, 0.0, -0., 0.],
               [0.0, 0.0, 0.0, 0.0, -0., 0.],
               [0.0, 0.0, 0.0, 0.0, -0., 0.],
               [0.0, 0.0, 0.0, 0.0, -0., 0.]]

for point in list_points:
    t0 = clock()
    simulator.step(point)
    t1 = clock()
    print("Step function [ms] : ", 1000 * (t1 - t0))
    obs = simulator.getObervation()
    print("obs.lvref : " , obs.lvref)
    print("obs.orientation_ref : ", obs.orientation_ref)
    

simulator.step(list_points[0])
simulator.step(list_points[1])
simulator.step(list_points[2])

simulator.reset([1.5,0.,0.3,0.,0.,0.9])
obs = simulator.getObervation()
print(obs.feet_pos)
print(obs.lfeet_pos)
print("----\n ---")

simulator.reset([1.5,0.,0.3,0.,0.,0.1])
obs = simulator.getObervation()
print(obs.feet_pos)
print(obs.lfeet_pos)
print("----\n ---")

simulator.reset([1.5,0.,0.3,0.,0.,-0.9])
obs = simulator.getObervation()
print(obs.feet_pos)
print(obs.lfeet_pos)
# simulator.step(list_points[0])
# simulator.step(list_points[2])
# simulator.step(list_points[1])
# simulator.step(list_points[3])


current_dir = os.path.dirname(os.path.abspath(__file__))
relative_path = "../mjpc_rl/log/tmp.bin"
filename = os.path.join(current_dir, relative_path)
simulator.save_logger(filename)

data = loadData(filename)

# import matplotlib.pyplot as plt
# plt.ion()

# simulator.run_simulation(500)
# sleep(0.5)
# simulator.run_simulation(500)
# qs = simulator.get_logged_joint_positions()
# q_tmp = np.zeros(19)

# for i,q in enumerate(qs):
#     q_tmp[:] = q[:]
#     q_tmp[3] = q[4]
#     q_tmp[4] = q[5]
#     q_tmp[5] = q[6]
#     q_tmp[6] = q[3]
#     robot.display(q_tmp)
#     sleep(0.005)
