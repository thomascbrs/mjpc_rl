# from build_release.libmjpc_rl_pywrap import MujocoSimulator
from build_release.libmjpc_rl_pywrap import MujocoSimulator,loadData
import example_robot_data
import numpy as np
from time import sleep
from time import perf_counter as clock
import matplotlib.pyplot as plt
plt.ion()

# robot = example_robot_data.load("a1")
# robot.initViewer(windowName="mjpc_rl", loadModel=False)
# robot.loadViewerModel(rootNodeName="robot")

simulator = MujocoSimulator(1, False, True, "/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/unitree_a1/task_hill.xml")

list_points = [[0.5, 0.0, 0.0, 0.0, -0.1, 0.0], [0.5, 0.0, 0.0, 0.0, -0.15, 0.0], [0.5, 0.0, 0.0, 0.0, -0.15, 0.0],
               [0.8, 0.0, 0.0, 0.0, -0.15, 0.0], [0.8, 0.0, 0.0, 0.0, -0.15, 0.0], [0.8, 0.0, 0.0, 0.0, -0.15, 0.0],
               [0.8, 0.0, 0.0, 0.0, -0.15, 0.0], [0.0, 0.0, 0.0, 0.0, -0.15, 0.0], [0.0, 0.0, 0.0, 0.0, -0.15, 0.0],
               [0.0, 0.0, 0.0, 0.0, -0.15, 0.0]]

for point in list_points:
    t0 = clock()
    simulator.step(point)
    t1 = clock()
    print("Step function [ms] : ", 1000 * (t1 - t0))

obs = simulator.getObervation()

filename = "/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/log/tmp.bin"
simulator.save_logger(filename)

data = loadData(filename)

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
