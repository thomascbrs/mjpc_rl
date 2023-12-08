from build.libmjpc_rl_pywrap import MujocoSimulator
import example_robot_data
import numpy as np
from time import sleep

# robot = example_robot_data.load("a1")
# robot.initViewer(windowName="mjpc_rl", loadModel=False)
# robot.loadViewerModel(rootNodeName="robot")

simulator = MujocoSimulator("/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/unitree_a1/a1_simu.xml")
simulator.run_simulation(1000)
qs = simulator.get_logged_joint_positions()
q_tmp = np.zeros(19)

# for i,q in enumerate(qs):
#     q_tmp[:] = q[:]
#     q_tmp[3] = q[4]
#     q_tmp[4] = q[5]
#     q_tmp[5] = q[6]
#     q_tmp[6] = q[3]
#     robot.display(q_tmp)
#     sleep(0.005)