import example_robot_data
import pinocchio as pin
import numpy as np
from build_release.libmjpc_rl_pywrap import loadData
from copy import copy
from time import sleep

robot = example_robot_data.load("a1")
model = robot.model
mdata = model.createData()

robot.initViewer(windowName="mjpc_rl", loadModel=True)

data = loadData("/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/log/tmp.bin")

q0 = np.zeros(19)
q0[2] = 0.26
q0[6] = 1.

# robot.q0
# robot.display(q0)

for i, mpc_data in enumerate(data.mpc_traj):
    x = [state[:19] for state in mpc_data]
    for i, q in enumerate(x):
        q_tmp = copy(q)
        q_tmp[3:7] = q[4], q[5], q[6], q[3]
        q_tmp[7:] = robot.q0[7:] + q[7:]
        if i == 0:
            print(i)
            robot.display(np.array(q_tmp))
            sleep(0.1)
