### MJPC-RL

Project using mujoco and mujoco_mpc for high-level planning strategies.


##### Custom Mujoco_mpc

https://github.com/thomascbrs/mujoco_mpc/tree/topic-devel


##### New cost :
```
<!-- Params for fly-high : 3 -->
<!-- [0] : gamma factor -->
<!-- [1] : Front legs factor  -->
<!-- [2] : Back  legs  factor -->
<user name="Fly-high" dim="12" user="9 0.0 0.0 1.0 90. 1. 0.5" />
```

##### Python interface
Only working on release mode.

From root directory. Plotting tools :
```
python3 scripts/plots.py
```

##### TODO list

Explore new costs on the OCP formulation:
- Penalize contact forces
- Penalize feet acceleration
- Air time cost
- Symmetric cost

Improve plots and debug:
- Add references curves on plot (velocity/angular)
- Print MPC solutions on the graphs.
