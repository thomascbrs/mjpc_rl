// main.cpp
#include "mujoco_simulator.h"
#include <iostream>

int main() {

  MujocoSimulator mjsimulator = MujocoSimulator(
      "/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/unitree_a1/a1_simu.xml");
  mjsimulator.runSimulation(1000);
  return 0;
}