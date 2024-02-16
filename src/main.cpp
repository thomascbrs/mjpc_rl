// main.cpp
#include "mujoco_simulator.h"
#include <iostream>
#include <thread>

int main() {

  MujocoSimulator mjsimulator = MujocoSimulator(5,true,
      "/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/unitree_a1/task_hill.xml");
  // MujocoSimulator mjsimulator = MujocoSimulator(
  //     "/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/cartpole/task.xml");
  mjsimulator.runSimulation(1000);
  // Add a 2-second sleep
  // std::this_thread::sleep_for(std::chrono::seconds(2));
  return 0;
}