// main.cpp
#include "mujoco_simulator.h"
#include <iostream>
#include <thread>

int main() {

  MujocoSimulator mjsimulator = MujocoSimulator(1,false,false,
      "/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/unitree_a1/task_hill.xml");
  // MujocoSimulator mjsimulator = MujocoSimulator(
  //     "/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/cartpole/task.xml");
  // mjsimulator.runSimulation(1000);
  // Add a 2-second sleep
  // std::this_thread::sleep_for(std::chrono::seconds(2));

  std::vector<std::vector<double>> list_points;
  std::vector<double> point;
  point = {0.5, 0.0, 0.0, 0.0, 0.0, 0.0};
  list_points.push_back(point);
  point = {0.7, 0.0, 0.0, 0.0, 0.0, 0.0};
  list_points.push_back(point);
  point = {1.1, 0.0, 0.0, 0.0, 0.0, 0.0};
  list_points.push_back(point);
  point = {1.3, 0.0, 0.0, 0.0, 0.0, 0.0};
  list_points.push_back(point);
  point = {1.5, 0.0, 0.0, 0.0, 0.0, 0.0};
  list_points.push_back(point);
  point = {1.7, 0.0, 0.0, 0.0, 0.0, 0.0};
  list_points.push_back(point);
  point = {1.9, 0.0, 0.0, 0.0, 0.0, 0.0};
  list_points.push_back(point);
  point = {2.1, 0.0, 0.0, 0.0, 0.0, 0.0};
  list_points.push_back(point);
  point = {2.3, 0.0, 0.0, 0.0, 0.0, 0.0};
  list_points.push_back(point);
  point = {2.5, 0.0, 0.0, 0.0, 0.0, 0.0};
  list_points.push_back(point);
  point = {2.7, 0.0, 0.0, 0.0, 0.0, 0.0};
  list_points.push_back(point);

  mjsimulator.step(list_points[0]);
  mjsimulator.step(list_points[1]);
  mjsimulator.step(list_points[2]);


  return 0;
}