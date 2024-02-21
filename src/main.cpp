// main.cpp
#include "mujoco_simulator.h"
#include <iostream>
#include <thread>

void my_function01(std::vector<double> vector){
  std::cout << "vector : " << vector[0] << std::endl;
}
void my_function02(std::vector<double> vector){
  std::cout << "vector2 : " << vector[0] << std::endl;
}
void my_function03(int step){
  int kk = 0;
  for (int i=0;i < step;i++){
    kk += i + 3*i;
  }
}

int main() {

  MujocoSimulator mjsimulator = MujocoSimulator(1,false,false,
      "/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/unitree_a1/task_hill.xml");
  // MujocoSimulator mjsimulator = MujocoSimulator(
  //     "/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/cartpole/task.xml");
  // mjsimulator.runSimulation(3000);
  // Add a 2-second sleep
  // std::this_thread::sleep_for(std::chrono::seconds(2));

  std::vector<std::vector<double>> list_points;
  std::vector<double> point;
  point = {0.5, 0.0, 0.0, 0.0, -0.05, 0.0};
  list_points.push_back(point);
  point = {0.5, 0.0, 0.0, 0.0, -0.1, 0.0};
  list_points.push_back(point);
  point = {0.8, 0.0, 0.0, 0.0, -0.1, 0.0};
  list_points.push_back(point);
  point = {0.8, 0.0, 0.0, 0.0, -0.1, 0.0};
  list_points.push_back(point);
  point = {0.8, 0.0, 0.0, 0.0, -0.1, 0.0};
  list_points.push_back(point);
  point = {0.8, 0.0, 0.0, 0.0, -0.1, 0.0};
  list_points.push_back(point);
  point = {0.1, 0.0, 0.0, 0.0, -0.1, 0.0};
  list_points.push_back(point);
  point = {0.1, 0.0, 0.0, 0.0, -0.1, 0.0};
  list_points.push_back(point);
  point = {0.1, 0.0, 0.0, 0.0, -0.07, 0.0};
  list_points.push_back(point);
  point = {0.1, 0.0, 0.0, 0.0, -0.07, 0.0};
  list_points.push_back(point);
  point = {0.1, 0.0, 0.0, 0.0, -0.07, 0.0};
  list_points.push_back(point);

  // ProfilerStart("test.prof"); //Start profiling section and save to file
  // HeapProfilerStart("output_inside.heap");
  // for (int k=0;k < 10000 ; k++){
  //   point = {2.7, 0.0, 0.0, 0.0, 0.0, 0.0};
  //   list_points.push_back(point);
  //   my_function01(point);
  //   my_function02(point);
  //   my_function03(10000);
  // }
  // ProfilerStop();
  // mjsimulator.step(list_points[0]);

  // mjsimulator.step(list_points[2]);
  // Start time before calling function1

  // Call function1
  for (int j = 0; j < 20   ; j++){
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << "j : " << j << std::endl;
    mjsimulator.step(list_points[j]);
    // End time after calling function1
    auto end = std::chrono::high_resolution_clock::now();
    // Calculate the duration taken by function1
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Step function [ms] : " << duration1.count()  << std::endl;
  }



  // ProfilerFlush();
  // delete &mjsimulator;
  // HeapProfilerDump("output_inside.heap");
  // HeapProfilerStop();


  return 0;
}