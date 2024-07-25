// main.cpp
#include "mujoco_simulator.h"
#include <filesystem> // C++17
#include <fstream>
#include <iostream>
#include <thread>

namespace fs = std::filesystem;

void my_function01(std::vector<double> vector) {
  std::cout << "vector : " << vector[0] << std::endl;
}
void my_function02(std::vector<double> vector) {
  std::cout << "vector2 : " << vector[0] << std::endl;
}
void my_function03(int step) {
  int kk = 0;
  for (int i = 0; i < step; i++) {
    kk += i + 3 * i;
  }
}

int main() {

  // Get the current directory
  fs::path current_dir = fs::current_path();

  // Construct the relative path
  fs::path relative_path = "../../mjpc_rl/unitree_a1/task_hill.xml";

  // Construct the absolute path
  fs::path filename = current_dir / relative_path;

  MujocoSimulator mjsimulator =
      MujocoSimulator(2, true, false, filename.c_str());
  // mjsimulator.runSimulation(3000);
  // Add a 2-second sleep
  // std::this_thread::sleep_for(std::chrono::seconds(2));

  // Intereseting behaviour. Warmstart MPC ? Run 2 times
  // and different behaviour.
  std::vector<double> q0 = {0., 14., 0.5, 0., 0., 0.};
  mjsimulator.reset(q0, 2);
  // mjsimulator.set_mpc_params(2,2,4,10);
  mjsimulator.set_mpc_params(1,2,5,18);
  std::vector<std::vector<double>> list_points;
  std::vector<double> point;
  point = {0.5, 0.0, 0.0, 0.0, -0.1, 0.0};
  list_points.push_back(point);
  point = {0.4, 0.0, 0.0, 0.0, -0., 0.1};
  list_points.push_back(point);
  point = {0.4, 0.0, 0.0, 0.0, -0., 0.2};
  list_points.push_back(point);
  point = {0.4, 0.0, 0.0, 0.0, -0., 0.3};
  list_points.push_back(point);
  point = {0., 0.0, 0.0, 0.0, -0., 0.4};
  list_points.push_back(point);
  point = {0., 0.0, 0.0, 0.0, -0., 0.5};
  list_points.push_back(point);
  point = {0., 0.0, 0.0, 0.0, -0., 0.5};
  list_points.push_back(point);
  point = {0., 0.0, 0.0, 0.0, -0., 0.0};
  list_points.push_back(point);
  point = {0., 0.0, 0.0, 0.0, -0., 0.0};
  list_points.push_back(point);
  point = {0., 0.0, 0.0, 0.0, -0., 0.0};
  list_points.push_back(point);
  point = {0., 0.0, 0.0, 0.0, -0., 0.0};
  list_points.push_back(point);
  point = {0., 0.0, 0.0, 0.0, -0., 0.0};
  list_points.push_back(point);
  point = {0., 0.0, 0.0, 0.0, -0., 0.0};
  list_points.push_back(point);
  point = {0., 0.0, 0.0, 0.0, -0., 0.0};
  list_points.push_back(point);

  // std::vector<double> q0 = {0., 22., 0.5, 0., 0., 0.};
  // mjsimulator.reset(q0, 4);
  // mjsimulator.set_mpc_params(1,2,5,15);
  // std::vector<std::vector<double>> list_points;
  // std::vector<double> point;
  // point = {0.208, 0.0, 0.0, 0.0, -0.1, 0.0};
  // list_points.push_back(point);
  // point = {0.17, 0.0, 0.0, 0.0, -0.1, 0.1};
  // list_points.push_back(point);
  // point = {0.33, 0.0, 0.0, 0.0, -0., 0.2};
  // list_points.push_back(point);
  // point = {0.2, 0.0, 0.0, 0.0, -0., 0.};
  // list_points.push_back(point);
  // point = {0.17, 0.0, 0.0, 0.0, -0., 0.};
  // list_points.push_back(point);
  // point = {0.17, 0.0, 0.0, 0.0, -0., 0.};
  // list_points.push_back(point);
  // point = {0.17, 0.0, 0.0, 0.0, -0., 0.};
  // list_points.push_back(point);
  // point = {0., 0.0, 0.0, 0.0, -0., 0.};
  // list_points.push_back(point);
  // point = {0., 0.0, 0.0, 0.0, -0., 0.};
  // list_points.push_back(point);
  // for (int i=0;i < 50;i++){
  //   point = {0., 0.0, 0.0, 0.0, -0., 0.};
  //   list_points.push_back(point);
  // }

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
  mjsimulator.step(list_points[0]);
  stateNode node0 = mjsimulator.get_node();

  mjsimulator.step(list_points[1]);
  mjsimulator.set_node(node0);
  // mjsimulator.set_node(node0);
  // mjsimulator.set_node(node0);
  std::cout << "ICI ok" << std::endl;

  mjsimulator.step(list_points[2]);
  stateNode node1 = mjsimulator.get_node();

  mjsimulator.set_node(node1);
  std::cout << "ICI2 ok" << std::endl;

  mjsimulator.step(list_points[3]);
  mjsimulator.set_node(node1);

  // Save the data.
  fs::path relative_path_logger = "../../mjpc_rl/logs/logger/ldata.bin";
  fs::path filename_logger = current_dir / relative_path_logger;
  mjsimulator.save_logger(filename_logger);

  // ProfilerFlush();
  // delete &mjsimulator;
  // HeapProfilerDump("output_inside.heap");
  // HeapProfilerStop();

  return 0;
}