#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include <mujoco/mjvisualize.h>
#include <vector>

#include "mjpc/planners/ilqg/planner.h"
#include "mjpc/states/state.h"
#include "mjpc/tasks/quadruped/quadruped.h"
#include "mjpc/threadpool.h"

class MujocoSimulator {
public:
  MujocoSimulator(const char *modelFile);
  ~MujocoSimulator();

  void initialize();
  void runSimulation(int numSteps);
  static void sensor(const mjModel *model, mjData *data, int stage);
  void PlanIteration(mjpc::ThreadPool *pool);
  // void mycontroller(const mjModel* m, mjData* d);
  std::vector<std::vector<double>> getLoggedJointPositions() const;

private:
  mjModel *model;
  mjData *data;
  mjpc::State state_;
  mjvCamera cam;  // abstract camera
  mjvOption opt;  // visualization options
  mjvScene scn;   // abstract scene
  mjrContext con; // custom GPU context

  GLFWwindow *window;

  // Define PD controller parameters
  double kp_ = 5.;  // Proportional gain
  double kd_ = 0.3; // Derivative gain
  std::vector<double> q0_;
  int steps_;

  std::vector<std::vector<double>> jointPositionsLog;
};