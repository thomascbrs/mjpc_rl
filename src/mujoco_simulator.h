#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include <mujoco/mjvisualize.h>
#include <vector>

#include "mjpc/planners/ilqg/planner.h"
#include "mjpc/states/state.h"
// #include "mjpc/tasks/quadruped/quadruped.h"
// #include "mjpc/tasks/cartpole/cartpole.h"
#include "mjpc/threadpool.h"

class MujocoSimulator {
public:
  MujocoSimulator(const char *modelFile);
  ~MujocoSimulator();

  void initialize();
  void runSimulation(int numSteps);
  static void sensor(const mjModel *model, mjData *data, int stage);
  void PlanIteration(mjpc::ThreadPool *pool);
  void Plan(std::atomic<bool>& exitrequest,std::atomic<int>& uiloadrequest);
  // void mycontroller(const mjModel* m, mjData* d);
  std::vector<std::vector<double>> getLoggedJointPositions() const;
  void disableInteractionForGeoms(mjModel* m);
  void enableInteractionForGeoms(mjModel* m);

private:
  mjModel *model;
  mjData *data;
  mjvCamera cam;  // abstract camera
  mjvOption opt;  // visualization options
  mjvScene scn;   // abstract scene
  mjrContext con; // custom GPU context

  // ----- iLQG planner ----- //
  // mjpc::iLQGPlanner planner;

  GLFWwindow *window;

  // Define PD controller parameters
  double kp_ = 5.;  // Proportional gain
  double kd_ = 0.2; // Derivative gain
  std::vector<double> q0_;
  int steps_;
  std::vector<double> terms_;
  bool allocate_enabled;
  bool plan_enabled;
  int count_;
  double agent_compute_time_ = 0.;
  int planner_threads_ = 2;

  std::vector<double> original_friction_values;
  std::vector<double> original_solref_values;


  // residual function for the active task, updated once per planning iteration
  std::unique_ptr<mjpc::ResidualFn> residual_fn_;

  std::vector<std::vector<double>> jointPositionsLog;
};