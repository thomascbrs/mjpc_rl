#ifndef MUJOCO_SIMULATOR_H
#define MUJOCO_SIMULATOR_H

#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include <mujoco/mjvisualize.h>
#include <vector>
#include <Eigen/Dense>

#include "mjpc/planners/gradient/planner.h"
#include "mjpc/planners/ilqg/planner.h"
#include "mjpc/planners/sampling/planner.h"
#include "mjpc/states/state.h"
#include "mjpc/threadpool.h"

#include "settings.h"
#include "collision_checker.h"
#include "contact_data.h"
#include "observer.h"
#include "logger.h"
#include "custom_planner.h"
#include "quadruped_task.h"

// thread_local Task* QuadrupedTask::task_ = nullptr;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::VectorXd VectorXd;

class MujocoSimulator {
public:
  MujocoSimulator(int n_threads, bool rendering, bool loggin, const char *modelFile);
  ~MujocoSimulator();

  void initialize_viewer();
  void update_viewer();
  void put_robot_on_floor(int n_steps, VectorXd qref);
  void reset(Eigen::VectorXd q0);
  void runSimulation(int numSteps);
  void step(std::vector<double> actions);
  static void sensor(const mjModel *model, mjData *data, int stage);
  void save_logger(const std::string &fileName);
  void print_planner_timings();
  void update_ref_curve(std::vector<double> points);
  ObserverData getObervation();

private:
  // Impossible to get a member thread_local specified only at runtime.
  // Hence, using this tool to flag if thread_only is activated.
  bool flag_thread_local = true;
  inline thread_local static QuadrupedTask* task_;
  // bool flag_thread_local = false;
  // inline static QuadrupedTask* task_;

  mjModel *model;
  mjData *data;
  mjvCamera cam;  // abstract camera
  mjvOption opt;  // visualization options
  mjvScene scn;   // abstract scene
  mjrContext con; // custom GPU context
  GLFWwindow *window;

  // Settings.
  Settings settings;
  Eigen::Matrix<double,19,1 > q0_;
  std::vector<double> terms_;
  bool allocate_enabled;
  bool plan_enabled;
  int count_;
  double agent_compute_time_ = 0.;

  bool RENDERING_;
  bool LOGGING_;

  // Simulation parameters.
  double simstart;
  mjpc::ThreadPool plan_pool;
  int n_iteration = 0;

  std::vector<std::string> foot_names_;
  ContactData mcontactData;
  Observer observer;

  Logger logger_;

  mjpc::State state_;
  CustomiLQGPlanner planner;
  CollisionChecker col;
};

#endif // MUJOCO_SIMULATOR_H