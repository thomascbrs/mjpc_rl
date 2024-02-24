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
// #include "mjpc/tasks/quadruped/quadruped.h"
// #include "mjpc/tasks/cartpole/cartpole.h"
#include "contact_data.h"
#include "logger.h"
#include "custom_planner.h"

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
  std::vector<std::vector<double>> getLoggedJointPositions() const;
  void save_logger(const std::string &fileName);
  void print_planner_timings();
  void update_ref_curve(int idx_nn);
  void update_ref_curve(std::vector<double> points);

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
  Eigen::Matrix<double,19,1 > q0_;
  std::vector<double> terms_;
  bool allocate_enabled;
  bool plan_enabled;
  int count_;
  double agent_compute_time_ = 0.;

  bool RENDERING_;
  bool LOGGING_;

  // Simulation parameters.
  int planner_threads_;
  double horizon_;
  double timestep_;           // simulation timestep.
  double timestep_planner_;   // planner timestep.
  int kMaxTrajectoryHorizon_; // maximum lenght trajectory.
  int steps_;
  double simstart;
  mjpc::ThreadPool plan_pool;
  int n_iteration = 0;
  int num_trajectory_ = 0;

  std::vector<Vector6d> list_points;
  int idx_nn_;

  // residual function for the active task, updated once per planning iteration
  std::unique_ptr<mjpc::ResidualFn> residual_fn_;

  std::vector<std::vector<double>> jointPositionsLog;

  std::vector<std::string> foot_names_;
  ContactData mcontactData;

  Logger logger_;
};

#endif // MUJOCO_SIMULATOR_H