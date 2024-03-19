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

#include <Eigen/Geometry>
#include <pinocchio/math/quaternion.hpp>
#include "pinocchio/math/rpy.hpp"
#include "pinocchio/spatial/se3.hpp"

#include "types.h"
#include "settings.h"
#include "collision_checker.h"
#include "contact_data.h"
#include "observer.h"
#include "logger.h"
#include "custom_planner.h"
#include "quadruped_task.h"

// thread_local Task* QuadrupedTask::task_ = nullptr;

class MujocoSimulator {
public:
  MujocoSimulator(int n_threads, bool rendering, bool loggin, const char *modelFile);
  ~MujocoSimulator();

  void initialize_viewer();
  void update_viewer();
  void put_robot_on_floor(int n_steps, VectorXd qref);

  /**
   * @brief Reset the environment.
   *
   * @param q0 Inital config x6 [x,y,z,r,p,y]
   */
  void reset(std::vector<double> q0);
  void runSimulation(int numSteps);
  void step(std::vector<double> actions);
  static void sensor(const mjModel *model, mjData *data, int stage);
  void save_logger(const std::string &fileName);
  void print_planner_timings();
  void update_ref_curve(std::vector<double> points);
  void reset_task(std::vector<double> q);
  void update_goal_position(std::vector<double> q);
  ObserverData getObervation();
  Data getLoggerData();

private:
  // Impossible to get a member thread_local specified only at runtime.
  // Hence, using this tool to flag if thread_only is activated.
  // bool flag_thread_local = true;
  // inline thread_local static QuadrupedTask* task_;
  bool flag_thread_local = false;
  inline static QuadrupedTask* task_;

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
  bool is_viewer_init = false;

  // Simulation parameters.
  double simstart;
  mjpc::ThreadPool plan_pool;
  int n_iteration = 0;
  int k_mpc_ = 0;

  std::vector<std::string> foot_names_;
  ContactData mcontactData;
  Observer observer;

  Logger logger_;

  mjpc::State state_;
  CustomiLQGPlanner planner;
  CollisionChecker col;

  // Goal visualisation
  double vz_size[3] = {0.15};
  double vz_pos[3] = {0.,0.,-0.15};
  float vz_color[4] = {1.,0.8,0.2,0.4};
};

#endif // MUJOCO_SIMULATOR_H