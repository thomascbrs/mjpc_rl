#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include <mujoco/mjvisualize.h>
#include <vector>

#include "mjpc/planners/gradient/planner.h"
#include "mjpc/planners/ilqg/planner.h"
#include "mjpc/planners/sampling/planner.h"
#include "mjpc/states/state.h"
// #include "mjpc/tasks/quadruped/quadruped.h"
// #include "mjpc/tasks/cartpole/cartpole.h"
#include "logger.h"
#include "mjpc/threadpool.h"

struct ContactData {
  // Naming informations.
  std::vector<std::string> foot_names;
  std::string force_suffix;
  std::unordered_map<std::string, std::string> foot_site_names;

  // Informations dictionary.
  std::unordered_map<std::string, int> contact_status;
  std::unordered_map<std::string, std::array<double, 3>> contact_forces;
  std::unordered_map<std::string, std::array<double, 3>> contact_forces_sensors;

  // Constructor to initialize the struct
  ContactData(const std::vector<std::string> &foot_names)
      : foot_names(foot_names) {
    // TODO: Find a better way to initialize this. Modify .xml ?
    force_suffix = "_force";
    foot_site_names = {{"FR", "FR"}, {"FL", "FL"}, {"HR", "RR"}, {"HL", "RL"}};
    for (const auto &name : foot_names) {
      contact_status[name] = 0;
      contact_forces[name] = {0., 0., 0.};
      contact_forces_sensors[name] = {0., 0., 0.};
    }
  }

  void reset() {
    for (auto &status : contact_status) {
      status.second = 0;
    }
    for (auto &forces : contact_forces) {
      forces.second = {0., 0., 0.};
    }
    for (auto &forces : contact_forces_sensors) {
      forces.second = {0., 0., 0.};
    }
  }

  void update_force_sensors(const mjModel *model, mjData *data) {
    for (const auto &name : foot_names) {
      double *force = mjpc::SensorByName(model, data, name + force_suffix);

      int siteID = mj_name2id(model, mjOBJ_SITE, foot_site_names[name].c_str());
      int parentBodyIndex = model->site_bodyid[siteID];

      // Get the local position and orientation of the site
      const mjtNum *localPosition = model->site_pos + 3 * siteID;
      const mjtNum *localOrientation = model->site_quat + 4 * siteID;

      // Use mj_local2Global to get the global position and orientation
      mjtNum globalPosition[3];
      mjtNum globalOrientation[9];
      mjtNum vec[3];
      mj_local2Global(data, globalPosition, globalOrientation, localPosition,
                      localOrientation, parentBodyIndex, 0);
      mju_mulMatVec(vec, globalOrientation, force, 3, 3);

      // Update sensors dict.
      contact_forces_sensors[name][0] = -vec[0];
      contact_forces_sensors[name][1] = -vec[1];
      contact_forces_sensors[name][2] = -vec[2];
    }
  }

  void update_contact(const mjModel *model, mjData *data) {
    for (int contactIndex = 0; contactIndex < data->ncon; ++contactIndex) {
      const char *geomName0 =
          mj_id2name(model, mjOBJ_GEOM, data->contact[contactIndex].geom[0]);
      const char *geomName1 =
          mj_id2name(model, mjOBJ_GEOM, data->contact[contactIndex].geom[1]);

      auto it0 = std::find(foot_names.begin(), foot_names.end(), geomName0);
      auto it1 = std::find(foot_names.begin(), foot_names.end(), geomName1);

      if (it0 != foot_names.end() || it1 != foot_names.end()) {
        mjtNum mat[9], confrc[6], frc[3], vec[3];

        // mat = contact frame rotation matrix (normal along x)
        mju_transpose(mat, data->contact[contactIndex].frame, 3, 3);

        // get contact force:torque in contact frame
        mj_contactForce(model, data, contactIndex, confrc);

        // Get only the linear forces.
        mju_copy(frc, confrc, 3);

        mju_mulMatVec(vec, mat, frc, 3, 3);

        // Point from Geom[0] to Geom 1. Here Geom[0] is the foot.
        if (it0 != foot_names.end()) {
          mju_scl3(vec, vec, -1); // Geom[0] is foot.
          contact_status[geomName0] = 1;
          contact_forces[geomName0] = {vec[0], vec[1], vec[2]};
        } else {
          // Geom[0] is floor. Direction ok.
          contact_status[geomName1] = 1;
          contact_forces[geomName1] = {vec[0], vec[1], vec[2]};
        }
      }
    }
  }

  void update(const mjModel *model, mjData *data) {
    reset();
    update_contact(model, data);
    update_force_sensors(model, data);
  }
};

class MujocoSimulator {
public:
  MujocoSimulator(const char *modelFile);
  ~MujocoSimulator();

  void initialize();
  void runSimulation(int numSteps);
  void step();
  static void sensor(const mjModel *model, mjData *data, int stage);
  void PlanIteration(mjpc::ThreadPool *pool);
  void Plan(std::atomic<bool> &exitrequest, std::atomic<int> &uiloadrequest);
  // void mycontroller(const mjModel* m, mjData* d);
  std::vector<std::vector<double>> getLoggedJointPositions() const;
  void disableInteractionForGeoms(mjModel *m);
  void enableInteractionForGeoms(mjModel *m);

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
  std::vector<double> terms_;
  bool allocate_enabled;
  bool plan_enabled;
  int count_;
  double agent_compute_time_ = 0.;

  // Simulation parameters.
  int planner_threads_;
  double horizon_;
  double timestep_;           // simulation timestep.
  double timestep_planner_;   // planner timestep.
  int kMaxTrajectoryHorizon_; // maximum lenght trajectory.
  int steps_;

  std::vector<double> original_friction_values;
  std::vector<double> original_solref_values;

  // residual function for the active task, updated once per planning iteration
  std::unique_ptr<mjpc::ResidualFn> residual_fn_;

  std::vector<std::vector<double>> jointPositionsLog;

  std::vector<std::string> foot_names_;
  ContactData mcontactData;

  Logger logger_;
};