#ifndef LOGGER_H
#define LOGGER_H

#include "mjpc/trajectory.h"
#include "mjpc/utilities.h"
#include "mujoco/mujoco.h"
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <vector>

#include "contact_data.h"
#include "filter.h"
#include "types.h"
#include "utils.h"

struct Data {
  int size = 0;      // Usefull for loading.
  int horizon;       // Number of steps for each OCP. horizon = npsteps * dt_mpc
  int k_mpc;         // Number of iteration between MPCs.
  int mpc_iteration; //  Number of iteration done by the mpc.
  double dt_mpc;
  double dt_simu;
  std::vector<std::array<double, 19>>
      qpos; // qpos of the CoM/Trunk in world frame.
  std::vector<std::array<double, 6>>
      qpos_fil; // qpos of the CoM/Trunk in world frame.
  std::vector<std::array<double, 18>>
      qvel; // qvel of the CoM/Trunk in world frame.
  std::vector<std::array<double, 6>>
      qvel_fil; // qpos of the CoM/Trunk in world frame.
  std::unordered_map<std::string, std::vector<int>> foot_status;
  std::unordered_map<std::string, std::vector<int>> foot_status_touch;
  std::unordered_map<std::string, std::vector<std::array<double, 3>>>
      foot_position;
  std::unordered_map<std::string, std::vector<std::array<double, 3>>>
      foot_velocity;
  std::unordered_map<std::string, std::vector<std::array<double, 3>>>
      contact_forces;
  std::unordered_map<std::string, std::vector<std::array<double, 3>>>
      contact_forces_sensors;
  std::vector<std::vector<std::array<double, 37>>> mpc_traj;
};

class Logger {
public:
  Logger();
  void Initialize(const std::vector<std::string> &foot_names,
                  const double dt_mpc, const int horizon_mpc, const int k_mpc,
                  const double dt_simu);
  ~Logger();

  void log(const mjModel *model, mjData *data, const ContactData *mcontactData);

  void logState(const mjModel *model, mjData *data);

  void
  logFeetStatus(const std::unordered_map<std::string, int> &contact_status);
  void
  logFeetForces(const std::unordered_map<std::string, std::array<double, 3>>
                    &contact_forces);
  void logFeetForcesSensors(
      const std::unordered_map<std::string, std::array<double, 3>>
          &contact_forces_sensors);
  void logFeetPosition(const mjModel *model, mjData *data);
  void logFeetVelocity(const mjModel *model, mjData *data);
  void logFeetTouch(const mjModel *model, mjData *data);
  void logMPC(const mjpc::Trajectory *trajectory);

  void writeToCsvFile(const std::string &fileName);
  void saveData(const std::string &fileName);
  Data loadData(const std::string &fileName);
  Data getData(){return data_;};

private:
  Data data_;
  std::vector<std::string> foot_names_;

  std::vector<double> cutoff_pos = {2., 2., 2., 2., 2., 2.};
  double fs_pos = 1 / 0.002;
  int order_pos = 1;
  std::vector<double> cutoff_vel = {2., 2., 2., 2., 2., 2.};
  double fs_vel = 1 / 0.002;
  int order_vel = 1;
  Filter filter_pos_;
  Filter filter_vel_;
};

#endif // LOGGER_H
