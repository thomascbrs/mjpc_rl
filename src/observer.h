#ifndef OBSERVER_H
#define OBSERVER_H

#include <iostream>
#include "mjpc/utilities.h"
#include "mujoco/mujoco.h"

// Define ObserverData outside the Observer class
struct ObserverData {
  std::vector<std::string> foot_names;
  std::array<double, 3> end_pos = {0.0, 0.0, 0.0};
  std::array<double, 3> end_vel = {0.0, 0.0, 0.0};
  std::array<double, 3> end_acc = {0.0, 0.0, 0.0};
  std::array<double, 4> end_quat = {0.0, 0.0, 0.0};
  std::array<double, 3> end_angVel = {0.0, 0.0, 0.0};
  std::unordered_map<std::string, std::array<double, 3>> feet_pos;
  std::unordered_map<std::string, std::array<double, 3>> feet_vel;
};

class Observer {
 private:
  ObserverData odata_;

 public:
  Observer(const std::vector<std::string> &foot_names) : odata_(){
    for (const auto &name : foot_names) {
      odata_.feet_vel[name] = {0., 0., 0.};
      odata_.feet_pos[name] = {0., 0., 0.};
    }
  }

  ObserverData getObervation() { return odata_; }

  void update(const mjModel *model, const mjData *data) {
    // Fill in end_pos, end_vel, end_acc, end_quat, end_ang
    for (int i = 0; i < 3; ++i) {
      odata_.end_pos[i] = data->qpos[i];
      odata_.end_vel[i] = data->qvel[i];
      odata_.end_acc[i] = data->qacc[i];
      odata_.end_angVel[i] = data->qvel[i + 3];
    }

    // Fill in end_quat (quaternion)
    for (int i = 0; i < 4; ++i) {
      odata_.end_quat[i] = data->qpos[3 + i];
    }

    for (auto &elem : odata_.feet_pos) {
      double *pos = mjpc::SensorByName(model, data, elem.first);
      elem.second[0] = pos[0];
      elem.second[1] = pos[1];
      elem.second[2] = pos[2];
    }

    for (auto &elem : odata_.feet_vel) {
      double *vel = mjpc::SensorByName(model, data, elem.first + "_vel");
      elem.second[0] = vel[0];
      elem.second[1] = vel[1];
      elem.second[2] = vel[2];
    }
  }
};

#endif  // OBSERVER_H
