#ifndef OBSERVER_H
#define OBSERVER_H

#include <iostream>
#include "mjpc/utilities.h"
#include "mujoco/mujoco.h"

#include "types.h"
#include "utils.h"

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
  std::unordered_map<std::string, std::array<double, 3>> lfeet_pos;
  std::unordered_map<std::string, std::array<double, 3>> lfeet_vel;
};

class Observer {
 private:
  ObserverData odata_;
  Matrix3d R_tmp;
  Vector3d pos_tmp;
  const mjtNum axis[3];  // z-axis (yaw)

 public:
  Observer(const std::vector<std::string> &fnames)
      : odata_(),
        R_tmp(Matrix3d::Identity()),
        pos_tmp(Vector3d::Zero()),
        axis{0., 0., 1.} {
    for (const auto &name : fnames) {
      odata_.foot_names.push_back(name);
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

    // Compute in local frame.
    pos_tmp[0] = odata_.end_pos[0];
    pos_tmp[1] = odata_.end_pos[1];
    pos_tmp[2] = odata_.end_pos[2];
    // Only considering position and yaw axis.

    mjtNum R_data[9];
    mjtNum quat_tmp[4];
    mju_mulQuatAxis(quat_tmp, &data->qpos[3],
                    axis);           // Convert axis-angle to quaternion
    mju_quat2Mat(R_data, quat_tmp);  // Convert quaternion to rotation matrix
    updateMatrix(R_tmp, R_data);

    for (const auto &name : odata_.foot_names) {
      // Foot position in local frame
      Vector3d vec_tmp;
      vec_tmp[0] = odata_.feet_pos[name][0];
      vec_tmp[1] = odata_.feet_pos[name][1];
      vec_tmp[2] = odata_.feet_pos[name][2];
      Vector3d res;
      res = (R_tmp.transpose() * (vec_tmp - pos_tmp)).array();
      odata_.lfeet_pos[name][0] = res[0];
      odata_.lfeet_pos[name][1] = res[1];
      odata_.lfeet_pos[name][2] = res[2];

      // Foot velocity in local frame
      vec_tmp(0) = odata_.feet_vel[name][0];
      vec_tmp(1) = odata_.feet_vel[name][1];
      vec_tmp(2) = odata_.feet_vel[name][2];
      res = (R_tmp.transpose() * (vec_tmp - pos_tmp)).array();
      odata_.lfeet_vel[name][0] = res[0];
      odata_.lfeet_vel[name][1] = res[1];
      odata_.lfeet_vel[name][2] = res[2];
    }
  }
};

#endif  // OBSERVER_H
