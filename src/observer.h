#ifndef OBSERVER_H
#define OBSERVER_H

#include <iostream>
#include "mjpc/utilities.h"
#include "mujoco/mujoco.h"

struct Observer {
  std::vector<std::string> foot_names;

  // Final observation
  std::array<double, 3> end_pos;
  std::array<double, 3> end_vel;
  std::array<double, 3> end_acc;
  std::array<double, 4> end_quat;
  std::array<double, 3> end_angVel;

  std::unordered_map<std::string, std::array<double, 3>> feet_pos;
  std::unordered_map<std::string, std::array<double, 3>> feet_vel;

  // Constructor to initialize arrays with zeros

  Observer(const std::vector<std::string> &foot_names)
      : end_pos({0.0, 0.0, 0.0}),
        end_vel({0.0, 0.0, 0.0}),
        end_acc({0.0, 0.0, 0.0}),
        end_quat({1., 0.0, 0.0, 0.0}),
        end_angVel({0.0, 0.0, 0.0}) {
    for (const auto &name : foot_names) {
      feet_vel[name] = {0., 0., 0.};
      feet_pos[name] = {0., 0., 0.};
    }
  }

  void update(const mjModel *model, const mjData *data) {
    // Fill in end_pos, end_vel, end_acc, end_quat, end_ang
    for (int i = 0; i < 3; ++i) {
      end_pos[i] = data->qpos[i];
      end_vel[i] = data->qvel[i];
      end_acc[i] = data->qacc[i];
      end_angVel[i] = data->qvel[i + 3];
    }

    // Fill in end_quat (quaternion)
    for (int i = 0; i < 4; ++i) {
      end_quat[i] = data->qpos[3 + i];
    }

    for (auto& elem : feet_pos){
      double *pos = mjpc::SensorByName(model, data, elem.first);
      elem.second[0] = pos[0];
      elem.second[1] = pos[1];
      elem.second[2] = pos[2];
    }

    for (auto& elem : feet_vel){
      double *vel = mjpc::SensorByName(model, data, elem.first + "_vel");
      elem.second[0] = vel[0];
      elem.second[1] = vel[1];
      elem.second[2] = vel[2];
    }
  }
};

#endif  // OBSERVER_H
