// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "quadruped_task.h"

#include <string>

#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/utilities.h"

std::string QuadrupedTask::XmlPath() const {
  return mjpc::GetModelPath("quadruped/task_hill.xml");
}

std::string QuadrupedTask::Name() const { return "Quadruped Task"; }

// --------------------- Residuals for quadruped task --------------------
//   Number of residuals: 4
//     Residual (0): position_z - average(foot position)_z - height_goal
//     Residual (1): position - goal_position
//     Residual (2): orientation - goal_orientation
//     Residual (3): control
//     Residual (4): Fly-hight cost
//   Number of parameters: 1
//     Parameter (1): height_goal
// -----------------------------------------------------------------------
void QuadrupedTask::ResidualFn::Residual(const mjModel* model,
                                         const mjData* data,
                                         double* residual) const {
  // ---------- Residual (0) ----------
  // standing height goal
  double height_goal = parameters_[0];

  // system's standing height
  double standing_height = mjpc::SensorByName(model, data, "position")[2];

  // average foot height
  double FRz = mjpc::SensorByName(model, data, "FR")[2];
  double FLz = mjpc::SensorByName(model, data, "FL")[2];
  double RRz = mjpc::SensorByName(model, data, "RR")[2];
  double RLz = mjpc::SensorByName(model, data, "RL")[2];
  double avg_foot_height = 0.25 * (FRz + FLz + RRz + RLz);

  residual[0] = (standing_height - avg_foot_height) - height_goal;

  // ---------- Residual (1) ----------
  // goal position
  const double* goal_position = data->mocap_pos;

  // system's position
  double* position = mjpc::SensorByName(model, data, "position");

  // position error
  mju_sub3(residual + 1, position, goal_position);

  // ---------- Residual (2) ----------
  // goal orientation
  double goal_rotmat[9];
  const double* goal_orientation = data->mocap_quat;
  mju_quat2Mat(goal_rotmat, goal_orientation);

  // system's orientation
  double body_rotmat[9];
  double* orientation = mjpc::SensorByName(model, data, "orientation");
  mju_quat2Mat(body_rotmat, orientation);

  mju_sub(residual + 4, body_rotmat, goal_rotmat, 9);

  // ---------- Residual (3) ----------
  mju_copy(residual + 13, data->ctrl, model->nu);

  // ---------- Residual (4) ----------
  double* FR = mjpc::SensorByName(model, data, "FR");
  double* FL = mjpc::SensorByName(model, data, "FL");
  double* RR = mjpc::SensorByName(model, data, "RR");
  double* RL = mjpc::SensorByName(model, data, "RL");
  double* FR_vel = mjpc::SensorByName(model, data, "FR_vel");
  double* FL_vel = mjpc::SensorByName(model, data, "FL_vel");
  double* RR_vel = mjpc::SensorByName(model, data, "RR_vel");
  double* RL_vel = mjpc::SensorByName(model, data, "RL_vel");
  double feet_position[12];

  feet_position[0] = FR_vel[0];
  feet_position[1] = FR_vel[1];
  feet_position[2] = FR[2];

  feet_position[3] = FL_vel[0];
  feet_position[4] = FL_vel[1];
  feet_position[5] = FL[2];

  feet_position[6] = RR_vel[0];
  feet_position[7] = RR_vel[1];
  feet_position[8] = RR[2];

  feet_position[9] = RL_vel[0];
  feet_position[10] = RL_vel[1];
  feet_position[11] = RL[2];

  // Copy in the residual.
  mju_copy(residual + 25, feet_position, 12);
}

// -------- Transition for quadruped task --------
//   If quadruped is within tolerance of goal ->
//   set goal to next from keyframes.
// -----------------------------------------------
void QuadrupedTask::TransitionLocked(mjModel* model, mjData* data) {
  // set mode to GUI selection
  if (mode > 0) {
    residual_.current_mode_ = mode - 1;
  } else {
    // ---------- Compute tolerance ----------
    // goal position
    const double* goal_position = data->mocap_pos;

    // goal orientation
    const double* goal_orientation = data->mocap_quat;

    // system's position
    double* position = mjpc::SensorByName(model, data, "position");

    // system's orientation
    double* orientation = mjpc::SensorByName(model, data, "orientation");

    // position error
    double position_error[3];
    mju_sub3(position_error, position, goal_position);
    double position_error_norm = mju_norm3(position_error);

    // orientation error
    double geodesic_distance =
        1.0 - mju_abs(mju_dot(goal_orientation, orientation, 4));

    // ---------- Check tolerance ----------
    double tolerance = 1.5e-1;
    if (position_error_norm <= tolerance && geodesic_distance <= tolerance) {
      // update task state
      residual_.current_mode_ += 1;
      if (residual_.current_mode_ == model->nkey) {
        residual_.current_mode_ = 0;
      }
    }
  }

  // ---------- Set goal ----------
  mju_copy3(data->mocap_pos, model->key_mpos + 3 * residual_.current_mode_);
  mju_copy4(data->mocap_quat, model->key_mquat + 4 * residual_.current_mode_);
}