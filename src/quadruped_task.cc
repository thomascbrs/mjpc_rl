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

#include "mjpc/task.h"
#include "mjpc/utilities.h"
#include <mujoco/mujoco.h>

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
void QuadrupedTask::ResidualFn::Residual(const mjModel *model,
                                         const mjData *data,
                                         double *residual) const {

  int res_index = 0;
  // ---------- Residual (0) ----------
  // Fly-high cost.
  double *FR = mjpc::SensorByName(model, data, "FR");
  double *FL = mjpc::SensorByName(model, data, "FL");
  double *RR = mjpc::SensorByName(model, data, "RR");
  double *RL = mjpc::SensorByName(model, data, "RL");
  double *FR_vel = mjpc::SensorByName(model, data, "FR_vel");
  double *FL_vel = mjpc::SensorByName(model, data, "FL_vel");
  double *RR_vel = mjpc::SensorByName(model, data, "RR_vel");
  double *RL_vel = mjpc::SensorByName(model, data, "RL_vel");
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
  mju_copy(residual, feet_position, 12);
  res_index += 12;

  // Time varying references.
  if (data->time - 1. >= 0. && data->time - 1. <= 0.82) {
    Eigen::Vector3d pos_ref = curve_(data->time - 1.);
    Eigen::Vector3d vel_ref = curve_vel_(data->time - 1.);
    Eigen::Vector3d acc_ref = curve_acc_(data->time - 1.);

    // Compute derivative of the curve wrt to x to retrieve pitch angle.
    double dt = 0.01;
    double fwd = 0.02;
    double pitch = 0.;
    double factor = 0.4;
    if (data->time + fwd + dt - 1. <= 1.) {
      Eigen::Vector3d pos_ref_dt = curve_(data->time + dt - 1.);
      pitch = (pos_ref_dt[2] - pos_ref[2]) / (pos_ref_dt[0] - pos_ref[0]);
    } else {
      Eigen::Vector3d pos_ref_dt = curve_(data->time - dt - 1.);
      pitch = (pos_ref_dt[2] - pos_ref[2]) / (pos_ref_dt[0] - pos_ref[0]);
    }
    pitch *= -factor;

    mjtNum axis[3] = {0.0, 1.0, 0.0}; // Set y-axis
    mjtNum quat[4];
    mjtNum ref_rotmat[9];
    mju_axisAngle2Quat(quat, axis, pitch); // Convert axis-angle to quaternion
    mju_quat2Mat(ref_rotmat, quat); // Convert quaternion to rotation matrix

    // ---------- Residual (1) ----------
    // system's position
    const double *p_ref = pos_ref.data();
    double *position = mjpc::SensorByName(model, data, "position");

    // position error
    mju_sub3(residual + res_index, position, p_ref);
    res_index += 3;

    // ---------- Residual (2) ----------
    // system's orientation
    double body_rotmat[9];
    double *orientation = mjpc::SensorByName(model, data, "orientation");
    mju_quat2Mat(body_rotmat, orientation);

    mju_sub(residual + res_index, body_rotmat, ref_rotmat, 9);
    res_index += 9;

    // ---------- Residual (3) ----------
    // system's linear velocity
    double *vel_trunk = mjpc::SensorByName(model, data, "velocity_trunk");
    const double *v_ref = vel_ref.data();
    mju_sub3(residual + res_index, vel_trunk, v_ref);
    res_index += 3;

    // ---------- Residual (4) ----------
    // system's linear acceleration
    // double *acc_trunk = mjpc::SensorByName(model, data, "acc_lin_trunk");
    // const double *a_ref = acc_ref.data();
    // mju_sub3(residual + res_index, acc_trunk, a_ref);
    // res_index += 3;

    // ---------- Residual (5) ----------
    // system's linear velocity
    double *ang_vel_trunk = mjpc::SensorByName(model, data, "ang_velocity_trunk");
    double ang_v_ref[3];
    ang_v_ref[0] = 0.;
    ang_v_ref[1] = wpitch[0];
    ang_v_ref[2] = 0.;
    mju_sub3(residual + res_index, ang_vel_trunk, ang_v_ref);
    res_index += 3;

  } else {
    // ---------- Residual (1) ----------
    // system's position
    const double p_ref[3] = {0., 0., 0.};
    double position[3] = {0., 0., 0.};

    // position error
    mju_sub3(residual + res_index, position, p_ref);
    res_index += 3;

    mjtNum axis[3] = {0.0, 1.0, 0.0}; // Set y-axis
    mjtNum quat[4];
    mjtNum ref_rotmat[9];
    mju_axisAngle2Quat(quat, axis, 0.); // Convert axis-angle to quaternion
    mju_quat2Mat(ref_rotmat, quat);     // Convert quaternion to rotation matrix

    // ---------- Residual (2) ----------
    // system's orientation
    double body_rotmat[9];
    double *orientation = mjpc::SensorByName(model, data, "orientation");
    mju_quat2Mat(body_rotmat, orientation);

    mju_sub(residual + res_index, body_rotmat, ref_rotmat, 9);
    res_index += 9;

    // ---------- Residual (3) ----------
    // system's linear velocity
    double *vel_trunk = mjpc::SensorByName(model, data, "velocity_trunk");
    const double v_ref[3] = {0., 0., 0.};
    mju_sub3(residual + res_index, vel_trunk, v_ref);
    res_index += 3;

    // ---------- Residual (4) ----------
    // system's linear acceleration
    // double acc_trunk[3] = {0.,0.,0.} ;
    // double a_ref[3] = {0.,0.,0.};
    // mju_sub3(residual + res_index, acc_trunk, a_ref);
    // res_index += 3;

    // ---------- Residual (4) ----------
    // system's linear acceleration
    // double acc_trunk[3] = {0.,0.,0.} ;
    // double a_ref[3] = {0.,0.,0.};
    // mju_sub3(residual + res_index, acc_trunk, a_ref);
    // res_index += 3;

    // ---------- Residual (5) ----------
    // system's linear velocity
    double *ang_vel_trunk = mjpc::SensorByName(model, data, "ang_velocity_trunk");
    const double ang_v_ref[3] = {0., 0., 0.};
    mju_sub3(residual + res_index, ang_vel_trunk, ang_v_ref);
    res_index += 3;
  }

  // ---------- Residual (4) ----------
  // Cost on the command
  mju_copy(residual + res_index, data->ctrl, model->nu);
}

void QuadrupedTask::ResidualFn::Update() {
  num_residual_ = task_->num_residual;
  num_term_ = task_->num_term;
  num_trace_ = task_->num_trace;
  dim_norm_residual_ = task_->dim_norm_residual;
  num_norm_parameter_ = task_->num_norm_parameter;
  norm_ = task_->norm;
  weight_ = task_->weight;
  norm_parameter_ = task_->norm_parameter;
  risk_ = task_->risk;
  parameters_ = task_->parameters;
  // cp[3] = Eigen::Vector3d(1.678, 0.0, 0.052);
  // curve_ = ndcurves::bezier_curve<double, double, true, Eigen::Vector3d>(
  //     cp.begin(), cp.end());
  // std::cout << "Hello" << std::endl;
}

// / draw task-related geometry in the scene
void QuadrupedTask::ModifyScene(const mjModel *model, const mjData *data,
                                mjvScene *scene) const {
  double size[3] = {0.01};
  double *pos;
  double pos_previous[3];

  int n_points = 20;
  // color
  float color[4];
  color[0] = 1.0;
  color[1] = 0.0;
  color[2] = 1.0;
  color[3] = 0.4;
  for (int i = 0; i < n_points; i++) {
    if (i > 0) {
      pos_previous[0] = pos[0];
      pos_previous[1] = pos[1];
      pos_previous[2] = pos[2];
    }
    pos = residual_.curve_(float(i) / float(n_points + 3)).data();
    mjvGeom *geomtest = scene->geoms + scene->ngeom++;
    mjv_initGeom(geomtest, mjGEOM_SPHERE, size, pos, NULL, color);
    scene->geoms[scene->ngeom].category = mjCAT_DECOR;

    if (i > 0) {
      // mjvGeom* geomtest2 = scene->geoms + scene->ngeom++;
      // make connector geom
      mjvGeom *geomtest2 = scene->geoms + scene->ngeom++;
      mjv_initGeom(geomtest2, mjGEOM_LINE,
                   /*size=*/nullptr, /*pos=*/nullptr, /*mat=*/nullptr, color);
      scene->geoms[scene->ngeom].category = mjCAT_DECOR;
      double *from = pos_previous;
      double *to = pos;
      mjv_makeConnector(geomtest2, mjGEOM_LINE, 2, from[0], from[1], from[2],
                        to[0], to[1], to[2]);
    }
  }
  // Plot current time target along the horizon.
  if (data->time - 1. >= 0. && data->time - 1. <= 0.82) {
    size[0] = 0.02;
    size[1] = 0.02;
    size[2] = 0.02;
    color[0] = 0.;
    color[1] = 1.;
    color[2] = 0.;
    color[3] = 1.;
    pos = residual_.curve_(data->time - 1.).data();
    mjvGeom *geomtest = scene->geoms + scene->ngeom++;
    mjv_initGeom(geomtest, mjGEOM_SPHERE, size, pos, NULL, color);
    scene->geoms[scene->ngeom].category = mjCAT_DECOR;
  }
}

// -------- Transition for quadruped task --------
//   If quadruped is within tolerance of goal ->
//   set goal to next from keyframes.
// -----------------------------------------------
void QuadrupedTask::TransitionLocked(mjModel *model, mjData *data) {
  // set mode to GUI selection
  if (mode > 0) {
    residual_.current_mode_ = mode - 1;
  } else {
    // ---------- Compute tolerance ----------
    // goal position
    const double *goal_position = data->mocap_pos;

    // goal orientation
    const double *goal_orientation = data->mocap_quat;

    // system's position
    double *position = mjpc::SensorByName(model, data, "position");

    // system's orientation
    double *orientation = mjpc::SensorByName(model, data, "orientation");

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