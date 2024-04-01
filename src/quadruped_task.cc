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

#include "pinocchio/math/rpy.hpp"
#include "pinocchio/spatial/se3.hpp"
#include "types.h"
#include <Eigen/Geometry>
#include <pinocchio/math/quaternion.hpp>

std::string QuadrupedTask::XmlPath() const {
  return mjpc::GetModelPath("quadruped/task_hill.xml");
}

std::string QuadrupedTask::Name() const { return "Quadruped Task"; }

void QuadrupedTask::ResidualFn::ParameterIndexes(
    int indexes[2], const mjModel *model, const std::string_view name) const {
  int id =
      // mj_name2id(model, mjOBJ_NUMERIC, absl::StrCat("residual_",
      // name).c_str()); Use residual in name.
      mj_name2id(model, mjOBJ_NUMERIC, std::string(name).c_str());

  if (id == -1) {
    mju_error_s("Parameter '%s' not found", std::string(name).c_str());
  }

  int shift = 0;
  int first_residual = 0;
  int i;
  // Suppose all residual are defined at in block
  for (i = 0; i < model->nnumeric; i++) {
    const char *obj_name = mj_id2name(model, mjOBJ_NUMERIC, i);
    if (i == id) {
      break;
    }
    if (absl::StartsWith(obj_name, "residual_")) {
      shift += model->numeric_size[i];
      first_residual = (first_residual == 0) ? i : first_residual;
    }
  }
  indexes[0] = shift;
  indexes[1] = shift + model->numeric_size[i];
}

// TODO: Compute pitch angle once, when the curve is created.
void QuadrupedTask::ResidualFn::getPitch(double pitch[1], double wpitch[1],
                                         double t) const {
  // Compute derivative of the curve wrt to x to retrieve pitch angle.
  double dt = 0.01;
  double factor = 0.5;

  Eigen::Vector3d p0; // Current position
  Eigen::Vector3d p1; // Backward (- dt)
  Eigen::Vector3d p2; // Forward  (+ dt)

  if (t - dt >= curve_.min() && t + dt <= curve_.max()) {
    p0 = curve_(t);
    p1 = curve_(t - dt);
    p2 = curve_(t + dt);
  } else {
    if (t - dt < curve_.min()) {
      // Beginning of the curve. Shift of dt.
      p0 = curve_(t + dt);
      p1 = curve_(t);
      p2 = curve_(t + 2 * dt);
    } else {
      // End of the curve. Shift of -dt.
      p0 = curve_(t - dt);
      p1 = curve_(t - 2 * dt);
      p2 = curve_(t);
    }
  }

  // Compute pitch angle. Forward.
  if (p2[0] - p0[0] == 0.) {
    pitch[0] = 0.;
  } else {
    pitch[0] = (p2[2] - p0[2]) / (p2[0] - p0[0]);
  }
  double pitch_backward = 0.;
  if (p0[0] - p1[0] == 0.) {
    pitch_backward = 0.;
  } else {
    pitch_backward = (p0[2] - p1[2]) / (p0[0] - p1[0]);
  }

  // Cmpute derivative.
  wpitch[0] = (pitch[0] - pitch_backward) / dt;

  // Add factor
  pitch[0] *= -factor;
  wpitch[0] *= -factor;
}

void QuadrupedTask::ResidualFn::Residual(const mjModel *model,
                                         const mjData *data,
                                         double *residual) const {

  int res_index = 0;
  // ---------- Residual (0) ----------
  // Fly-high cost.
  double *FR = mjpc::SensorByName(model, data, "FR");
  double *FL = mjpc::SensorByName(model, data, "FL");
  double *RR = mjpc::SensorByName(model, data, "HR");
  double *RL = mjpc::SensorByName(model, data, "HL");
  double *FR_vel = mjpc::SensorByName(model, data, "FR_vel");
  double *FL_vel = mjpc::SensorByName(model, data, "FL_vel");
  double *RR_vel = mjpc::SensorByName(model, data, "HR_vel");
  double *RL_vel = mjpc::SensorByName(model, data, "HL_vel");
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
  // TODO: Make this cost dependent on the height.
  mju_copy(residual, feet_position, 12);
  res_index += 12;

  // ---------- Residual (1) ----------
  // Control.
  int indexes[2];
  ParameterIndexes(indexes, model, "residual_ctrl_factor_hip");
  double factor = parameters_[indexes[0]];
  double ctrl[12];
  std::copy(data->ctrl, data->ctrl + 12, ctrl);
  ctrl[0] *= factor;
  ctrl[3] *= factor;
  ctrl[6] *= factor;
  ctrl[9] *= factor;
  mju_copy(residual + res_index, ctrl, model->nu);
  res_index += 12;

  double data_time = std::roundf(data->time * 1000) / 1000;

  // ---------- Residual (2) ----------
  // system's linear velocity
  Eigen::Vector3d vel_ref = pcVel_(data_time); // Defined in Local frame.
  ParameterIndexes(indexes, model, "residual_yaw_local");
  Matrix3d R_tmp = pinocchio::rpy::rpyToMatrix(0., 0., parameters_[indexes[0]]);
  Eigen::Vector3d vel_ref_world = R_tmp * vel_ref;
  vel_ref_world(2) = vel_ref(2); // Only x,y axis.
  // Eigen::Vector3d vel_ref = Eigen::Vector3d::Zero(3);
  double *vel_trunk = mjpc::SensorByName(model, data, "velocity_trunk");
  const double *v_ref = vel_ref_world.data();
  mju_sub3(residual + res_index, vel_trunk, v_ref);
  res_index += 3;

  // ---------- Residual (3) ----------
  // system's orientation
  // Eigen::Vector3d rot_ref = Eigen::Vector3d::Zero(3);
  Eigen::Vector3d rot_ref = pcRot_(data_time);

  // Compute derivative of the curve wrt to x to retrieve pitch angle.
  Matrix3d R = pinocchio::rpy::rpyToMatrix(rot_ref(0), rot_ref(1), rot_ref(2));

  // mjtNum axis[3] = {0.0, 1.0, 0.0}; // Set y-axis
  // mjtNum quat[4];
  // mjtNum ref_rotmat[9];
  // mju_axisAngle2Quat(quat, axis,
  //                     rot_ref[1]);   // Convert axis-angle to quaternion
  // mju_quat2Mat(ref_rotmat, quat); // Convert quaternion to rotation matrix
  // Convert R to mjtNum ref_rotmat[9]
  mjtNum ref_rotmat[9];
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      ref_rotmat[i * 3 + j] = static_cast<mjtNum>(R(i, j));
    }
  }

  double body_rotmat[9];
  double *orientation = mjpc::SensorByName(model, data, "orientation");
  mju_quat2Mat(body_rotmat, orientation);

  mju_sub(residual + res_index, body_rotmat, ref_rotmat, 9);
  res_index += 9;

  // ---------- Residual (4) ----------
  // Angular Velocity
  double *ang_vel_trunk = mjpc::SensorByName(model, data, "ang_velocity_trunk");
  double ang_v_ref[3];
  ang_v_ref[0] = 0.;
  ang_v_ref[1] = 0.;
  ang_v_ref[2] = 0.;
  mju_sub3(residual + res_index, ang_vel_trunk, ang_v_ref);
  res_index += 3;

  // ---------- Residual (5) -----------
  // Symmetric term
  Eigen::Map<Eigen::Matrix<double, 12, 1>> u(data->ctrl);
  Eigen::Matrix<double, 4, 1> C2_u = C2 * u;
  mju_copy(residual + res_index, C2_u.data(), 4);
  res_index += 4;

  // ---------- Residual (6) ----------
  // Residual Air-time.
  // std::string prefix = "residual_air_time_";
  // std::string foot_names[4] = {"FR", "FL", "HR", "HL"};
  // // int indexes[2];
  // ParameterIndexes(indexes, model, prefix + "limit");
  // double time_limit = parameters_[indexes[0]];
  // ParameterIndexes(indexes, model, prefix + "time0");
  // double time0 = parameters_[indexes[0]];

  // double z_positions[4];
  // double z_positions_ref[4] = {-0.0, -0.0, -0.05, -0.05};
  // int shift = 0;

  // for (const auto &name : foot_names) {
  //   ParameterIndexes(indexes, model, prefix + name);
  //   z_positions[shift] = 0.;
  //   // std::cout << "\ndata->time : " << data->time << std::endl;
  //   if (parameters_[indexes[0]] > 0.05) { // Foot currently the air
  //     if (data->time - time0 + parameters_[indexes[0]] > time_limit) {
  //       if (data->time - time0 + parameters_[indexes[0]] < time_limit + 0.2 )
  //       {
  //         z_positions[shift] = mjpc::SensorByName(model, data, name)[2];
  //       }
  //     }
  //   }
  //   shift++;
  // }
  // mju_sub3(residual + res_index, z_positions, z_positions_ref);
  // res_index += 4;
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
  // parameters_[0] --> residual_nn_updated
  if (parameters_[0] == 1.) {
    // Update the reference curve.
    // std::cout << "Update Reference curve." << std::endl;

    // Does not work : Update does not take model.
    // int indexes[2];
    // ParameterIndexes(indexes, model, "residual_nn");
    // const double *params_nn = &parameters_[0];

    updateCurvesLin(parameters_.begin() + 1, parameters_.begin() + 6);
    n_update += 1;
  }
  // Reset the reference curve.
  if (parameters_[0] == 0.) {
    reset_curves(parameters_.begin() + 7);
    n_update = 0;

    // Environement
    int envId = int(parameters_[10]);
    heightmap_.setCurrentEnvironment(envId);
  }

  if (int(parameters_[0]) == int(2)){
    // residual_nn_horizon
    set_horizon_nn(parameters_[11]);
  }
  if (int(parameters_[0]) == int(3)){
    // residual_nn_horizon
    set_horizon_reset(parameters_[11]);
  }
}

void QuadrupedTask::ResidualFn::set_horizon_reset(double horizon_reset) {
  horizon_reset_ = horizon_reset;
  cp_rot.clear();
  cp_lin.clear();

  // Define a constant polynomial curve.
  cp_rot.push_back(Eigen::Vector3d(0.,0.,0.));
  cp_lin.push_back(Eigen::Vector3d(0., 0., 0.));
  for (int i = 1; i < 3; i++) {
    cp_rot.push_back(Eigen::Vector3d(0., 0., 0.));
    cp_lin.push_back(Eigen::Vector3d(0., 0., 0.));
  }
  lin_velocity_ = Polynomial(cp_lin.begin(), cp_lin.end(), 0., horizon_reset_);
  ang_rotation_ = Polynomial(cp_rot.begin(), cp_rot.end(), 0., horizon_reset_);
  pcRot_ = PieceWise();
  pcVel_ = PieceWise();
  pcRot_.add_curve(ang_rotation_);
  pcVel_.add_curve(lin_velocity_);
}

void QuadrupedTask::ResidualFn::reset_curves(
    const std::vector<double>::iterator start) {
  // std::cout << "Reset function in task" << std::endl;
  // std::cout << "params : [" << *start << "," << *(start+1) << "," << *(start
  // +2) << "]" << std::endl; Update the container of points.
  cp_rot.clear();
  cp_lin.clear();

  // Define a constant polynomial curve.
  cp_rot.push_back(Eigen::Vector3d(*start, *(start + 1), *(start + 2)));
  cp_lin.push_back(Eigen::Vector3d(0., 0., 0.));
  for (int i = 1; i < 3; i++) {
    cp_rot.push_back(Eigen::Vector3d(0., 0., 0.));
    cp_lin.push_back(Eigen::Vector3d(0., 0., 0.));
  }
  lin_velocity_ = Polynomial(cp_lin.begin(), cp_lin.end(), 0., horizon_reset_);
  ang_rotation_ = Polynomial(cp_rot.begin(), cp_rot.end(), 0., horizon_reset_);

  pcRot_ = PieceWise();
  pcVel_ = PieceWise();
  pcRot_.add_curve(ang_rotation_);
  pcVel_.add_curve(lin_velocity_);
}

void QuadrupedTask::ResidualFn::updateCurvesVEL(
    const std::vector<double>::iterator start,
    const std::vector<double>::iterator end) {
  double T = lin_velocity_.max();
  double T2 = lin_velocity_.max() + horizon_nn_;
  Eigen::MatrixXd minv(3, 3);
  Eigen::MatrixXd coeffs(3, 3);
  Eigen::MatrixXd b(3, 3);
  minv << 1, 0, 0, 0, 1, 0, -std::pow((T2 - T), -2), -std::pow((T2 - T), -1),
      std::pow((T2 - T), -2);

  // Linear velocities.
  b.row(0) = pcVel_(pcVel_.max());
  b.row(1) = pcVel_.derivate(pcVel_.max(), 1);
  b.row(2) << *start, *(start + 1), *(start + 2);
  coeffs = minv * b;

  Polynomial curve_tmp;
  curve_tmp =
      Polynomial(coeffs.transpose(), pcVel_.max(), pcVel_.max() + horizon_nn_);
  pcVel_.add_curve(curve_tmp);

  // Rotation angles.
  b.row(0) = pcRot_(pcRot_.max());
  b.row(1) = pcRot_.derivate(pcRot_.max(), 1);
  b.row(2) << *start + 3, *(start + 4), *(start + 5);
  coeffs = minv * b;

  Polynomial curveRot_tmp;
  curveRot_tmp =
      Polynomial(coeffs.transpose(), pcRot_.max(), pcRot_.max() + horizon_nn_);
  pcRot_.add_curve(curveRot_tmp);
}

void QuadrupedTask::ResidualFn::updateCurvesLin(
    const std::vector<double>::iterator start,
    const std::vector<double>::iterator end) {
  double T = 0.;
  double T2 = horizon_nn_;

  Eigen::MatrixXd coeffs_vel(2, 3);
  Eigen::MatrixXd b(2, 3);
  Eigen::MatrixXd minv(2, 2);
  minv << 1, 0, -1 / (T2 - T), 1 / (T2 - T);
  b.row(0) = pcVel_(pcVel_.max());
  b.row(1) = pcVel_(pcVel_.max());
  // Parameters = dV(+horizon)
  b(1, 0) += *(start);
  b(1, 1) += *(start + 1);
  b(1, 2) += *(start + 2);
  // Parameters = V(+horizon)
  // b.row(1) << *(start), *(start +1), *(start +2);
  coeffs_vel = minv * b;

  Polynomial curveVel_tmp;
  curveVel_tmp = Polynomial(coeffs_vel.transpose(), pcVel_.max(),
                            pcVel_.max() + horizon_nn_);
  pcVel_.add_curve(curveVel_tmp);

  Eigen::MatrixXd coeffs_rot(2, 3);
  minv << 1, 0, -1 / (T2 - T), 1 / (T2 - T);
  b.row(0) = pcRot_(pcRot_.max());
  b.row(1) = pcRot_(pcRot_.max());
  b(1, 0) += *(start + 3);
  b(1, 1) += *(start + 4);
  b(1, 2) += *(start + 5);
  // b.row(1) << *(start+3), *(start +4), *(start +5);
  coeffs_rot = minv * b;

  Polynomial curveRot_tmp;
  curveRot_tmp = Polynomial(coeffs_rot.transpose(), pcRot_.max(),
                            pcRot_.max() + horizon_nn_);
  pcRot_.add_curve(curveRot_tmp);

  // Visualisation.
  // double tt = 0.;
  // std::cout << "\n\n----" << std::endl;
  // while( tt <= pcVel_.max()){
  //   std::cout << "Vel_ref(" << tt << ") = [" <<
  //   pcVel_(tt)[0] << "," << pcVel_(tt)[1] << "," << pcVel_(tt)[2] << "," <<
  //   pcRot_(tt)[0] << "," << pcRot_(tt)[1] << "," << pcRot_(tt)[2] << "]" <<
  //   std::endl; tt += 0.01;
  // }
}

void QuadrupedTask::ResidualFn::updateCurvesACC(
    const std::vector<double>::iterator start,
    const std::vector<double>::iterator end) {
  double T = 0.;
  double T2 = horizon_nn_;
  Eigen::MatrixXd coeffs(3, 3);

  coeffs.row(0) = pcVel_(pcVel_.max());
  coeffs.row(1) = pcVel_.derivate(pcVel_.max(), 1);
  coeffs.row(2) << *start, *(start + 1), *(start + 2);
  coeffs.row(2) -= coeffs.row(1);
  coeffs.row(2) *= 0.5 / (T2 - T);

  Polynomial curve_tmp;
  curve_tmp =
      Polynomial(coeffs.transpose(), pcVel_.max(), pcVel_.max() + horizon_nn_);
  pcVel_.add_curve(curve_tmp);

  // Rotation angles.
  // Polynomial curve in angular position. May induce waving motion due to
  // the representation (trying to keep the c1 continuity)
  // Polynomial 2nd degree

  // Eigen::MatrixXd b(3, 3);
  // Eigen::MatrixXd minv(3, 3);
  // minv << 1, 0, 0,
  //         0, 1, 0,
  //         -std::pow((T2 - T), -2), -std::pow((T2 - T), -1), std::pow((T2 -
  //         T), -2);
  // b.row(0) = pcRot_(pcRot_.max());
  // b.row(1) = pcRot_.derivate(pcRot_.max(),1);
  // b.row(2) << *(start+3), *(start +4), *(start +5);
  // coeffs = minv * b;

  // Polynomial curveRot_tmp;
  // curveRot_tmp = Polynomial(coeffs.transpose(),pcRot_.max(),pcRot_.max() +
  // 0.4); pcRot_.add_curve(curveRot_tmp);

  // 1st degree in rotation angle.
  Eigen::MatrixXd coeffs_rot(2, 3);
  coeffs_rot.row(0) = pcRot_(pcRot_.max());
  Eigen::MatrixXd b(2, 3);
  Eigen::MatrixXd minv(2, 2);
  minv << 1, 0, -1 / (T2 - T), 1 / (T2 - T);
  b.row(0) = pcRot_(pcRot_.max());
  b.row(1) << *(start + 3), *(start + 4), *(start + 5);
  coeffs = minv * b;

  Polynomial curveRot_tmp;
  curveRot_tmp =
      Polynomial(coeffs.transpose(), pcRot_.max(), pcRot_.max() + horizon_nn_);
  pcRot_.add_curve(curveRot_tmp);

  // Visualisation.
  // double tt = 0.;
  // std::cout << "\n\n----" << std::endl;
  // while( tt <= pcVel_.max()){
  //   std::cout << "Vel_ref(" << tt << ") = [" <<
  //   pcVel_(tt)[0] << "," << pcVel_(tt)[1] << "," << pcVel_(tt)[2] << "," <<
  //   pcRot_(tt)[0] << "," << pcRot_(tt)[1] << "," << pcRot_(tt)[2] << "]" <<
  //   std::endl; tt += 0.01;
  // }
}

// Using update function for now, maybe to use for resetting the curves.
void QuadrupedTask::ResetLocked(const mjModel *model) {}

// / draw task-related geometry in the scene
void QuadrupedTask::ModifyScene(const mjModel *model, const mjData *data,
                                mjvScene *scene) const {

  double size[3] = {0.01};
  double pos[3];
  pos[0] = data->qpos[0];
  pos[1] = data->qpos[1];
  pos[2] = data->qpos[2] + 0.05;
  double pos_previous[3] = {pos[0], pos[1], pos[2]};

  // color
  float color[4];
  color[0] = 0.;
  color[1] = 1.0;
  color[2] = 0.;
  color[3] = 0.7;

  // At time data-time, get rotationmatrix for world frame velocity reference.
  const mjtNum axis[3] = {0., 0., 1.}; // z-axis (yaw)
  Matrix3d R_tmp = Matrix3d::Zero();
  mjtNum R_data[9];
  mjtNum quat_tmp[4];
  // Get quaternion projected on z-axis (only yaw component).
  mju_mulQuatAxis(quat_tmp, &data->qpos[3],
                  axis);          // Convert axis-angle to quaternion
  mju_quat2Mat(R_data, quat_tmp); // Convert quaternion to rotation matrix
  updateMatrix(R_tmp, R_data);

  Vector3d vel_world = Vector3d::Zero();
  Vector3d vel_tmp = Vector3d::Zero();
  Vector3d dx = Vector3d::Zero();
  Matrix3d dR = Matrix3d::Zero();

  double t_min = data->time;
  double t_max = residual_.pcVel_.max() - t_min;
  double dt = 0.02;
  int n_points = int(t_max / dt);

  for (int i = 0; i < n_points -1; i++) {
    if (i > 0) {
      pos_previous[0] = pos[0];
      pos_previous[1] = pos[1];
      pos_previous[2] = pos[2];
    }
    double t = t_min + t_max * (float(i) / float(n_points));

    vel_tmp = residual_.pcVel_(t);

    // Vector3d rot = residual_.pcRot_.derivate(t,1);
    Vector3d rot = residual_.pcRot_(t);
    // R_tmp = pinocchio::rpy::rpyToMatrix(0., 0., rot(2));
    vel_world = vel_tmp;

    dR = pinocchio::rpy::rpyToMatrix(rot(0), rot(1), rot(2));
    Vector3d dx = dR * dt * vel_world;
    pos[0] = pos[0] + dx(0);
    pos[1] = pos[1] + dx(1);
    pos[2] = pos[2] + dx(2);

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

// initial residual parameters from model
void QuadrupedTask::SetParameters(const mjModel *model) {
  // set counter
  int num_parameters = 0;

  // search custom numeric in model for "residual"
  for (int i = 0; i < model->nnumeric; i++) {
    if (absl::StartsWith(model->names + model->name_numericadr[i],
                         "residual_")) {
      num_parameters += model->numeric_size[i];
    }
  }

  // allocate memory
  parameters.resize(num_parameters);

  // set values
  int shift = 0;
  for (int i = 0; i < model->nnumeric; i++) {
    // residual_select_ not taken into account here.
    // Incrementally fill parameters
    if (absl::StartsWith(model->names + model->name_numericadr[i],
                         "residual_")) {
      int startIdx = model->numeric_adr[i];
      int endIdx = startIdx + model->numeric_size[i];

      // Incrementally fill parameters
      for (int j = startIdx; j < endIdx; j++) {
        parameters[shift++] = model->numeric_data[j];
      }
      // Update the internal dictionnay.
      // param_index[model->names + model->name_numericadr[i]] = startIdx;
      // param_size[model->names + model->name_numericadr[i]] = endIdx;
    }
  }
}