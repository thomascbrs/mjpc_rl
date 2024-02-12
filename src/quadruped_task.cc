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
  mju_copy(residual, feet_position, 12);
  res_index += 12;

  // Time varying references.
  if (data->time - 1. >= 0. && data->time - 1. <= 8.) {
    Eigen::Vector3d pos_ref = {0.,0.,0.};
    // std::cout << "data->time - 1. : " << data->time - 1. << std::endl;
    Eigen::Vector3d vel_ref = pcVel_(data->time - 1.);
    Eigen::Vector3d rot_ref = pcRot_(data->time - 1.);
    // std::cout << "vel_ref : [" << vel_ref[0] << "," << vel_ref[1] << "," << vel_ref[2] << "]" << std::endl ;
    // std::cout << "data->time - 1.2 : " << data->time - 1. << std::endl;
    // Eigen::Vector3d acc_ref = curve_acc_(data->time - 1.);

    // Compute derivative of the curve wrt to x to retrieve pitch angle.
    double pitch[1];
    double wpitch[1];
    double fwd = 0.0;
    // getPitch(pitch, wpitch, data->time + fwd - 1.);

    mjtNum axis[3] = {0.0, 1.0, 0.0}; // Set y-axis
    mjtNum quat[4];
    mjtNum ref_rotmat[9];
    mju_axisAngle2Quat(quat, axis,
                       rot_ref[1]);   // Convert axis-angle to quaternion
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
    double *ang_vel_trunk =
        mjpc::SensorByName(model, data, "ang_velocity_trunk");
    double ang_v_ref[3];
    ang_v_ref[0] = 0.;
    ang_v_ref[1] = wpitch[0];
    ang_v_ref[2] = 0.;
    mju_sub3(residual + res_index, ang_vel_trunk, ang_v_ref);
    res_index += 3;

    // ---------- Residual (6) ----------
    // std::string list_names[6] = {"nn", "Height", "air_time_FR",
    // "air_time_FL", "air_time_HR", "air_time_HL"}; for (const auto&
    // name:list_names){
    //   double indexes[2];
    //   ParameterIndexes(indexes, model,"residual_" + name);
    //   std::cout << name << " : [" << indexes[0] << " , " << indexes[1] << "]"
    //   << std::endl; std::cout << "param = ["; for (int k=indexes[0];k <
    //   indexes[1] ; k++ ){
    //     std::cout << parameters_[k] << ",";
    //   }
    //   std::cout << "]" << std::endl;
    // }
    std::string prefix = "residual_air_time_";
    std::string foot_names[4] = {"FR", "FL", "HR", "HL"};
    int indexes[2];
    ParameterIndexes(indexes, model, prefix + "limit");
    double time_limit = parameters_[indexes[0]];
    ParameterIndexes(indexes, model, prefix + "time0");
    double time0 = parameters_[indexes[0]];

    double z_positions[4];
    double z_positions_ref[4] = {-0.0, -0.0, -0.05, -0.05};
    int shift = 0;

    for (const auto &name : foot_names) {
      ParameterIndexes(indexes, model, prefix + name);
      z_positions[shift] = 0.;
      // std::cout << "\ndata->time : " << data->time << std::endl;
      if (parameters_[indexes[0]] > 0.05) { // Foot currently the air
        if (data->time - time0 + parameters_[indexes[0]] > time_limit) {
          if (data->time - time0 + parameters_[indexes[0]] < time_limit + 0.2 ) {
            // std::cout << name << "indexes[0] : " << indexes[0] << std::endl;
            // mju_sub3(residual + res_index, mjpc::SensorByName(model, data,
            // name)[2], 0.); std::cout << "Activate air time cost on " << name
            // << std::endl;
            z_positions[shift] = mjpc::SensorByName(model, data, name)[2];
            // z_positions[shift] = std::pow(mjpc::SensorByName(model, data, name + "_touch")[0],-2);
            // std::cout << name << " : " << z_positions[shift] << std::endl;
            // std::cout << name << " : " << mjpc::SensorByName(model, data, name + "_touch")[0] << std::endl;
            // if (z_positions[shift] < 0.){
            //   z_positions[shift] = 0.;
            // }
          }
        }
      }
      shift++;
      // std::cout << name << " = " <<  parameters_[indexes[0]] << std::endl;
    }
    // std::cout << "[" << z_positions[0] << z_positions[1] << z_positions[2] <<
    // z_positions[3] << "]" << std::endl;
    mju_sub3(residual + res_index, z_positions, z_positions_ref);
    res_index += 4;
    // ---------- Residual (7) ----------
    // Force feet penalisation
    // std::vector<std::string> force_name =
    // {"FR_force","FL_force","HR_force","HL_force"}; double forces_ref[3] =
    // {33.,0.,0.}; for (const auto& name:force_name){
    //   double *forces = mjpc::SensorByName(model, data, name);
    //   mju_sub3(residual + res_index, forces, forces_ref);
    //   res_index += 3;
    // }

    // ---------- Residual (7) ----------
    // Feet velocity
    std::vector<std::string> force_name = {"FR_vel", "FL_vel", "HR_vel",
                                           "HL_vel"};
    double feet_acc_ref[3] = {0., 0., 0.};
    for (const auto &name : force_name) {
      double *feet_acc = mjpc::SensorByName(model, data, name);
      mju_sub3(residual + res_index, feet_acc, feet_acc_ref);
      res_index += 3;
    }

    // ---------- Residual (8) -----------
    // Symmetric term
    Eigen::Map<Eigen::Matrix<double, 12, 1>> u(data->ctrl);
    Eigen::Matrix<double, 4, 1> C2_u = C2 * u;
    mju_copy(residual + res_index, C2_u.data() , 4);
    res_index += 4;

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
    double *ang_vel_trunk =
        mjpc::SensorByName(model, data, "ang_velocity_trunk");
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
  // parameters_[0] --> residual_nn_updated
  if (parameters_[0] > 0){
    // Update the reference curve.
    std::cout << "Update Reference curve." << std::endl;

    // Does not work : Update does not take model.
    // int indexes[2];
    // ParameterIndexes(indexes, model, "residual_nn");
    // const double *params_nn = &parameters_[0];

    updateCurves(parameters_.begin() + 1, parameters_.begin() + 6);
    n_update += 1;
  }
}

void QuadrupedTask::ResidualFn::updateCurves(const std::vector<double>::iterator start, const std::vector<double>::iterator end) {
  double T = lin_velocity_.max();
  double T2 = lin_velocity_.max() + 0.4;
  Eigen::MatrixXd minv(3, 3);
  Eigen::MatrixXd coeffs(3,3);
  Eigen::MatrixXd b(3, 3);
  minv << 1, 0, 0,
          0, 1, 0,
          -std::pow((T2 - T), -2), -std::pow((T2 - T), -1), std::pow((T2 - T), -2);

  // Linear velocities.
  b.row(0) = pcVel_(pcVel_.max());
  b.row(1) = pcVel_.derivate(pcVel_.max(),1);
  b.row(2) << *start, *(start +1), *(start +2);
  coeffs = minv * b;

  Polynomial curve_tmp;
  curve_tmp = Polynomial(coeffs.transpose(),pcVel_.max(),pcVel_.max() + 0.4);
  pcVel_.add_curve(curve_tmp);

  // Rotation angles.
  b.row(0) = pcRot_(pcRot_.max());
  b.row(1) = pcRot_.derivate(pcRot_.max(),1);
  b.row(2) << *start+3, *(start +4), *(start +5);
  coeffs = minv * b;

  Polynomial curveRot_tmp;
  curveRot_tmp = Polynomial(coeffs.transpose(),pcRot_.max(),pcRot_.max() + 0.4);
  pcRot_.add_curve(curveRot_tmp);
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