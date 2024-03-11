#include "observer.h"

// #include <pinocchio/fwd.hpp>
#include <Eigen/Geometry>
#include <pinocchio/math/quaternion.hpp>
#include "pinocchio/math/rpy.hpp"
#include "pinocchio/spatial/se3.hpp"

void Observer::update_filter(const mjModel *model, const mjData *data){
  Vector3d rpy;
  rpy = pinocchio::rpy::matrixToRpy(Eigen::Quaterniond(data->qpos[3], data->qpos[4], data->qpos[5], data->qpos[6]).toRotationMatrix());

  std::vector<double> qpos_tmp(data->qpos, data->qpos + 3); // Position
  qpos_tmp.push_back(rpy(0));
  qpos_tmp.push_back(rpy(1));
  qpos_tmp.push_back(rpy(2));

  std::vector<double> filtered_tmp = filter_pos_._filter(qpos_tmp);
  // Copy data inside the filtered_pose
  std::copy(filtered_tmp.begin(), filtered_tmp.end(), odata_.filtered_pose.begin());

  std::vector<double> qvel_tmp(data->qvel, data->qvel + 6); // Velocity
  std::vector<double> vfiltered_tmp = filter_vel_._filter(qvel_tmp);
  std::copy(vfiltered_tmp.begin(), vfiltered_tmp.end(), odata_.filtered_vel.begin());
}

void Observer::reset(const std::vector<double>& q){
  odata_.end_pose = {0.0};
  odata_.end_vel = {0.0};
  odata_.end_acc = {0.0};
  odata_.filtered_pose = {0.0};
  odata_.filtered_vel = {0.0};
  for (const auto &name : odata_.foot_names) {
    odata_.feet_vel[name] = {0.};
    odata_.feet_pos[name] = {0.};
    odata_.contact_status[name] = 1; // Initialisation in contact.
  }
  filter_pos_.reset();
  filter_vel_.reset();
  odata_.collision_status = false;
  reset_curves(q);
}

void Observer::update_final_pose(const mjModel *model, const mjData *data) {
  // Fill in end_pos, end_vel, end_acc, end_quat, end_ang
  for (int i = 0; i < 6; ++i) {
    if (i < 3) {
      odata_.end_pose[i] = data->qpos[i];
    }
    odata_.end_vel[i] = data->qvel[i];
    odata_.end_acc[i] = data->qacc[i];
  }

  // Fill pose with rpy.
  Vector3d rpy_tmp;
  rpy_tmp = pinocchio::rpy::matrixToRpy(
      Eigen::Quaterniond(data->qpos[3], data->qpos[4], data->qpos[5],
                         data->qpos[6])
          .toRotationMatrix());
  for (int i = 0; i < 3; ++i) {
    odata_.end_pose[i + 3] = rpy_tmp(i);
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
  pos_tmp[0] = odata_.end_pose[0];
  pos_tmp[1] = odata_.end_pose[1];
  // pos_tmp[2] = odata_.end_pose[2];
  pos_tmp[2] = 0.;
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
    Vector3d res = Vector3d::Zero(3);
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

  // Update final reference velocity and angular position
  Vector3d vref = pcVel_(pcVel_.max());
  Vector3d angRef = pcRot_(pcRot_.max());
  for (int i=0; i < 3;i++){
    odata_.lvref[i] = vref(i);
    odata_.orientation_ref[i] = angRef(i);
  }
}

void Observer::update_contact_status(const ContactData &contactData) {
  for (auto &elem : contactData.contact_status) {
    odata_.contact_status[elem.first] = elem.second;
  }
}

void Observer::update_ref_curve(const std::vector<double>& actions){
  if (actions.size() != 6){
    throw std::runtime_error("Actions should be size 6.");
  }
  double T = 0.;
  double T2 = horizon_nn_;
  Eigen::MatrixXd coeffs(3,3);

  coeffs.row(0) = pcVel_(pcVel_.max());
  coeffs.row(1) = pcVel_.derivate(pcVel_.max(),1);
  coeffs.row(2) << actions.at(0), actions.at(1), actions.at(2);
  coeffs.row(2) -= coeffs.row(1);
  coeffs.row(2) *= 0.5/(T2 - T);

  Polynomial curve_tmp;
  curve_tmp = Polynomial(coeffs.transpose(),pcVel_.max(),pcVel_.max() + horizon_nn_);
  pcVel_.add_curve(curve_tmp);

  // 1st degree in rotation angle.
  Eigen::MatrixXd coeffs_rot(2,3);
  coeffs_rot.row(0) = pcRot_(pcRot_.max());
  Eigen::MatrixXd b(2, 3);
  Eigen::MatrixXd minv(2, 2);
  minv << 1, 0, -1 / (T2 - T), 1 / (T2 - T);
  b.row(0) = pcRot_(pcRot_.max());
  b.row(1) << actions.at(3), actions.at(4), actions.at(5);
  coeffs = minv * b;

  Polynomial curveRot_tmp;
  curveRot_tmp = Polynomial(coeffs.transpose(),pcRot_.max(),pcRot_.max() + horizon_nn_);
  pcRot_.add_curve(curveRot_tmp);
}

void Observer::reset_curves(const std::vector<double>& q) {
  if (q.size() != 6) {
    throw std::runtime_error("q0 should be size 6, [x,y,z,r,p,y]");
  }

  Matrix3d coeffs = Matrix3d::Zero();
  Matrix3d coeffs_rot = Matrix3d::Zero();
  coeffs_rot.row(0) << q.at(3),q.at(4),q.at(5); // First coefficients --> constant.

  Polynomial lin_velocity_ = Polynomial(coeffs.transpose(),0.,horizon_reset_);
  Polynomial ang_rotation_ = Polynomial(coeffs_rot.transpose(),0.,horizon_reset_);
  pcRot_ = PieceWise();
  pcVel_ = PieceWise();
  pcRot_.add_curve(ang_rotation_);
  pcVel_.add_curve(lin_velocity_);
}