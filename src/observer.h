#ifndef OBSERVER_H
#define OBSERVER_H

#include "mjpc/utilities.h"
#include "mujoco/mujoco.h"
#include "ndcurves/piecewise_curve.h"
#include "ndcurves/polynomial.h"
#include <iostream>

#include "contact_data.h"
#include "filter.h"
#include "types.h"
#include "utils.h"

// Define ObserverData outside the Observer class
// This class will be shared between c++ <--> python.
// Working with RPY.
struct ObserverData {
  std::vector<std::string> foot_names;
  std::array<double, 6> end_pose = {0.0};
  std::array<double, 6> end_vel = {0.0};
  std::array<double, 6> end_acc = {0.0};
  std::array<double, 6> filtered_pose = {0.0};
  std::array<double, 6> filtered_vel = {0.0};
  std::array<double, 3> lvref = {0.0};           // reference linear velocity.
  std::array<double, 3> orientation_ref = {0.0}; // reference rpy.
  // std::array<double, 6> filtered_acc = {0.0};
  std::unordered_map<std::string, std::array<double, 3>> feet_pos;
  std::unordered_map<std::string, std::array<double, 3>> feet_vel;
  std::unordered_map<std::string, std::array<double, 3>> lfeet_pos;
  std::unordered_map<std::string, std::array<double, 3>> lfeet_vel;

  // Collision status
  bool collision_status = false;
  std::unordered_map<std::string, int> contact_status;

  // Squared sum
  std::array<double, 1> sq_height = {0.0};
  std::array<double, 3> sq_angle = {0.0};
  std::array<double, 6> sq_vel = {0.0};
  std::array<double, 12> sq_control = {0.0};
};

class Observer {
private:
  ObserverData odata_;
  Matrix3d R_tmp;
  Vector3d pos_tmp;
  const mjtNum axis[3]; // z-axis (yaw)

  std::vector<double> cutoff_pos = {2., 2., 2., 2., 2., 2.};
  double fs_pos = 1 / 0.002;
  int order_pos = 1;
  std::vector<double> cutoff_vel = {2., 2., 2., 2., 2., 2.};
  double fs_vel = 1 / 0.002;
  int order_vel = 1;
  Filter filter_pos_;
  Filter filter_vel_;

  double horizon_nn_ = 0.24;
  double horizon_reset_ = 0.4;

  // Reference trajectories, copies from Quadruped_Task
  // Cannot access due to the trhead system. Need to modify ResidualFn
  // otherwise.
  PieceWise pcVel_;
  PieceWise pcRot_;

public:
  Observer(const std::vector<std::string> &fnames, double horizon_nn, double horizon_reset)
      : odata_(), R_tmp(Matrix3d::Identity()),
        pos_tmp(Vector3d::Zero()), axis{0., 0., 1.},
        filter_pos_(cutoff_vel, fs_vel, order_vel),
        filter_vel_(cutoff_vel, fs_vel, order_vel) {
    for (const auto &name : fnames) {
      odata_.foot_names.push_back(name);
      odata_.feet_vel[name] = {0., 0., 0.};
      odata_.feet_pos[name] = {0., 0., 0.};
      odata_.contact_status[name] = 1; // Initialisation in contact.
    }
    horizon_reset_ = horizon_reset;
    horizon_nn_ = horizon_nn;
    // Initialize the reference trajectories.
    // Update the container of points.
    Matrix3d coeffs = Matrix3d::Zero();
    // coeffs.row(0) << 0.5,0.7,0.8; // First coefficients --> constant.

    Polynomial lin_velocity_ =
        Polynomial(coeffs.transpose(), 0., horizon_reset_);
    Polynomial ang_rotation_ =
        Polynomial(coeffs.transpose(), 0., horizon_reset_);
    pcVel_.add_curve(lin_velocity_);
    pcRot_.add_curve(ang_rotation_);
  }

  ObserverData getObervation() { return odata_; }

  void reset(const std::vector<double> &q);
  void update_final_pose(const mjModel *model, const mjData *data);
  void update_filter(const mjModel *model, const mjData *data);
  double get_yaw_filtered() { return odata_.filtered_pose[5]; };
  void update_collision_status(const bool &status) {
    odata_.collision_status = status;
  };
  void update_contact_status(const ContactData &contactData);
  void update_ref_curve(const std::vector<double> &actions);
  void reset_curves(const std::vector<double> &q);
};

#endif // OBSERVER_H
