#ifndef OBSERVER_H
#define OBSERVER_H


#include <iostream>
#include "mjpc/utilities.h"
#include "mujoco/mujoco.h"

#include "filter.h"
#include "types.h"
#include "utils.h"
#include "contact_data.h"

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
  // std::array<double, 6> filtered_acc = {0.0};
  std::unordered_map<std::string, std::array<double, 3>> feet_pos;
  std::unordered_map<std::string, std::array<double, 3>> feet_vel;
  std::unordered_map<std::string, std::array<double, 3>> lfeet_pos;
  std::unordered_map<std::string, std::array<double, 3>> lfeet_vel;

  // Collision status
  bool collision_status = false;
  std::unordered_map<std::string, int> contact_status;
};

class Observer {
 private:
  ObserverData odata_;
  Matrix3d R_tmp;
  Vector3d pos_tmp;
  const mjtNum axis[3];  // z-axis (yaw)

  std::vector<double> cutoff_pos = {2., 2., 2., 2., 2., 2.};
  double fs_pos = 1 / 0.002;
  int order_pos = 1;
  std::vector<double> cutoff_vel = {2., 2., 2., 2., 2., 2.};
  double fs_vel = 1 / 0.002;
  int order_vel = 1;
  Filter filter_pos_;
  Filter filter_vel_;

 public:
  Observer(const std::vector<std::string> &fnames)
      : odata_(),
        R_tmp(Matrix3d::Identity()),
        pos_tmp(Vector3d::Zero()),
        axis{0., 0., 1.},
        filter_pos_(cutoff_vel, fs_vel, order_vel),
        filter_vel_(cutoff_vel, fs_vel, order_vel)  {
    for (const auto &name : fnames) {
      odata_.foot_names.push_back(name);
      odata_.feet_vel[name] = {0., 0., 0.};
      odata_.feet_pos[name] = {0., 0., 0.};
      odata_.contact_status[name] = 1; // Initialisation in contact.
    }
  }

  ObserverData getObervation() { return odata_; }

  void reset();
  void update_final_pose(const mjModel *model, const mjData *data);
  void update_filter(const mjModel *model, const mjData *data);
  double get_yaw_filtered(){return odata_.filtered_pose[5];};
  void update_collision_status(const bool& status){odata_.collision_status = status;};
  void update_contact_status(const ContactData &contactData);

};

#endif  // OBSERVER_H
