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

#include "mjpc/task.h"
#include "ndcurves/bezier_curve.h"
#include <absl/strings/match.h>
#include <mujoco/mujoco.h>
#include <string>

class QuadrupedTask : public mjpc::Task {
public:
  std::string Name() const override;
  std::string XmlPath() const override;
  class ResidualFn : public mjpc::BaseResidualFn {
  public:
    explicit ResidualFn(const QuadrupedTask *task, int current_mode = 0)
        : mjpc::BaseResidualFn(task), current_mode_(current_mode) {
      // Initialize Bezier points;
      // P0 = Eigen::Vector3d(0.007, 0.0, 0.243);
      // P1 = Eigen::Vector3d(0.656, 0.0, 0.009);
      // P2 = Eigen::Vector3d(1.764, 0.0, 0.209);
      // P3 = Eigen::Vector3d(0.756, 0.0, 0.938);
      // P4 = Eigen::Vector3d(1.678, 0.0, 0.052);
      // P5 = Eigen::Vector3d(2.801, 0.0, 0.324);

      P0 = Eigen::Vector3d(0., 0.0, 0.245);
      P1 = Eigen::Vector3d(0.1, 0.0, 0.245);
      P2 = Eigen::Vector3d(0.3, 0.0, 0.245);
      P3 = Eigen::Vector3d(0.5, 0.0, 0.5);
      P4 = Eigen::Vector3d(0.8, 0.0, 0.245);
      P5 = Eigen::Vector3d(1.1, 0.0, 0.28);
      P6 = Eigen::Vector3d(1.3, 0.0, 0.245);

      // Update the container of points.
      cp.push_back(P0);
      cp.push_back(P1);
      cp.push_back(P2);
      cp.push_back(P3);
      cp.push_back(P4);
      cp.push_back(P5);
      cp.push_back(P6);

      // Create the Bezier curve and its derivatives.
      curve_ = ndcurves::bezier_curve<double, double, true, Eigen::Vector3d>(
          cp.begin(), cp.end());
      curve_vel_ = curve_.compute_derivate(1);
      curve_acc_ = curve_.compute_derivate(2);
    }

    // --------------------- Residuals for quadruped task --------------------
    void Residual(const mjModel *model, const mjData *data,
                  double *residual) const override;

    // Update function override
    void Update() override;
    void getPitch(double pitch[1], double wpitch[1], double t) const;
    // TODO : Either a function get parameters indexes, called at each iteration
    // updating [startIdx, endIdx]. Or a dictionnary containing these values
    // computed only once (require modifying mjpc::Task and adding
    // std::unorder_map)
    void ParameterIndexes(int indexes[2], const mjModel *model,
                          const std::string_view name) const;

  private:
    friend class QuadrupedTask;
    int current_mode_;
    std::unordered_map<std::string, int> param_index_;
    std::unordered_map<std::string, int> param_size_;

    // Control points
    Eigen::Vector3d P0;
    Eigen::Vector3d P1;
    Eigen::Vector3d P2;
    Eigen::Vector3d P3;
    Eigen::Vector3d P4;
    Eigen::Vector3d P5;
    Eigen::Vector3d P6;

    // Creation of the container of control points
    std::vector<Eigen::Vector3d> cp;

    // Bezier curves
    ndcurves::bezier_curve<double, double, true, Eigen::Vector3d> curve_;
    ndcurves::bezier_curve<double, double, true, Eigen::Vector3d> curve_vel_;
    ndcurves::bezier_curve<double, double, true, Eigen::Vector3d> curve_acc_;
  };
  QuadrupedTask() : residual_(this) {}
  void TransitionLocked(mjModel *model, mjData *data) override;
  void SetParameters(const mjModel *model);

  // draw task-related geometry in the scene
  void ModifyScene(const mjModel *model, const mjData *data,
                   mjvScene *scene) const override;

protected:
  std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
    return std::make_unique<ResidualFn>(this, residual_.current_mode_);
  }
  ResidualFn *InternalResidual() override { return &residual_; }

private:
  friend class ResidualFn;
  ResidualFn residual_;
};
