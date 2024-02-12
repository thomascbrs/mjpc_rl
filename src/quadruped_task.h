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
// #include "ndcurves/bezier_curve.h"
#include "ndcurves/polynomial.h"
#include "ndcurves/piecewise_curve.h"
#include <absl/strings/match.h>
#include <mujoco/mujoco.h>
#include <string>

typedef Eigen::Vector3d point3;
typedef std::vector<point3, Eigen::aligned_allocator<point3> > t_point3;

typedef ndcurves::polynomial<double, double, true, point3> Polynomial;
typedef ndcurves::piecewise_curve<double, double, true,point3 ,point3, Polynomial> PieceWise;

class QuadrupedTask : public mjpc::Task {
public:
  std::string Name() const override;
  std::string XmlPath() const override;
  class ResidualFn : public mjpc::BaseResidualFn {
  public:
    explicit ResidualFn(const QuadrupedTask *task, int current_mode = 0)
        : mjpc::BaseResidualFn(task), current_mode_(current_mode) {

      // Update the container of points.
      for (int i = 0; i < 3; i++) {
        cp_lin.push_back(Eigen::Vector3d(0., 0., 0.));
        cp_rot.push_back(Eigen::Vector3d(0., 0., 0.));
        cp_pos.push_back(Eigen::Vector3d(0., 0., 0.));
      }

      // Create the polynomial curve, constraining the velocity.
      curve_ = Polynomial(
          cp_pos.begin(), cp_pos.end(),0.,0.4);
      lin_velocity_ = Polynomial(
          cp_lin.begin(), cp_lin.end(),0.,0.4);
      ang_rotation_ = Polynomial(
          cp_rot.begin(), cp_rot.end(),0.,0.4);
      ang_velocity_ = ang_rotation_.compute_derivate(1);


      pcVel_.add_curve(lin_velocity_);
      pcRot_.add_curve(ang_rotation_);

      C2 << 0,1.,0,0,0,0,0,0,0,0,-1., 0,
            0,0,1.,0,0,0,0,0,0,0, 0,-1.,
            0,0,0,0,1.,0.,0,-1.,0.,0,0,0,
            0,0,0,0,0.,1.,0,0.,-1.,0,0,0;
      // C2 << 0,1,0,   0,0,0,   0,-1,0,   0,0,0,
      //       0,0,1.,  0,0,0,   0,0,-1,   0,0,0.,
      //       0,0,0,   0,1.,0   ,0,0,0,   0,-1,0,
      //       0,0,0,   0,0.,1.,  0,0.,0., 0,0,-1.;
      // C2 << 0,1,0,   0,-1,0,   0,0,0,   0,0,0,
      //       0,0,1,   0,0,-1,   0,0,0,   0,0,0,
      //       0,0,0,   0,0,0,    0,1,0,   0,-1,0,
      //       0,0,0,   0,0,0,    0,0,1,   0,0,-1.;
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
    /// @brief Update the ref curves
    /// @param param
    void updateCurves(const std::vector<double>::iterator start, const std::vector<double>::iterator end);

  private:
    friend class QuadrupedTask;
    int current_mode_;
    std::unordered_map<std::string, int> param_index_;
    std::unordered_map<std::string, int> param_size_;

    // Creation of the container of control points
    std::vector<Eigen::Vector3d> cp_pos;
    std::vector<Eigen::Vector3d> cp_lin;
    std::vector<Eigen::Vector3d> cp_rot;

    double t0 = 0.;
    double t1 = 0.4;
    int n_update = 0.;

    // Bezier curves
    Polynomial curve_; // Won't be used in current setup.
    Polynomial lin_velocity_;
    Polynomial ang_velocity_;
    Polynomial ang_rotation_;

    std::vector<double> try_={0.,1.};
    Eigen::Matrix<double, 4,12> C2;

    // Trajectories
    PieceWise pcVel_;
    PieceWise pcRot_;
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
