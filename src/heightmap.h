#ifndef MUJOCO_HEIGHTMAP_H
#define MUJOCO_HEIGHTMAP_H

#include <iostream>
#include <vector>
#include "types.h"


#include "pinocchio/math/rpy.hpp"
#include "pinocchio/spatial/se3.hpp"
#include <Eigen/Geometry>
#include <pinocchio/math/quaternion.hpp>


// Define a structure for a rectangle
struct Rectangle {
  double centerX, centerY;       // Center coordinates
  double halfWidth, halfHeight;  // Half-width and half-height

  // Constructor to initialize the rectangle with center coordinates and
  // half-widths
  Rectangle(double cx, double cy, double dx, double dy)
      : centerX(cx), centerY(cy), halfWidth(dx), halfHeight(dy) {}
};

// Class for Heightmap
class Heightmap {
 public:
  std::vector<std::vector<Rectangle>>
      environments;                   // Vector of vectors of rectangles
  std::vector<Rectangle> startZones;  // limits for reset method.
  std::vector<Rectangle> goalZones;   // limits for reset method.
  int currentEnvironment;             // Current environment flag

  int Nx_ = 13;
  int Ny_ = 7;
  double lx_ = 1.;
  double dx_ = 0.5;
  double ly_ = 0.5;
  // Resize matrix and vector according to header
  Eigen::MatrixXd z_;

  // x Vector
  Eigen::VectorXd xVector;
  Eigen::MatrixXd xx_w;

  // y Vector
  Eigen::VectorXd yVector;
  Eigen::MatrixXd yy_w;

  // Constructor
  Heightmap()
      : currentEnvironment(-1),
        z_(Eigen::MatrixXd::Zero(Nx_, Ny_)), // Initialize current environment flag to -1
        xVector(Eigen::VectorXd::LinSpaced(Nx_, -lx_/2 + dx_,lx_/2 + dx_)),
        xx_w(Eigen::MatrixXd::Zero(Nx_,Ny_)),
        yVector(Eigen::VectorXd::LinSpaced(Ny_, -ly_/2,ly_/2)),
        yy_w(Eigen::MatrixXd::Zero(Nx_,Ny_)){}

  // Function to set the environments
  void setEnvironments(const std::vector<std::vector<Rectangle>>& envs) {
    environments = envs;
  }

  // Function to set the starting zones
  void setStartZones(const std::vector<Rectangle>& zones) {
    startZones = zones;
  }

  // Function to set the goal zones
  void setGoalZones(const std::vector<Rectangle>& zones) { goalZones = zones; }

  // Function to set the current environment
  void setCurrentEnvironment(int env) {
    if (env >= 0 && env < environments.size()) {
      currentEnvironment = env;
    } else {
      // std::cerr << "Error: Invalid environment index." << std::endl;
      throw std::runtime_error("Error: Invalid environment index (Heightmap)");
    }
  }

  // Function to get the current environment
  int getCurrentEnvironment() const { return currentEnvironment; }

  // Function to check if a point is inside the current environment
  double get_height(double x, double y) const {
    if (currentEnvironment == -1) {
      // std::cerr << "Error: No environment is set." << std::endl;
      throw std::runtime_error("Error: No environment is set.");
      return -1;
    }

    for (const Rectangle& rect : environments[currentEnvironment]) {
      if (x >= rect.centerX - rect.halfWidth &&
          x <= rect.centerX + rect.halfWidth &&
          y >= rect.centerY - rect.halfHeight &&
          y <= rect.centerY + rect.halfHeight) {
        return double(0.);  // Inside the rectangle of the current environment
      }
    }

    return double(-1);  // Outside the rectangles of the current environment
  }

  double get_mean_height(double x, double y) const{
    // List of height values.
    int nx = 5;
    int ny = 3;
    Eigen::VectorXd x_list = Eigen::VectorXd::LinSpaced(nx, x-0.01,x+0.01);
    Eigen::VectorXd y_list = Eigen::VectorXd::LinSpaced(ny, y-0.01,y+0.01);
    double mean = 0;
    for (int i = 0;i < nx;i++){
      for (int j = 0;j < ny;j++){
        mean += get_height(x_list(i), y_list(j));
      }
    }
    return abs(mean / (nx * ny));
  }

  std::vector<double> get_heightmap() {
    std::vector<double> sorted_data;
    int index = 0;
    for (int col = 0; col < z_.cols(); ++col) {
      for (int row = 0; row < z_.rows(); ++row) {
        sorted_data.push_back(z_(row, col));
      }
    }
    return sorted_data;
  }

  void update_heightmap(const mjModel* model, const mjData* data) {
    Eigen::Quaterniond quat(data->qpos[3], data->qpos[4], data->qpos[5],
                            data->qpos[6]);  // (w, x, y, z)

    // Convert quaternion to rotation matrix
    Eigen::Matrix3d rotation_matrix = quat.normalized().toRotationMatrix();
    Vector3d rpy = pinocchio::rpy::matrixToRpy(rotation_matrix);
    Matrix3d R_tmp = pinocchio::rpy::rpyToMatrix(0., 0., rpy(2));
    Vector2d q0 = {data->qpos[0], data->qpos[1]};

    // Get the heightmap around the robot.
    for (int i=0;i < Nx_; i++){
      for (int j=0;j < Ny_;j++){
        // Position of the point to query in local frame.
        Vector2d pos = {xVector(i), yVector(j)};
        // Position in world frame.
        Vector2d pos_w = R_tmp.block(0,0,2,2) * pos + q0;
        // Update
        xx_w(i,j) = pos_w(0);
        yy_w(i,j) = pos_w(1);
        z_(i,j) = get_height(pos_w(0), pos_w(1));
      }
    }
  }

  void create_environment1() {
    // Define environments with rectangles
    std::vector<std::vector<Rectangle>> environments = {
        {Rectangle(0.0, 0.0, 4.0, 4.0)},  // Environment 0
        {Rectangle(0.0, 8.0, 2., 2.),
         Rectangle(4.06, 8.0, 2., 2.)},  // Environment 1
        {Rectangle(0.0, 14.0, 2., 2.),
         Rectangle(4.12, 14.0, 2., 2.)},  // Environment 2
        {Rectangle(0.0, 22.0, 2., 2.),
         Rectangle(4.2, 22.0, 2., 2.)},  // Environment 3
        {Rectangle(0.0, 28.0, 2., 2.),
         Rectangle(4.25, 28.0, 2., 2.)},  // Environment 4
        {Rectangle(0.0, 34.0, 2., 2.),
         Rectangle(4.3, 34.0, 2., 2.)},  // Environment 5
        {Rectangle(0.0, 40.0, 2., 2.),
         Rectangle(4.5, 40.0, 2., 2.)},  // Environment 6
        {Rectangle(0.0, 46.0, 2., 2.),
         Rectangle(4., 46.0, 2., 2.)}  // Environment 7
    };

    std::vector<Rectangle> start_zones = {
        Rectangle(0.0, 0.0, 0.25, 0.25),  // Environment 0
        Rectangle(0.5, 8.0, 0.5, 0.5),    // Environment 1
        Rectangle(0.5, 14.0, 0.5, 0.5),   // Environment 2
        Rectangle(0.5, 22.0, 0.5, 0.5),   // Environment 3
        Rectangle(0.5, 28.0, 0.5, 0.5),   // Environment 4
        Rectangle(0.5, 34.0, 0.5, 0.5),   // Environment 5
    };

    std::vector<Rectangle> goal_zones = {
        Rectangle(0.0, 0.0, 3.5, 3.5),    // Environment 0
        Rectangle(3.56, 8.0, 0.5, 0.5),   // Environment 1
        Rectangle(3.62, 14.0, 0.5, 0.5),  // Environment 2
        Rectangle(3.7, 22.0, 0.5, 0.5),   // Environment 3
        Rectangle(3.75, 28.0, 0.5, 0.5),  // Environment 4
        Rectangle(3.8, 34.0, 0.5, 0.5),   // Environment 5
    };

    // Set environments for the heightmap
    setEnvironments(environments);
    setStartZones(start_zones);
    setGoalZones(goal_zones);
  }

  void create_environment_baseline() {
    // Define environments with rectangles
    std::vector<std::vector<Rectangle>> environments = {
        {Rectangle(0.0, 0.0, 4.0, 4.0)},  // Environment 0
        {Rectangle(0.0, 0.0, 4.0, 4.0)},  // Environment 0
        {Rectangle(0.0, 0.0, 4.0, 4.0)},  // Environment 0
    };

    std::vector<Rectangle> start_zones = {
        Rectangle(0.0, 0.0, 0.25, 0.25),  // Environment 0
        Rectangle(0.0, 0.0, 0.25, 0.25),  // Environment 0
        Rectangle(0.0, 0.0, 0.25, 0.25),  // Environment 0
    };

    std::vector<Rectangle> goal_zones = {
        Rectangle(0.3, 0.0, 0.5, 0.5),    // Environment 0
        Rectangle(0.7, 0., 0.5, 0.2),   // Environment 1
        Rectangle(2.,  0., 0.5, 2.),  // Environment 2
    };

    // Set environments for the heightmap
    setEnvironments(environments);
    setStartZones(start_zones);
    setGoalZones(goal_zones);
  }

};

#endif // MUJOCO_HEIGHTMAP_H