
#ifndef COLLISION_CHECKER_HPP
#define COLLISION_CHECKER_HPP

#include "mujoco/mujoco.h"
#include <unordered_map>

#include <hpp/fcl/internal/tools.h>
#include <hpp/fcl/collision.h>
#include <hpp/fcl/distance.h>

#include <Eigen/Core>
#include <Eigen/Dense>

class CollisionChecker {
 public:
  // Constructor
  CollisionChecker() {createCollisionObjects();}

  // Destructor
  ~CollisionChecker() {}

  void createCollisionObjects() {
    // TODO: scrapping from .xml
    // Modify behaviour when dealing with multiple environment.
    // Trunk collision objects.
    body_objects["trunk_01"] = new hpp::fcl::Box(0.125, 0.04, 0.057);
    body_objects["trunk_02"] = new hpp::fcl::Cylinder(0.058, 0.125);
    body_objects["trunk_03"] = new hpp::fcl::Cylinder(0.058, 0.125);
    body_objects["trunk_04"] = new hpp::fcl::Box(0.005, 0.06, 0.05);

    // Hip collision objects.
    body_objects["FR_hip"] = new hpp::fcl::Cylinder(0.04,0.04);
    body_objects["FL_hip"] = new hpp::fcl::Cylinder(0.04,0.04);
    body_objects["HR_hip"] = new hpp::fcl::Cylinder(0.04,0.04);
    body_objects["HL_hip"] = new hpp::fcl::Cylinder(0.04,0.04);

    tf1_ = hpp::fcl::Transform3f::Identity(); // Bodies.
    tf2_ = hpp::fcl::Transform3f::Identity(); // Environment.

    // Environment
    env_objects["floor"] = new hpp::fcl::Box(4.,4.,0.1);
    tf2_.setTranslation(hpp::fcl::Vec3f(0.,0.,-0.1));

    // use distance function in hppfcl
    request_distance_ = hpp::fcl::DistanceRequest(false, 0., 0.);
  }

  void collision(mjModel* model, mjData* data) {

    for (auto& elem : body_objects) {
      int geomId = mj_name2id(model, mjOBJ_GEOM, elem.first.c_str());
      if (geomId != -1) {
        // Retrieve position from geom_xpos
        mjtNum* geomPos = &data->geom_xpos[3 * geomId];

        // Retrieve orientation from geom_xmat
        mjtNum* geomMat = &data->geom_xmat[9 * geomId];
        // Convert orientation matrix to quaternion
        mjtNum quat[4];
        mju_mat2Quat(quat, geomMat);

        tf1_.setTransform(
            hpp::fcl::Quaternion3f(quat[0], quat[1], quat[2], quat[3]),
            hpp::fcl::Vec3f(geomPos[0], geomPos[1], geomPos[2]));

        // Collision check
        res_distance_.clear();
        hpp::fcl::distance(elem.second, tf1_, env_objects.at("floor"), tf2_,
                           request_distance_, res_distance_);
        if (res_distance_.min_distance <= 0) {
          std::cout << "Found collision with : " << elem.first.c_str() << std::endl;
        }
      } else {
        throw std::runtime_error(
            "Geometry does not exist in Collision Checker.");
      }
    }
  }

 private:
  hpp::fcl::GJKSolver solver;
  // Create a list to store the geometries
  hpp::fcl::Cylinder cylinder4;
  std::unordered_map<std::string, hpp::fcl::ShapeBase*> body_objects;
  std::unordered_map<std::string, hpp::fcl::ShapeBase*> env_objects;

  // Use hpp-fcl distance function to evaluate the collision (faster).
  hpp::fcl::DistanceRequest request_distance_;
  hpp::fcl::DistanceResult res_distance_;

  hpp::fcl::Transform3f tf1_, tf2_;
};

#endif  // COLLISION_CHECKER_HPP
