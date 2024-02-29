#include "mjpc/planners/ilqg/planner.h"

class CustomiLQGPlanner : public mjpc::iLQGPlanner {

private:
  int num_trajectory_;

public:
    void InitializeCustom(mjModel* model, const mjpc::Task& task) {
      num_trajectory_ = mjpc::GetNumberOrDefault(10, model, "ilqg_num_rollouts");
      this->Initialize(model, task);
    }
    // Override the Iteration function
    void IterationCustom(int horizon, mjpc::ThreadPool& pool);
    void NominalTrajectoryCustom(int horizon, mjpc::ThreadPool& pool);
    void OptimizePolicyCustom(int horizon, mjpc::ThreadPool& pool);
};


//---------------------------------- enum types (mjt) ----------------------------------------------

// typedef enum mjtDisableBit_ {     // disable default feature bitflags
//   mjDSBL_CONSTRAINT   = 1<<0,     // entire constraint solver
//   mjDSBL_EQUALITY     = 1<<1,     // equality constraints
//   mjDSBL_FRICTIONLOSS = 1<<2,     // joint and tendon frictionloss constraints
//   mjDSBL_LIMIT        = 1<<3,     // joint and tendon limit constraints
//   mjDSBL_CONTACT      = 1<<4,     // contact constraints
//   mjDSBL_PASSIVE      = 1<<5,     // passive forces
//   mjDSBL_GRAVITY      = 1<<6,     // gravitational forces
//   mjDSBL_CLAMPCTRL    = 1<<7,     // clamp control to specified range
//   mjDSBL_WARMSTART    = 1<<8,     // warmstart constraint solver
//   mjDSBL_FILTERPARENT = 1<<9,     // remove collisions with parent body
//   mjDSBL_ACTUATION    = 1<<10,    // apply actuation forces
//   mjDSBL_REFSAFE      = 1<<11,    // integrator safety: make ref[0]>=2*timestep
//   mjDSBL_SENSOR       = 1<<12,    // sensors
//   mjDSBL_MIDPHASE     = 1<<13,    // mid-phase collision filtering
//   mjDSBL_EULERDAMP    = 1<<14,    // implicit integration of joint damping in Euler integrator

//   mjNDISABLE          = 15        // number of disable flags
// } mjtDisableBit;


// typedef enum mjtEnableBit_ {      // enable optional feature bitflags
//   mjENBL_OVERRIDE     = 1<<0,     // override contact parameters
//   mjENBL_ENERGY       = 1<<1,     // energy computation
//   mjENBL_FWDINV       = 1<<2,     // record solver statistics
//   mjENBL_INVDISCRETE  = 1<<3,     // discrete-time inverse dynamics
//   mjENBL_SENSORNOISE  = 1<<4,     // add noise to sensor data
//                                   // experimental features:
//   mjENBL_MULTICCD     = 1<<5,     // multi-point convex collision detection
//   mjENBL_ISLAND       = 1<<6,     // constraint island discovery

//   mjNENABLE           = 7         // number of enable flags
// } mjtEnableBit;
