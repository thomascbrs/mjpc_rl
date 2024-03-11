#include "mujoco/mujoco.h"

#ifndef SETTINGS_H_
#define SETTINGS_H_

// settings
struct Settings {
  double kp = 5.;
  double kd = 0.2;
  double timestep = 0.002;
  double timestep_planner = 1.0e-2;
  double horizon_planner = 0.24;
  double n_steps = horizon_planner / timestep_planner + 1 ;
  int k_mpc = 10; // Not used for now.

  void set_settings(mjModel* model) {
    // Access the simulation options
    mjOption* options = &model->opt;
    options->timestep = timestep;
    options->integrator = mjINT_EULER;  // mjINT_RK4
    options->cone = mjCONE_ELLIPTIC;    // mjCONE_PYRAMIDAL
    options->jacobian = mjJAC_AUTO;
    options->solver = mjSOL_NEWTON;  // mjSOL_CG, mjSOL_PGS

    options->iterations = 50;   // 100
    options->tolerance = 1e-8;  // 1e-8

    options->noslip_tolerance = 1e-6;
    options->noslip_iterations = 0;  // 3
    options->mpr_tolerance = 1e-6;

    // Contact settings.
    options->enableflags = mjENBL_OVERRIDE;
    options->o_solimp[0] = 0.28;
    options->o_solimp[1] = 0.65;
    options->o_solimp[2] = 0.02;
    options->o_margin = 0.001;
    options->o_solref[0] = 0.005;
    options->disableflags =
        mjDSBL_LIMIT | mjDSBL_EQUALITY | mjDSBL_FILTERPARENT | mjDSBL_MIDPHASE;

    // Objectives : Break/creates contacts a lot more at slow velocity so that
    // the robot does not fall.

    // Previous working settings.
    // Reminder. If not set manually mjENBL_OVERRIDE, Contact parameters are
    // taking as an average from between the parameters of the surfaces in
    // contact (ie floor and foot).

    // Set before this->NominalTrajectoryCustom(horizon, pool) only for the 1st
    // iteration. and remove from the settings to get the soft contact
    // behaviour.
    //   model->opt.o_margin = 0.00;
    //   model->opt.o_solimp[0] = 0.90;
    //   model->opt.o_solimp[1] = 0.949999;
    //   model->opt.o_solimp[2] = 0.001;
    //   model->opt.o_solref[0] = 0.02;
    // And before : this->IterationCustom(horizon, pool);
    //   model->opt.enableflags = mjENBL_OVERRIDE;
    //   model->opt.o_solimp[0] = 0.45;
    //   model->opt.o_solimp[1] = 0.7;
    //   model->opt.o_solimp[2] = 0.02;
    //   model->opt.o_margin = 0.001;
    //   model->opt.o_solref[0] = 0.005;
    //   model->opt.disableflags = mjDSBL_LIMIT | mjDSBL_EQUALITY |
    //   mjDSBL_FILTERPARENT | mjDSBL_MIDPHASE;
    //
    // Hypothesis : The first contact are soft and create a lot of penetration
    // after the first rollout which then trigger contact creation after the
    // first, ending up in a nice motion at slow velocity. I think this is more
    // like a bug.

    // New setting. Divide o_slimp[0] by 2.
    // Need to add a new iteration every 5 iterations.
    // Hypothesis : 1 more iteration generates way more information in the
    // derivative.

    // All of these behaviour do not appear and seem to converge for 2 iteration
    // of MPC.
  }
};

#endif  // SETTINGS_H_
