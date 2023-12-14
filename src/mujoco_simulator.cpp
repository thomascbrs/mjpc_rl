#include "mujoco_simulator.h"
#include <iostream>

// Define the task outisde the class function.
mjpc::QuadrupedFlat *task_;

MujocoSimulator::MujocoSimulator(const char *modelFile)
    : model(nullptr), data(nullptr) {
  // Load Mujoco model
  char loadError[1024] = "";
  constexpr int kErrorLength = 1024;
  // mjModel* model =
  // mj_loadXML("/home/thomas_cbrs/Desktop/edin_23/mujoco_rl/unitree_a1/a1.xml",
  // nullptr, loadError, kErrorLength);
  model = mj_loadXML(modelFile, nullptr, loadError, kErrorLength);
  if (!model) {
    std::cerr << "Error loading Mujoco model: " << loadError << std::endl;
  }

  // Access the simulation options
  mjOption *options = &model->opt;
  options->integrator = mjINT_EULER; // mjINT_RK4
  options->cone = mjCONE_ELLIPTIC;
  options->jacobian = mjJAC_AUTO;
  options->solver = mjSOL_NEWTON;
  options->timestep = 0.002;
  options->iterations = 100;
  options->tolerance = 1e-8;
  options->noslip_tolerance = 1e-6;
  options->noslip_iterations = 2;
  options->mpr_tolerance = 1e-6;

  // Initialize Mujoco simulation
  data = mj_makeData(model);

  // Define reference joint configuration and assign values to data->qpos
  // q0_ = {0.0, 0.9, 0.3,   // Joint 1
  //         0.0, 0.9, 0.3,   // Joint 2
  //         0.0, 0.9, 0.3,   // Joint 3
  //         0.0, 0.9, 0.3};  // Joint 4
  q0_ = {0., -0., 0.3, 1., 0., 0., 0., 0., 0., 0.,
         0., 0.,  0.,  0., 0., 0., 0., 0., 0.};

  for (int i = 0; i < model->nv; ++i) {
    data->qpos[i] = q0_[i];
  }

  // Mujoco visualisation
  // init GLFW, create window, make OpenGL context current, request v-sync
  glfwInit();
  window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  // Declare mjvPerturb variable
  mjvPerturb pert;
  mjv_defaultCamera(&cam);
  mjv_defaultPerturb(&pert);
  mjv_defaultOption(&opt);
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(model, &scn, 1000);
  mjr_makeContext(model, &con, mjFONTSCALE_100);

  // ----- iLQG planner ----- //
  mjpc::iLQGPlanner planner;

  // Params
  double horizon_ = 0.5;
  double timestep_ = 1.0e-2;
  int kMaxTrajectoryHorizon = 128;

  // planning steps
  steps_ = horizon_ / timestep_ + 1;
  task_ = new mjpc::QuadrupedFlat();
  task_->Reset(model);

  // Initialize State.
  state_.Initialize(model);
  state_.Allocate(model);
  state_.Reset();
  state_.Set(model, data);

  // Initialise planner.
  planner.Initialize(model, *task_);
  planner.Allocate();
  planner.Reset(kMaxTrajectoryHorizon);

  mjpc::ThreadPool planner_pool(1);
  // ---- -settings ----- //
  std::atomic<bool> exitrequest(false);
  std::atomic<int> uiloadrequest(0);

  // main loop
  // while (!exitrequest.load()) {
  //     if (model_ && uiloadrequest.load() == 0) {
  //     PlanIteration(&planner_pool);
  //     }
  // }  // exitrequest sent -- stop planning
}

MujocoSimulator::~MujocoSimulator() {
  if (model)
    mj_deleteModel(model);
  if (data)
    mj_deleteData(data);
}

// simple controller applying damping to each dof
void mycontroller(const mjModel *m, mjData *d) {
  if (m->nu == m->nv) {
    mju_scl(d->ctrl, d->qvel, -0.8, m->nv);
  }
}

// sensor
extern "C" {
void sensor(const mjModel *m, mjData *d, int stage);
}

// sensor callback
void MujocoSimulator::sensor(const mjModel *model, mjData *data, int stage) {
  //   if (stage == mjSTAGE_ACC) {
  //     if (!sim->agent->allocate_enabled && sim->uiloadrequest.load() == 0) {
  //       if (sim->agent->IsPlanningModel(model)) {
  //         // the planning thread and rollout threads don't need
  //         // synchronization when using PlanningResidual.
  //         const mjpc::ResidualFn* residual = sim->agent->PlanningResidual();
  //         residual->Residual(model, data, data->sensordata);
  //       } else {
  //         // this residual is used by the physics thread and the UI thread
  //         (for
  //         // plots), and is run with a shared lock, to safely run with
  //         changes to
  //         // weights and parameters
  //         sim->agent->ActiveTask()->Residual(model, data, data->sensordata);
  //       }
  //     }
  //   }
  if (stage == mjSTAGE_ACC) {
    task_->Residual(model, data, data->sensordata);
  }
}

void MujocoSimulator::initialize() {
  // Additional initialization steps if needed
}

void MujocoSimulator::runSimulation(int numSteps) {
  // Set control callback
  // mjcb_control = mycontroller;

  // Set sensor callback
  mjcb_sensor = &MujocoSimulator::sensor;

  mj_step(model, data);
  while (!glfwWindowShouldClose(window)) {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually
    //  can, this loop will finish on time for the next frame to be rendered at
    //  60 fps. Otherwise add a cpu timer and exit this loop when it is time to
    //  render.
    mjtNum simstart = data->time;
    while (data->time - simstart < 1.0 / 60.0) {
      if (data->time > 0.) {
        mju_zero(data->ctrl, model->nu);
        mju_zero(data->qfrc_applied, model->nv);
        mju_zero(data->xfrc_applied, 6 * model->nbody);
        for (int i = 0; i < model->nu; ++i) {
          double error = q0_[i + 7] - data->qpos[i + 7];
          double vel_error =
              data->qvel[i +
                         6]; // Assuming you have access to velocity information
          // double control_signal = data_i->qfrc_inverse[i+6] + kp_ * error -
          // kd_ * vel_error;
          double control_signal = kp_ * error - kd_ * vel_error;

          // Apply control signal to actuators or joints
          data->ctrl[i] = control_signal;
        }
      }
      mj_step(model, data);
    }

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(model, data, &opt, NULL, &cam, mjCAT_ALL, &scn);
    mjr_render(viewport, &scn, &con);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  // close GLFW, free visualization storage
  glfwTerminate();
  mjv_freeScene(&scn);
  mjr_freeContext(&con);
}

std::vector<std::vector<double>>
MujocoSimulator::getLoggedJointPositions() const {
  return jointPositionsLog;
}