#include "mujoco_simulator.h"
#include "quadruped_task.h"
#include <iostream>

// Define the task outisde the class function.
// mjpc::QuadrupedFlat *task_;
// mjpc::QuadrupedHill *task_;
// mjpc::Cartpole *task_;
QuadrupedTask *task_;

// State and planner.
mjpc::State state_;
mjpc::iLQGPlanner planner;

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

  // Adjust camera distance
  cam.azimuth = 90.0;  // Set azimuth angle
  cam.elevation = -20.0;  // Set elevation angle
  cam.distance = 2.;  // Set camera distance to 1.0

  // Params
  double horizon_ = 0.35;
  double timestep_ = 1.0e-2;
  double timestep_simu = 0.002;
  int kMaxTrajectoryHorizon = 128;
  // model->opt.timestep = timestep_;

  // planning steps
  steps_ = horizon_ / timestep_ + 1;

  // Define tasks.
  // task_ = new mjpc::QuadrupedFlat();
  // task_ = new mjpc::QuadrupedHill();
  // task_ = new mjpc::Cartpole();
  task_ = new QuadrupedTask();
  task_->Reset(model);

  // set data
  mj_forward(model, data);

  // Initialize State.
  state_.Initialize(model);
  state_.Allocate(model);
  state_.Reset();
  state_.Set(model, data);

  // Initialise planner.
  planner.Initialize(model, *task_);
  planner.Allocate();
  planner.Reset(kMaxTrajectoryHorizon);
  planner.settings.verbose = 1;

  // cost
  terms_.resize(task_->num_term * kMaxTrajectoryHorizon);
  std::fill(terms_.begin(), terms_.end(), 0.0);
  allocate_enabled = false;
  plan_enabled = true;
  count_ = 0;

  // task_->
  task_->UpdateResidual();

  // Initialize vectors for friction  parameters.
  if (original_friction_values.empty()) {
    original_friction_values.resize(model->ngeom);
    original_solref_values.resize(model->ngeom * 3);
    // Save the original properties for later restoration
    for (int geom_idx = 0; geom_idx < model->ngeom; ++geom_idx) {
        original_friction_values[geom_idx] = model->geom_friction[geom_idx];
        original_solref_values[geom_idx * 3] = model->geom_solref[geom_idx * 3];
    }
  }

  // TODO : Modify values of the geom to disable geom params and to plan
  // only with the feet and not with other body parts.
  // Not working for now.
  // Iterate over all geoms
  for (int geom_idx = 0; geom_idx < model->ngeom; ++geom_idx) {
    int geom_name_ptr = model->name_geomadr[geom_idx];

    // Determine the length of the geom name
    size_t name_length = 0;
    while (model->names[geom_name_ptr + name_length] != '\0') {
        ++name_length;
    }

    // Convert the char to a string
    std::string geom_name(&model->names[geom_name_ptr], name_length);

    // Print the string
    std::cout << "Geom " << geom_idx << " Name: " << geom_name << std::endl;
  }

  // start plan thread
  runSimulation(1000);
}

MujocoSimulator::~MujocoSimulator() {
  if (model)
    mj_deleteModel(model);
  if (data)
    mj_deleteData(data);
}

// Function to disable interaction for specific geoms during Jacobian computation
void MujocoSimulator::disableInteractionForGeoms(mjModel* m) {
    // Iterate over the geoms you want to disable
    for (int geom_idx = 0; geom_idx < m->ngeom; ++geom_idx) {
        // Modify relevant geom properties (friction, solref, etc.)
        m->geom_friction[geom_idx] = 1e9;
        m->geom_solref[geom_idx * 3] = 1e9;
        // You may need to adjust other properties based on your specific requirements
    }
}

// Function to enable interaction for specific geoms after Jacobian computation
void MujocoSimulator::enableInteractionForGeoms(mjModel* m) {
    // Iterate over the geoms you disabled
    for (int geom_idx = 0; geom_idx < m->ngeom; ++geom_idx) {
        // Restore original geom properties
        m->geom_friction[geom_idx] = original_friction_values[geom_idx];
        m->geom_solref[geom_idx * 3] = original_solref_values[geom_idx];
        // Restore other properties if necessary
    }
}


// simple controller applying damping to each dof
void mycontroller(const mjModel *m, mjData *d) {
  // if (m->nu == m->nv) {
    // mju_scl(d->ctrl, d->qvel, -0.8, m->nv);
  // }
  // planner.ActionFromPolicy(
  //       d->ctrl, &state_.state()[0],state_.time());
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
  // if (stage == mjSTAGE_ACC) {
  task_->Residual(model, data, data->sensordata);
  // }
}

void MujocoSimulator::PlanIteration(mjpc::ThreadPool* pool) {
  // start agent timer
  auto agent_start = std::chrono::steady_clock::now();

  // plan
  if (!allocate_enabled) {

    // set state
    state_.Set(model, data);
    planner.SetState(state_);

    // copy the task's residual function parameters into a new object, which
    // remains constant during planning and doesn't require locking from the
    // rollout threads
    residual_fn_ = task_->Residual();

    task_->ModifyScene(model, data, &scn);

    if (plan_enabled) {
      // planner policy
      planner.OptimizePolicy(steps_, *pool);

      // compute time
      agent_compute_time_ =
          std::chrono::duration_cast<std::chrono::microseconds>(
              std::chrono::steady_clock::now() - agent_start)
              .count();

      // counter
      count_ += 1;
    } else {
      // rollout nominal policy
      planner.NominalTrajectory(steps_, *pool);

      // set timers
      agent_compute_time_ = 0.0;
    }

    // release the planning residual function
    residual_fn_.reset();
  }
}

// call planner to update nominal policy
void MujocoSimulator::Plan(std::atomic<bool>& exitrequest,
                 std::atomic<int>& uiloadrequest) {
  // instantiate thread pool
  mjpc::ThreadPool pool_planner(planner_threads_);

  // main loop
  while (!exitrequest.load()) {
    if (model && uiloadrequest.load() == 0) {
      PlanIteration(&pool_planner);
    }
  }  // exitrequest sent -- stop planning
}

void MujocoSimulator::initialize() {
  // Additional initialization steps if needed
}

void MujocoSimulator::runSimulation(int numSteps) {

  // Start planner in separate thread.
  std::atomic<bool> exitrequest(false);
  std::atomic<int> uiloadrequest(0);
  mjpc::ThreadPool plan_pool(4);
  // plan_pool.Schedule([this,&exitrequest, &uiloadrequest]() { Plan(exitrequest, uiloadrequest); });

  // Set control callback
  mjcb_control = mycontroller;

  // Set sensor callback
  mjcb_sensor = &MujocoSimulator::sensor;

  int counter_wbc = 0;

  mj_step(model, data);
  while (!glfwWindowShouldClose(window)) {
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually
    //  can, this loop will finish on time for the next frame to be rendered at
    //  60 fps. Otherwise add a cpu timer and exit this loop when it is time to
    //  render.
    mjtNum simstart = data->time;
    while (data->time - simstart < 1.0 / 60.0) {
      if (data->time < 1.) {
        // mju_zero(data->ctrl, model->nu);
        // mju_zero(data->qfrc_applied, model->nv);
        // mju_zero(data->xfrc_applied, 6 * model->nbody);
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
          // data->ctrl[i] = 0.;
        }
      }
      else{
        data->mocap_pos[0] = 0.8;
        data->mocap_pos[1] = 0.;
        data->mocap_pos[2] = 0.25;
        data->mocap_quat[0] = 1.;
        data->mocap_quat[1] = 0.;
        data->mocap_quat[2] = 0.;
        data->mocap_quat[3] = 0.;

        // task_->Residual(model, data, data->sensordata);
        // uiloadrequest.fetch_add(1);
        // task_->Reset(model);
        // planner.task->UpdateResidual();
        // task_->Residual(model, data, data->sensordata);task_->Residual(model, data, data->sensordata);
        if (counter_wbc % 10 == 0){
          // PlanIteration(&plan_pool);
          // set state
          // task_->SetFeatureParameters(model);
          // task_->parameters[0] = -0.9;
          // data->sensordata[1] = data->qpos[0] - 0.9;
          // task_->Reset();
          state_.Set(model, data);
          planner.SetState(state_);
          residual_fn_ = task_->Residual();
          task_->risk = 0.;
          // planner policy
          for (int i = 0;i <= 1; i++){
              double horizon_ = 0.35;
              // Params
              double timestep_ = 1.0e-2;
              int kMaxTrajectoryHorizon = 128;

              // planning steps
              steps_ = horizon_ / timestep_ + 1;
              model->opt.timestep = timestep_;
              planner.OptimizePolicy(steps_, plan_pool);
              // planner.NominalTrajectory(steps_, plan_pool);

              // iteration
              // planner.Iteration(steps_, plan_pool);
          }
        }
        state_.Set(model, data);
        std::cout << "state_.time() : " << state_.time() << std::endl;
        double timestep_simu = 0.002;
        model->opt.timestep = timestep_simu;


        // Direct torques from policy.
        planner.ActionFromPolicy(
          data->ctrl, &state_.state()[0],state_.time(), false);


        // std::vector<double>
        // for (int i = 0; i < model->nu; ++i) {
        //   double error = planner.BestTrajectory()->states[37*1 + 7 + i] - data->qpos[i + 7];
        //   double vel_error = data->qvel[i + 6]; // Assuming you have access to velocity information
        //   // double control_signal = data_i->qfrc_inverse[i+6] + kp_ * error -
        //   // kd_ * vel_error;
        //   double control_signal = kp_ * error - kd_ * vel_error;

        //   // Apply control signal to actuators or joints
        //   data->ctrl[i] += control_signal;
        // }

        // for (int i = 0; i < model->nu; ++i) {
          // data->ctrl[i] = 0.;
          // data->ctrl[i] = planner.BestTrajectory()->actions[i];
        // }
        int counting = 0;
        // std::this_thread::sleep_for(std::chrono::milliseconds(30));
        // data->ctrl[7] = 5.;
        counter_wbc += 1;
      }
      double timestep_simu = 0.002;
      model->opt.timestep = timestep_simu;
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