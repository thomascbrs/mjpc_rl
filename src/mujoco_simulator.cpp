#include "mujoco_simulator.h"
#include "utils.cpp"
#include <iostream>
#include <thread>

// simple controller applying damping to each dof
void mycontroller(const mjModel *m, mjData *d) {
  // if (m->nu == m->nv) {
  // mju_scl(d->ctrl, d->qvel, -0.8, m->nv);
  // }
  // planner.ActionFromPolicy(
  //       d->ctrl, &state_.state()[0],state_.time());
}

// Logger path.
std::string filename = "/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/log/tmp.csv";

MujocoSimulator::MujocoSimulator(int n_threads, bool rendering, bool logging, const char *modelFile)
    : model(nullptr), data(nullptr), foot_names_{"FR", "FL", "HR", "HL"},
      mcontactData(foot_names_, 0.002),
      observer(foot_names_),
      plan_pool(n_threads)
      {
  if (n_threads == 1 && !flag_thread_local){
    throw std::runtime_error("1 thread selected. Flag thread only should be activated during conpilation.");
  }
  if (n_threads != 1 && flag_thread_local){
    throw std::runtime_error("Multi-threading selected. Flag thread only should be de-activated during conpilation.");
  }

  model = load_model(modelFile);
  settings.set_settings(model);

  // Set flags
  RENDERING_ = rendering;
  LOGGING_ = logging;

  // Initialize Mujoco simulation
  data = mj_makeData(model);

  col = CollisionChecker();
  // Define reference configuration
  q0_ << 0., -0., 0.3,  // position
      1., 0., 0., 0.,  // orientation
      0., 0., 0.,      // FR
      0., 0., 0.,      // FL
      0., 0., 0.,      // HR
      0., 0., 0.;      // HL

  // Assign q0_ to data->qpos
  for (int i = 0; i < model->nq; ++i) {
    data->qpos[i] = q0_[i];
  }

  // General simulation parameters
  mcontactData.dt_simu = settings.timestep; // TODO, find a better way.

  // Define tasks.
  task_ = new QuadrupedTask();
  task_->Reset(model);
  task_->SetParameters(model);

  // Set data
  mj_forward(model, data);

  // Initialize State.
  state_.Initialize(model);
  state_.Allocate(model);
  state_.Reset();
  state_.Set(model, data);

  // Initialise planner.
  planner.InitializeCustom(model, *task_);
  // planner.Initialize(model, *task_);
  planner.Allocate();
  planner.Reset(settings.n_steps);
  task_->num_trace = 0;
  // planner.settings.verbose = 1;
  planner.settings.fd_tolerance = 1.0e-6;
  task_->UpdateResidual();

  // cost
  terms_.resize(task_->num_term * settings.n_steps);
  std::fill(terms_.begin(), terms_.end(), 0.0);


  // task_->

  foot_names_ = {"FR", "FL", "HR", "HL"};

  if (RENDERING_) {
    infos_models(model); // Print infos in terminal.
    initialize_viewer();
  }
  if (LOGGING_){
    logger_.Initialize(foot_names_, settings.timestep_planner, settings.n_steps, settings.k_mpc, settings.timestep);
  }

  // Set engine callbacks
  mjcb_control = mycontroller;
  mjcb_sensor = &MujocoSimulator::sensor;

  // Initialisation
  planner.UpdateNumTrajectoriesFromGUI();
  put_robot_on_floor(200, q0_.tail(12));
  col.collision(model, data);
}

void MujocoSimulator::reset(Eigen::VectorXd q0) {
  if (q0.size() != 19) {
    throw std::runtime_error("q0 should be size 19");
  }
  mj_resetData(model,data);
  for (int i = 0; i < model->nq; ++i) {
    data->qpos[i] = q0[i];
  }

  // Reset task
  task_->Reset(model);
  task_->SetParameters(model);

  // Reset state
  state_.Reset();
  state_.Set(model, data);

  // Reset planner
  planner.Reset(settings.n_steps);
  task_->UpdateResidual();

  mj_step(model, data);
  update_viewer();

  put_robot_on_floor(200,q0_.tail(12));
  simstart = data->time;
}

MujocoSimulator::~MujocoSimulator() {
  // delete plan_pool; // Release the allocated memory in the destructor
  if (model)
    mj_deleteModel(model);
  if (data)
    mj_deleteData(data);
  if (task_)
    delete task_;
}

void MujocoSimulator::print_planner_timings() {
  std::cout << "\nTotal time [ms] : " << 1e-3*planner.nominal_compute_time << std::endl;
  std::cout << "Model derivative [ms] : " << 1e-3*planner.model_derivative_compute_time << std::endl;
  std::cout << "Cost derivative [ms] : " << 1e-3*planner.cost_derivative_compute_time << std::endl;
  std::cout << "Rollout [ms] : " << 1e-3*planner.rollouts_compute_time << std::endl;
  std::cout << "Backward pass [ms] : " << 1e-3*planner.backward_pass_compute_time << std::endl;
  std::cout << "Policy update [ms] : " << 1e-3*planner.policy_update_compute_time << std::endl;
}

// sensor callback
void MujocoSimulator::sensor(const mjModel *model, mjData *data, int stage) {
  // std::cout << "iter : " << data->solver_niter[0] << std::endl;
  if (stage == mjSTAGE_ACC) {
    task_->Residual(model, data, data->sensordata);
  }
}

void MujocoSimulator::initialize_viewer() {
  // Mujoco visualisation
  // init GLFW, create window, make OpenGL context current, request v-sync
  glfwInit();
  window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjvPerturb pert;  // Declare mjvPerturb variable
  mjv_defaultCamera(&cam);
  mjv_defaultPerturb(&pert);
  mjv_defaultOption(&opt);

  // Flags options.
  // opt.flags[mjVIS_CONTACTPOINT] = 1;
  // opt.flags[mjVIS_CONTACTFORCE] = 1;
  // opt.flags[mjVIS_CONTACTSPLIT] = 1;
  // opt.flags[mjVIS_CONSTRAINT] = 1;
  mjr_defaultContext(&con);

  // create scene and context
  mjv_makeScene(model, &scn, 1000);
  mjr_makeContext(model, &con, mjFONTSCALE_100);

  // Adjust camera distance
  cam.azimuth = 70.0;     // Set azimuth angle
  cam.elevation = -20.0;  // Set elevation angle
  cam.distance = 3.5;     // Set camera distance to 1.0
}

void MujocoSimulator::update_viewer() {
  // Get trunk posiiton and update the camera position.
  int trunkBodyId = mj_name2id(model, mjOBJ_BODY, "trunk");
  // Adjust the camera position based on the trunk body position
  cam.lookat[0] = data->xpos[trunkBodyId * 3];
  cam.lookat[1] = data->xpos[trunkBodyId * 3 + 1];
  // scn.cam.lookat[2] = data->xpos[trunkBodyId * 3 + 2];

  // get framebuffer viewport
  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

  // update scene and render
  mjv_updateScene(model, data, &opt, NULL, &cam, mjCAT_ALL, &scn);

  // Add visualisation.
  // task_->ModifyScene(model, data, &scn);

  // Add contact-related geoms to the visualization scene
  // addContactGeom(model, data, 0, nullptr, &scn);

  mjr_render(viewport, &scn, &con);

  // swap OpenGL buffers (blocking call due to v-sync)
  glfwSwapBuffers(window);

  // process pending GUI events, call GLFW callbacks
  glfwPollEvents();
}

void MujocoSimulator::put_robot_on_floor(int n_steps, VectorXd qref) {
  if (qref.size() != 12){
    throw std::runtime_error("qref should be size 12");
  }
  simstart = data->time;
  for (int k=0; k < n_steps; k++){
    for (int i = 0; i < model->nu; ++i) {
      double error = qref[i] - data->qpos[i + 7];
      double vel_error =
          data->qvel[i + 6];  // Assuming you have access to velocity information
      // double control_signal = data_i->qfrc_inverse[i+6] + kp_ * error -
      // kd_ * vel_error;
      double control_signal = settings.kp * error - settings.kd * vel_error;
      // double control_signal = qref[i];

      // Apply control signal to actuators or joints
      data->ctrl[i] = control_signal;
      // data->ctrl[i] = 0.;
    }
    mj_step(model, data);
    std::cout << "time [s] : " << data->time << std::endl;
    if (RENDERING_ && data->time - simstart < 1.0 / 60.0){
      update_viewer();
      simstart = data->time;
    }
  }
}

void MujocoSimulator::update_ref_curve(std::vector<double> points){
  // Update the reference curve inside the task planner.
  int indexes[2];
  ParameterIndexes(indexes, model, "residual_nn_updated");
  task_->parameters[indexes[0]] = 1.; // Boolean for update

  ParameterIndexes(indexes, model, "residual_nn");
  // Velocity point target (x3) + angle position target.
  task_->parameters[indexes[0]] = points[0];
  task_->parameters[indexes[0]+1] = points[1];
  task_->parameters[indexes[0]+2] = points[2];
  task_->parameters[indexes[0]+3] = points[3];
  task_->parameters[indexes[0]+4] = points[4];
  task_->parameters[indexes[0]+5] = points[5];

  // Update task
  task_->UpdateResidual();

  // Reset boolean to not update curve on next Update().
  ParameterIndexes(indexes, model, "residual_nn_updated");
  task_->parameters[indexes[0]] = -1.;
}

void MujocoSimulator::step(std::vector<double> actions) {
  if (actions.size() != 6) {
    throw std::runtime_error("Action size should be 6.");
  }
  std::cout << "n_step : " << n_iteration << std::endl;
  update_ref_curve(actions); // Extend reference curve with point.

  if (n_iteration == 0) {
    // Robot initilized with put_on_floor function.
    // Extend horizon with actions.
    n_iteration++;
    return;
  }
  int k_mpc = 0;
  for (int k_wbc = 0; k_wbc < 202; k_wbc++) {
    // Reset the contact status to 0.
    // mcontactData.update(model, data);
    if (LOGGING_){
      logger_.log(model, data, &mcontactData);
    }

    if (k_wbc % 18 == 0) {
      int indexes[2];
      std::string prefix = "residual_air_time_";
      for (const auto &name : foot_names_) {
        ParameterIndexes(indexes, model, prefix + name);
        task_->parameters[indexes[0]] = mcontactData.air_timings[name];
      }
      // Update time0.
      ParameterIndexes(indexes, model, prefix + "time0");
      task_->parameters[indexes[0]] = data->time;

      // Update task
      task_->UpdateResidual();

      state_.Set(model, data);
      planner.SetState(state_);
      task_->risk = 0.;

      // planner policy
      int n_max = 1;
      if (k_mpc % 5 == 0){
        n_max = 2;
      }
      for (int i = 0; i < n_max; i++) {
        // Setup model timestep.
        model->opt.timestep = settings.timestep_planner;
        planner.OptimizePolicyCustom(settings.n_steps,plan_pool);
        k_mpc ++;

      }
      // print_planner_timings();
      // Log best OCP trajectory.
      if (LOGGING_){
        logger_.logMPC(planner.BestTrajectory());
      }
      // std::cout << "time [s] : " << data->time << std::endl;
    }
    model->opt.timestep = settings.timestep;
    state_.Set(model, data);

    // Direct position control from policy.
    planner.ActionFromPolicy(data->ctrl, &state_.state()[0], state_.time(),
                             false);

    col.collision(model, data);

    // Simulation step.
    mj_step(model, data);

    if (RENDERING_ && data->time - simstart < 1.0 / 60.0) {
      update_viewer();
      simstart = data->time;
    }
  }

  observer.update(model, data);
  n_iteration++;
  return;
}

void MujocoSimulator::save_logger(const std::string &fileName) {
  logger_.saveData(fileName);
}

ObserverData MujocoSimulator::getObervation() {
  return observer.getObervation();
}