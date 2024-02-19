#include "mujoco_simulator.h"
#include "quadruped_task.h"
#include "utils.cpp"
#include <iostream>
#include <thread>

// Define the task outisde the class function.
// mjpc::QuadrupedFlat *task_;
// mjpc::QuadrupedHill *task_;
// mjpc::Cartpole *task_;
QuadrupedTask *task_;

// State and planner.
mjpc::State state_;
mjpc::iLQGPlanner planner;
// mjpc::SamplingPlanner planner;
// mjpc::GradientPlanner planner;

// Logger path.
std::string filename = "/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/log/tmp.csv";

MujocoSimulator::MujocoSimulator(int n_threads, bool rendering, const char *modelFile)
    : model(nullptr), data(nullptr), foot_names_{"FR", "FL", "HR", "HL"},
      mcontactData(foot_names_, 0.002) {

  // Load Mujoco model
  char loadError[1024] = "";
  constexpr int kErrorLength = 1024;
  model = mj_loadXML(modelFile, nullptr, loadError, kErrorLength);
  if (!model) {
    std::cerr << "Error loading Mujoco model: " << loadError << std::endl;
  }

  // Rendering flag
  RENDERING_ = rendering;

  // Access the simulation options
  mjOption *options = &model->opt;
  options->integrator = mjINT_EULER; // mjINT_RK4
  options->cone = mjCONE_ELLIPTIC; // mjCONE_PYRAMIDAL
  // options->cone = mjCONE_PYRAMIDAL;
  options->jacobian = mjJAC_AUTO;
  options->solver = mjSOL_NEWTON; // mjSOL_CG, mjSOL_PGS
  options->iterations = 100; // 100
  options->tolerance = 1e-8; // 1e-8
  options->noslip_tolerance = 1e-6;
  options->noslip_iterations = 3; // 3
  options->mpr_tolerance = 1e-6;

  // Initialize Mujoco simulation
  data = mj_makeData(model);

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
  planner_threads_ = n_threads;
  horizon_ = 0.4;
  timestep_planner_ = 1.0e-2;
  timestep_ = 0.002;
  mcontactData.dt_simu = timestep_; // TODO, find a better way.
  options->timestep = timestep_;
  kMaxTrajectoryHorizon_ = 128;
  steps_ = horizon_ / timestep_planner_ + 1; // planning steps

  // Define tasks.
  // task_ = new mjpc::QuadrupedFlat();
  // task_ = new mjpc::QuadrupedHill();
  // task_ = new mjpc::Cartpole();
  task_ = new QuadrupedTask();
  task_->Reset(model);
  task_->SetParameters(model);

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
  planner.Reset(kMaxTrajectoryHorizon_);
  task_->num_trace = 0;
  // planner.settings.verbose = 1;
  // planner.settings.fd_tolerance = 1.0e-8;

  // cost
  terms_.resize(task_->num_term * kMaxTrajectoryHorizon_);
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

  ///////////////////////
  // Model description
  ///////////////////////
  foot_names_ = {"FR", "FL", "HR", "HL"};

  // Print model informations.
  infos_models(model);

  // Initialize logger.
  int k_mpc = 10;
  logger_.Initialize(foot_names_, timestep_planner_, steps_, k_mpc, timestep_);

  // Simulate NN decision for reference velocity curve.
  Vector6d point;
  point << 0.5, 0.0, 0.0, 0.0, 0.0, 0.0;
  list_points.push_back(point);
  point << 0.7,0.,0. ,0.,-0.,0.;
  list_points.push_back(point);
  point << 1.1, 0.0, 0.0, 0.0, -0., 0.1;
  list_points.push_back(point);
  point << 1.3, 0.0, 0.0, 0.0, -0., 0.2;
  list_points.push_back(point);
  point << 1.5, 0.0, 0., 0.0, -0., 0.3;
  list_points.push_back(point);
  point << 1.7, 0.0, 0.0, 0.0, -0., 0.5;
  list_points.push_back(point);
  point << 1.9, 0.0, 0.0, 0.0, -0., 0.7;
  list_points.push_back(point);
  point << 2.1, 0.0, 0.0, 0.0, -0., 0.0;
  list_points.push_back(point);
  point << 2.3, 0.0, 0.0, 0.0, -0., 0.0;
  list_points.push_back(point);
  point << 2.5, 0.0, 0.0, 0.0, -0., 0.0;
  list_points.push_back(point);
  point << 2.7, 0.0, 0.0, 0.0, -0., 0.0;
  list_points.push_back(point);
  idx_nn_ = 0;

  initialize_viewer();

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
  planner.Reset(kMaxTrajectoryHorizon_);
  task_->UpdateResidual();

  mj_step(model, data);
  update_viewer();

  put_robot_on_floor(200,q0_.tail(12));

  std::this_thread::sleep_for(std::chrono::seconds(2));
}

MujocoSimulator::~MujocoSimulator() {
  if (model)
    mj_deleteModel(model);
  if (data)
    mj_deleteData(data);
}

void MujocoSimulator::print_planner_timings() {
  std::cout << "\nTotal time [ms] : " << 1e-3*planner.nominal_compute_time << std::endl;
  std::cout << "Model derivative [ms] : " << 1e-3*planner.model_derivative_compute_time << std::endl;
  std::cout << "Cost derivative [ms] : " << 1e-3*planner.cost_derivative_compute_time << std::endl;
  std::cout << "Rollout [ms] : " << 1e-3*planner.rollouts_compute_time << std::endl;
  std::cout << "Backward pass [ms] : " << 1e-3*planner.backward_pass_compute_time << std::endl;
  std::cout << "Policy update [ms] : " << 1e-3*planner.policy_update_compute_time << std::endl;
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
  mjtNum simstart;
  simstart = data->time;
  for (int k=0; k < n_steps; k++){
    for (int i = 0; i < model->nu; ++i) {
      double error = qref[i] - data->qpos[i + 7];
      double vel_error =
          data->qvel[i + 6];  // Assuming you have access to velocity information
      // double control_signal = data_i->qfrc_inverse[i+6] + kp_ * error -
      // kd_ * vel_error;
      double control_signal = kp_ * error - kd_ * vel_error;
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

void MujocoSimulator::update_ref_curve(int idx){
  // Update the reference curve inside the task planner.
  int indexes[2];
  ParameterIndexes(indexes, model, "residual_nn_updated");
  task_->parameters[indexes[0]] = 1.; // Boolean for update

  ParameterIndexes(indexes, model, "residual_nn");
  // Velocity point target (x3) + angle position target.
  task_->parameters[indexes[0]] = list_points[idx][0];
  task_->parameters[indexes[0]+1] = list_points[idx][1];
  task_->parameters[indexes[0]+2] = list_points[idx][2];
  task_->parameters[indexes[0]+3] = list_points[idx][3];
  task_->parameters[indexes[0]+4] = list_points[idx][4];
  task_->parameters[indexes[0]+5] = list_points[idx][5];
}

void MujocoSimulator::runSimulation(int numSteps) {

  // Start planner in separate thread.
  // std::atomic<bool> exitrequest(false);
  // std::atomic<int> uiloadrequest(0);
  mjpc::ThreadPool plan_pool(planner_threads_);
  // plan_pool.Schedule([this,&exitrequest, &uiloadrequest]() {
  // Plan(exitrequest, uiloadrequest); });

  // Set control callback
  mjcb_control = mycontroller;

  // Set sensor callback
  mjcb_sensor = &MujocoSimulator::sensor;

  int counter_wbc = 0;

  // mj_step(model, data);
  put_robot_on_floor(500, q0_.tail(12)); // Initialisation

  mjtNum simstart = data->time;

  for (int k_wbc = 0; k_wbc < numSteps; k_wbc++) {
    // Check if windows is open on rendering.
    if (RENDERING_) {
      if (glfwWindowShouldClose(window)) {
        // close GLFW, free visualization storage
        glfwTerminate();
        mjv_freeScene(&scn);
        mjr_freeContext(&con);
        break;
      }
    }
    // Reset the contact status to 0.
    mcontactData.update(model, data);
    logger_.log(model, data, &mcontactData);

    // Update reference curve.
    if (k_wbc % 200 == 0){
      update_ref_curve(idx_nn_);
      idx_nn_ ++;
    }
    else{
      // TODO call this function at each loop. Once to reset to -1 is enough.
      int indexes[2];
      ParameterIndexes(indexes, model, "residual_nn_updated");
      task_->parameters[indexes[0]] = -1.;
    }

    // if (data->time > 5.2) {
    //       Eigen::VectorXd q_pos(19);
    //       q_pos << 1., 0.5, 0.3, 1., 0., 0., 0., 0., 0., 0.,0., 0.,  0.,  0., 0., 0., 0., 0., 0.;
    //       reset(q_pos);
    //       // mj_resetData(model,data);
    //       // logger_.saveData(
    //       //     "/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/log/tmp.bin");
    //       // Data data = logger_.loadData(
    //       //     "/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/log/tmp.bin");
    //       // // logger_.writeToCsvFile(filename);
    //       // std::this_thread::sleep_for(std::chrono::seconds(2));
    //       return;
    // }

    // Planner iteration
    if (k_wbc % 10 == 0) {
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
      // residual_fn_ = task_->Residual();
      // planner policy
      for (int i = 0; i <= 1; i++) {
        // Setup model timestep.
        model->opt.timestep = timestep_planner_;
        planner.OptimizePolicy(steps_, plan_pool);
      }
      // print_planner_timings();
      // Log best OCP trajectory.
      logger_.logMPC(planner.BestTrajectory());
    }
    std::cout << "time [s] : " << data->time << std::endl;
    model->opt.timestep = timestep_;
    state_.Set(model, data);

    // Direct position control from policy.
    planner.ActionFromPolicy(data->ctrl, &state_.state()[0], state_.time(),
                              false);

    // Simulation step.
    mj_step(model, data);

    if (RENDERING_ && data->time - simstart < 1.0 / 60.0){
      update_viewer();
      simstart = data->time;
    }
  }
  // close GLFW, free visualization storage
  // glfwTerminate();
  // mjv_freeScene(&scn);
  // mjr_freeContext(&con);
}

std::vector<std::vector<double>>
MujocoSimulator::getLoggedJointPositions() const {
  return jointPositionsLog;
}