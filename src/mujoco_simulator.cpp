#include "mujoco_simulator.h"
#include "quadruped_task.h"
#include <iostream>

// Function to convert enum value to string
const char *enumToString(mjtObj value) {
  switch (value) {
  case mjOBJ_UNKNOWN:
    return "mjOBJ_UNKNOWN";
  case mjOBJ_BODY:
    return "mjOBJ_BODY";
  case mjOBJ_XBODY:
    return "mjOBJ_XBODY";
  case mjOBJ_JOINT:
    return "mjOBJ_JOINT";
  case mjOBJ_DOF:
    return "mjOBJ_DOF";
  case mjOBJ_GEOM:
    return "mjOBJ_GEOM";
  case mjOBJ_SITE:
    return "mjOBJ_SITE";
  case mjOBJ_CAMERA:
    return "mjOBJ_CAMERA";
  case mjOBJ_LIGHT:
    return "mjOBJ_LIGHT";
  case mjOBJ_FLEX:
    return "mjOBJ_FLEX";
  case mjOBJ_MESH:
    return "mjOBJ_MESH";
  case mjOBJ_SKIN:
    return "mjOBJ_SKIN";
  case mjOBJ_HFIELD:
    return "mjOBJ_HFIELD";
  case mjOBJ_TEXTURE:
    return "mjOBJ_TEXTURE";
  case mjOBJ_MATERIAL:
    return "mjOBJ_MATERIAL";
  case mjOBJ_PAIR:
    return "mjOBJ_PAIR";
  case mjOBJ_EXCLUDE:
    return "mjOBJ_EXCLUDE";
  case mjOBJ_EQUALITY:
    return "mjOBJ_EQUALITY";
  case mjOBJ_TENDON:
    return "mjOBJ_TENDON";
  case mjOBJ_ACTUATOR:
    return "mjOBJ_ACTUATOR";
  case mjOBJ_SENSOR:
    return "mjOBJ_SENSOR";
  case mjOBJ_NUMERIC:
    return "mjOBJ_NUMERIC";
  case mjOBJ_TEXT:
    return "mjOBJ_TEXT";
  case mjOBJ_TUPLE:
    return "mjOBJ_TUPLE";
  case mjOBJ_KEY:
    return "mjOBJ_KEY";
  case mjOBJ_PLUGIN:
    return "mjOBJ_PLUGIN";
  default:
    return "Unknown Enum Value";
  }
}

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

MujocoSimulator::MujocoSimulator(const char *modelFile)
    : model(nullptr), data(nullptr) {

  // Load Mujoco model
  char loadError[1024] = "";
  constexpr int kErrorLength = 1024;
  model = mj_loadXML(modelFile, nullptr, loadError, kErrorLength);
  if (!model) {
    std::cerr << "Error loading Mujoco model: " << loadError << std::endl;
  }

  // Access the simulation options
  mjOption *options = &model->opt;
  options->integrator = mjINT_EULER; // mjINT_RK4
  options->cone = mjCONE_ELLIPTIC;
  options->jacobian = mjJAC_AUTO;
  options->solver = mjSOL_NEWTON; // mjSOL_CG, mjSOL_PGS
  options->iterations = 100;
  options->tolerance = 1e-8;
  options->noslip_tolerance = 1e-6;
  options->noslip_iterations = 3;
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
  cam.azimuth = 90.0;    // Set azimuth angle
  cam.elevation = -20.0; // Set elevation angle
  cam.distance = 3.5;    // Set camera distance to 1.0

  // Params
  planner_threads_ = 5;
  horizon_ = 0.4;
  timestep_planner_ = 1.0e-2;
  timestep_ = 0.002;
  options->timestep = timestep_;
  kMaxTrajectoryHorizon_ = 128;
  steps_ = horizon_ / timestep_planner_ + 1; // planning steps

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
  planner.Reset(kMaxTrajectoryHorizon_);
  // planner.settings.verbose = 1;

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

  ///////////////////////
  // Model description
  ///////////////////////
  foot_names_ = {"FR", "FL", "HR", "HL"};
  for (const auto &name : foot_names_) {
    contact_status_[name] = 0;
    std::array<double, 3> tmp = {0., 0., 0.};
    contact_forces_[name] = tmp;
  }

  std::cout << "\nModel of the robot" << std::endl;
  for (int objType = mjOBJ_UNKNOWN; objType < mjOBJ_PLUGIN; ++objType) {
    mjtObj enumValue = static_cast<mjtObj>(objType);

    // Convert enum value to string
    const char *enumName = enumToString(enumValue);

    std::vector<std::pair<const char *, int>> objNames;
    for (int k = 0; k < 100; k++) {
      const char *objName = mj_id2name(model, objType, k);
      if (objName != nullptr) {
        objNames.push_back(std::make_pair(objName, k));
      }
    }
    if (objNames.size() > 0) {
      std::cout << "\n------- Types : " << enumName << "-------" << std::endl;
      for (const auto &element : objNames) {
        const char *objName = element.first;
        int index = element.second;
        std::cout << "Name : " << objName << " -- Index : " << index
                  << std::endl;

        auto it = std::find(foot_names_.begin(), foot_names_.end(),
                            std::string(objName));
        // Create a list of geometry.
        if (it != foot_names_.end() && objType == mjOBJ_GEOM) {
          foot_idx_.push_back(index);
        }
      }
    }
  }

  // Initialize logger.
  logger_.Initialize(foot_names_);

  // start plan thread
  runSimulation(1000);
}

MujocoSimulator::~MujocoSimulator() {
  if (model)
    mj_deleteModel(model);
  if (data)
    mj_deleteData(data);
}

// Function to disable interaction for specific geoms during Jacobian
// computation
void MujocoSimulator::disableInteractionForGeoms(mjModel *m) {
  // Iterate over the geoms you want to disable
  for (int geom_idx = 0; geom_idx < m->ngeom; ++geom_idx) {
    // Modify relevant geom properties (friction, solref, etc.)
    m->geom_friction[geom_idx] = 1e9;
    m->geom_solref[geom_idx * 3] = 1e9;
    // You may need to adjust other properties based on your specific
    // requirements
  }
}

// Function to enable interaction for specific geoms after Jacobian computation
void MujocoSimulator::enableInteractionForGeoms(mjModel *m) {
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

void MujocoSimulator::PlanIteration(mjpc::ThreadPool *pool) {
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
void MujocoSimulator::Plan(std::atomic<bool> &exitrequest,
                           std::atomic<int> &uiloadrequest) {
  // instantiate thread pool
  mjpc::ThreadPool pool_planner(planner_threads_);

  // main loop
  while (!exitrequest.load()) {
    if (model && uiloadrequest.load() == 0) {
      PlanIteration(&pool_planner);
    }
  } // exitrequest sent -- stop planning
}

void MujocoSimulator::initialize() {
  // Additional initialization steps if needed
}

void MujocoSimulator::runSimulation(int numSteps) {

  // Start planner in separate thread.
  std::atomic<bool> exitrequest(false);
  std::atomic<int> uiloadrequest(0);
  mjpc::ThreadPool plan_pool(planner_threads_);
  // plan_pool.Schedule([this,&exitrequest, &uiloadrequest]() {
  // Plan(exitrequest, uiloadrequest); });

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
      } else {
        data->mocap_pos[0] = 1.2;
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
        // task_->Residual(model, data, data->sensordata);task_->Residual(model,
        // data, data->sensordata);

        // Contact detection code. TODO: Write a proper function/ class to
        // handle this.

        // Reset the contact status to 0.
        for (auto &status : contact_status_) {
          status.second = 0;
        }
        for (int contactIndex = 0; contactIndex < data->ncon; ++contactIndex) {
          int geomIndex0 = data->contact[contactIndex].geom[0];
          auto it0 = std::find(foot_idx_.begin(), foot_idx_.end(), geomIndex0);
          int geomIndex1 = data->contact[contactIndex].geom[1];
          auto it1 = std::find(foot_idx_.begin(), foot_idx_.end(), geomIndex1);
          const char *geomName0 = mj_id2name(model, mjOBJ_GEOM, geomIndex0);
          const char *geomName1 = mj_id2name(model, mjOBJ_GEOM, geomIndex1);
          if (it0 != foot_idx_.end() || it1 != foot_idx_.end()) {
            mjtNum mat[9], confrc[6], frc[3], vec[3];

            // mat = contact frame rotation matrix (normal along x)
            mju_transpose(mat, data->contact[contactIndex].frame, 3, 3);

            // get contact force:torque in contact frame
            mj_contactForce(model, data, contactIndex, confrc);

            // Get nonly the linear forces.
            mju_copy(frc, confrc, 3);

            mju_mulMatVec(vec, mat, frc, 3, 3);

            // Calculate the index by subtracting iterators
            if (it0 != foot_idx_.end()) {
              contact_status_[geomName0] = 1;
              // Point from Geom[0] to Geom 1. Here Geom[0] is the foot.
              mju_scl3(vec, vec, -1);
              // std::array<double, 3> tmp_vec = {vec[1], vec[2], vec[0]}
              contact_forces_[geomName0][0] = vec[0];
              contact_forces_[geomName0][1] = vec[1];
              contact_forces_[geomName0][2] = vec[2];
            } else {
              contact_status_[geomName1] = 1;
              // Point from Geom[0] to Geom 1. Here Geom[1] is the foot.
              // Direction Ok. mju_scl3(vec, vec, -1);
              contact_forces_[geomName0][0] = vec[0];
              contact_forces_[geomName0][1] = vec[1];
              contact_forces_[geomName0][2] = vec[2];
            }
          }
        }

        // Print un-ordered map.
        // std::cout << "Contact status [";
        // for (auto& ct:contact_status_){
        //   std::cout << ct.first << ",";
        // }
        // std::cout << "] : [";
        // for (auto& ct:contact_status_){
        //   std::cout << ct.second << ",";
        // }
        // std::cout << "]" << std::endl;

        logger_.logState(model, data);

        logger_.logFeetStatus(contact_status_);
        logger_.logFeetPosition(model, data);
        logger_.logFeetVelocity(model, data);
        logger_.logFeetTouch(model, data);
        logger_.logFeetForces(contact_forces_);

        if (data->time > 2.) {
          logger_.saveData(
              "/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/log/tmp.bin");
          Data data = logger_.loadData(
              "/home/thomas_cbrs/Desktop/edin_23/mjpc_rl/log/tmp.bin");
          // logger_.writeToCsvFile(filename);
          return;
        }

        if (counter_wbc % 10 == 0) {
          // PlanIteration(&plan_pool);
          // set state
          // task_->SetFeatureParameters(model);
          // task_->parameters[0] = -0.9;
          // data->sensordata[1] = data->qpos[0] - 0.9;
          // task_->Reset();

          task_->parameters[0] = 1.;
          task_->UpdateResidual();

          state_.Set(model, data);
          planner.SetState(state_);
          residual_fn_ = task_->Residual();
          task_->risk = 0.;
          // planner policy
          for (int i = 0; i <= 1; i++) {
            // Setup model timestep.
            model->opt.timestep = timestep_planner_;
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
        planner.ActionFromPolicy(data->ctrl, &state_.state()[0], state_.time(),
                                 false);

        // std::vector<double>
        // for (int i = 0; i < model->nu; ++i) {
        //   double error = planner.BestTrajectory()->states[37*1 + 7 + i] -
        //   data->qpos[i + 7]; double vel_error = data->qvel[i + 6]; //
        //   Assuming you have access to velocity information
        //   // double control_signal = data_i->qfrc_inverse[i+6] + kp_ * error
        //   -
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
      model->opt.timestep = timestep_;

      // Monitoring real values of the sensors from data and datasensors.
      // double* FR = mjpc::SensorByName(model, data, "FR_vel");
      // int siteId = mj_name2id(model, mjOBJ_SITE, "FR_vel");
      // const double* sitePos = data->site_xpos + 3 * siteId;
      // std::cout << "FR (sensor) : " << FR[2] << std::endl;
      // std::cout << "FR (data)   : " << sitePos[2] << std::endl;

      mj_step(model, data);
    }

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
    task_->ModifyScene(model, data, &scn);

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