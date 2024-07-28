#include "mujoco_simulator.h"
#include "utils.h"
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

MujocoSimulator::MujocoSimulator(int n_threads, bool rendering, bool logging,
                                 const char *modelFile)
    : model(nullptr), data(nullptr), foot_names_{"FR", "FL", "HR", "HL"},
      observer(foot_names_, settings.horizon_nn, settings.horizon_reset),
      mcontactData(foot_names_, 0.002),
      plan_pool(n_threads) {
  if (n_threads == 1 && !flag_thread_local) {
    throw std::runtime_error("1 thread selected. Flag thread only should be "
                             "activated during conpilation.");
  }
  if (n_threads != 1 && flag_thread_local) {
    throw std::runtime_error("Multi-threading selected. Flag thread only "
                             "should be de-activated during conpilation.");
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
  q0_ << 0., -0., 0.3, // position
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
  }
  if (LOGGING_) {
    logger_.Initialize(foot_names_, settings.timestep_planner, settings.n_steps,
                       settings.k_mpc, settings.timestep);
  }

  // Set engine callbacks
  mjcb_control = mycontroller;
  mjcb_sensor = &MujocoSimulator::sensor;

  // Heightmap
  heightmap_ = Heightmap();
  // heightmap_.create_environment1();
  // heightmap_.create_environment_baseline();
  heightmap_.create_environment_baseline_holes();

  // Setup task horizons.
  set_horizon_nn(settings.horizon_nn);
  set_horizon_reset(settings.horizon_reset);

  // Initialisation
  planner.UpdateNumTrajectoriesFromGUI();
  put_robot_on_floor(int(settings.horizon_reset / settings.timestep), q0_.tail(12));
  std::vector<double> q(6, 0.0);
  observer.reset(q);
  observer.update_final_pose(model, data);
  observer.update_filter(model, data);
  col.collision(model, data);
}

void MujocoSimulator::reset0(std::vector<double> q, int envId) {
    // Function definition with default argument
    reset(q, envId, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
}

void MujocoSimulator::reset(std::vector<double> q, int envId,const std::vector<double>& action_init /*={0.,0.,0.,0.,0.,0.}*/) {
  if (q.size() != 6) {
    throw std::runtime_error("q0 should be size 6, [x,y,z,r,p,y]");
  }
  if (action_init.size() != 6) {
    throw std::runtime_error("action_init should be size 6, [vx,vy,vz,r,p,y]");
  }
  mj_resetData(model, data); // reset Data
  data->qpos[0] = q.at(0);   // x
  data->qpos[1] = q.at(1);   // y
  data->qpos[2] = q.at(2);   // z
  // Get quaternion.
  Eigen::Quaterniond quat(
      pinocchio::rpy::rpyToMatrix(q.at(3), q.at(4), q.at(5)));
  data->qpos[3] = quat.w();
  data->qpos[4] = quat.x();
  data->qpos[5] = quat.y();
  data->qpos[6] = quat.z();
  for (int i = 7; i < model->nq; ++i) {
    data->qpos[i] = q0_[i];
  }
  // Keep track of actions and q0
  h_actions.clear();
  h_actions.push_back(action_init);
  h_q0 = q;
  // Set environement for heightmap
  heightmap_.setCurrentEnvironment(envId);
  heightmap_.update_heightmap(model, data);

  // Reset task
  task_->Reset(model);
  task_->SetParameters(model);

  // Reset state
  state_.Reset();
  state_.Set(model, data);

  // Reset planner
  planner.Reset(settings.n_steps);
  task_->UpdateResidual();

  // mj_step(model, data); // Increment data->time.
  mj_forward(model, data);
  if (RENDERING_) {
    update_viewer();
  }

  // Fix time 198. 200 not working, do a round approx.
  put_robot_on_floor(int(settings.horizon_reset / settings.timestep), q0_.tail(12));

  reset_task(q); // Warning q is size 6 and rpy are the last 3 elements.
  simstart = data->time;

  // Reset iteration
  n_iteration = 0;
  k_mpc_ = 0;
  k_wbc_ = 0;
  first_step(action_init);

  col.resetCollisionStatus();
  observer.reset(q);
  observer.update_final_pose(model, data);
  observer.update_filter(model, data);
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

// void MujocoSimulator::print_planner_timings() {
//   std::cout << "\nTotal time [ms] : " << 1e-3 * planner.nominal_compute_time
//             << std::endl;
//   std::cout << "Model derivative [ms] : "
//             << 1e-3 * planner.model_derivative_compute_time << std::endl;
//   std::cout << "Cost derivative [ms] : "
//             << 1e-3 * planner.cost_derivative_compute_time << std::endl;
//   std::cout << "Rollout [ms] : " << 1e-3 * planner.rollouts_compute_time
//             << std::endl;
//   std::cout << "Backward pass [ms] : "
//             << 1e-3 * planner.backward_pass_compute_time << std::endl;
//   std::cout << "Policy update [ms] : "
//             << 1e-3 * planner.policy_update_compute_time << std::endl;
// }

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
  mjvPerturb pert; // Declare mjvPerturb variable
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
  mjv_defaultScene(&scn);
  mjv_makeScene(model, &scn, 1000);
  mjr_makeContext(model, &con, mjFONTSCALE_100);

  // Adjust camera distance
  cam.azimuth = 70.0;    // Set azimuth angle
  cam.elevation = -20.0; // Set elevation angle
  cam.distance = 10.;    // Set camera distance to 1.0
}

void MujocoSimulator::update_viewer() {
  if (!is_viewer_init) {
    initialize_viewer();
    is_viewer_init = true;
  }
  // Get trunk posiiton and update the camera position.
  int trunkBodyId = mj_name2id(model, mjOBJ_BODY, "trunk");
  // Adjust the camera position based on the trunk body position
  cam.lookat[0] = data->xpos[trunkBodyId * 3];
  cam.lookat[1] = data->xpos[trunkBodyId * 3 + 1];
  cam.lookat[2] = data->xpos[trunkBodyId * 3 + 2];
  // scn.cam.lookat[2] = data->xpos[trunkBodyId * 3 + 2];

  // get framebuffer viewport
  mjrRect viewport = {0, 0, 0, 0};
  glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

  // update scene and render
  mjv_updateScene(model, data, &opt, NULL, &cam, mjCAT_ALL, &scn);

  // Goal visualisation
  mjvGeom *geomtest = scn.geoms + scn.ngeom++;
  mjv_initGeom(geomtest, mjGEOM_SPHERE, vz_size, vz_pos, NULL, vz_color);
  scn.geoms[scn.ngeom].category = mjCAT_DECOR;

  // Add heightmap.
  if (heightmap_.getCurrentEnvironment() != -1){
    heightmap_.update_heightmap(model, data);
  }
  double vh_size[3] = {0.02};
  float vh_color[4] = {0., 0.9, 0.1, 0.6};
  for (int i=0; i < heightmap_.Nx_; i++) {
    for (int j=0; j < heightmap_.Ny_; j++) {
      double pos_tmp[3] = {0.,0.,0.};
      pos_tmp[0] = heightmap_.xx_w(i,j);
      pos_tmp[1] = heightmap_.yy_w(i,j);
      // if (i == 3 && j == 4){
      //     std::cout << "--height--\n" << std::endl;
      //     std::cout << "pos_w : [" << pos_tmp[0] << "," << pos_tmp[1] << "]" << std::endl;
      //     std::cout << "--" << std::endl;
      //   }
      if (heightmap_.z_(i, j) == 0.){
        pos_tmp[2] = 0.;
        vh_color[0] = 0.9;
        vh_color[1] = 0.0;
        vh_color[2] = 0.0;
        vh_color[3] = 0.7;
      }
      else{
        pos_tmp[2] = 0.;
        vh_color[0] = 1.;
        vh_color[1] = 1.;
        vh_color[2] = 1.;
        vh_color[3] = 0.90;
      }
      mjvGeom *geomtmp = scn.geoms + scn.ngeom++;
      mjv_initGeom(geomtmp, mjGEOM_SPHERE, vh_size, pos_tmp, NULL, vh_color);
      scn.geoms[scn.ngeom].category = mjCAT_DECOR;
    }
  }

  // Add visualisation.
  if (data->time > settings.horizon_reset + 0.002) {
    task_->ModifyScene(model, data, &scn);
    // print_traj();
    // print_tree();
  }

  // Add contact-related geoms to the visualization scene
  // addContactGeom(model, data, 0, nullptr, &scn);

  mjr_render(viewport, &scn, &con);

  // swap OpenGL buffers (blocking call due to v-sync)
  glfwSwapBuffers(window);

  // process pending GUI events, call GLFW callbacks
  glfwPollEvents();
}

void MujocoSimulator::print_traj(){
    double size[3] = {0.01};
  double pos[3];
  pos[0] = data->qpos[0];
  pos[1] = data->qpos[1];
  pos[2] = data->qpos[2] + 0.05;
  double pos_previous[3] = {pos[0], pos[1], pos[2]};

  // color
  float color[4];
  color[0] = 1.;
  color[1] = 1.0;
  color[2] = 1.;
  color[3] = 0.7;

  // At time data-time, get rotationmatrix for world frame velocity reference.
  const mjtNum axis[3] = {0., 0., 1.}; // z-axis (yaw)
  Matrix3d R_tmp = Matrix3d::Zero();
  mjtNum R_data[9];
  mjtNum quat_tmp[4];
  // Get quaternion projected on z-axis (only yaw component).
  mju_mulQuatAxis(quat_tmp, &data->qpos[3],
                  axis);          // Convert axis-angle to quaternion
  mju_quat2Mat(R_data, quat_tmp); // Convert quaternion to rotation matrix
  updateMatrix(R_tmp, R_data);

  Vector3d vel_world = Vector3d::Zero();
  Vector3d vel_tmp = Vector3d::Zero();
  Vector3d dx = Vector3d::Zero();
  Matrix3d dR = Matrix3d::Zero();

  double t_min = data->time;
  double t_max = observer.pcVel_.max() - t_min;
  double dt = 0.02;
  int n_points = int(t_max / dt);
  // int n_points = 6;
  std::cout << "data->time : " << data->time << std::endl;
  std::cout << "observer.pcVel_.max() : " << observer.pcVel_.max() << std::endl;

  for (int i = 0; i < n_points -1; i++) {
    if (i > 0) {
      pos_previous[0] = pos[0];
      pos_previous[1] = pos[1];
      pos_previous[2] = pos[2];
    }
    double t = t_min + t_max * (float(i) / float(n_points));

    vel_tmp = observer.pcVel_(t);

    // Vector3d rot = observer.pcRot_.derivate(t,1);
    Vector3d rot = observer.pcRot_(t);
    // R_tmp = pinocchio::rpy::rpyToMatrix(0., 0., rot(2));
    vel_world = vel_tmp;

    dR = pinocchio::rpy::rpyToMatrix(rot(0), rot(1), rot(2));
    Vector3d dx = dR * dt * vel_world;
    pos[0] = pos[0] + dx(0);
    pos[1] = pos[1] + dx(1);
    pos[2] = pos[2] + dx(2);

    mjvGeom *geomtest = scn.geoms + scn.ngeom++;
    mjv_initGeom(geomtest, mjGEOM_SPHERE, size, pos, NULL, color);
    scn.geoms[scn.ngeom].category = mjCAT_DECOR;

    if (i > 0) {
      // mjvGeom* geomtest2 = scn.geoms + scn.ngeom++;
      // make connector geom
      mjvGeom *geomtest2 = scn.geoms + scn.ngeom++;
      mjv_initGeom(geomtest2, mjGEOM_LINE,
                   /*size=*/nullptr, /*pos=*/nullptr, /*mat=*/nullptr, color);
      scn.geoms[scn.ngeom].category = mjCAT_DECOR;
      double *from = pos_previous;
      double *to = pos;
      mjv_makeConnector(geomtest2, mjGEOM_LINE, 2, from[0], from[1], from[2],
                        to[0], to[1], to[2]);
    }
  }
}

void MujocoSimulator::print_tree() {
  float color[4] = {1.,1.,1.,0.9};
  double size[3] = {0.01};
  double pos[3];
  // Iterate over each trajectory in the vector
  for (const auto &traj : trajectories_) {
    // Iterate over each trajectory point in the trajectory
    for (const auto &point : traj) {
      // Print the trajectory point
      if (point.is_connector) {
        // Handle connector points differently
        // Print connector point...
        mjvGeom *geomtest2 = scn.geoms + scn.ngeom++;
        mjv_initGeom(geomtest2, mjGEOM_LINE,
                    /*size=*/nullptr, /*pos=*/nullptr, /*mat=*/nullptr, color);
        scn.geoms[scn.ngeom].category = mjCAT_DECOR;
        const double *from = point.from;
        const double *to = point.pos;
        mjv_makeConnector(geomtest2, mjGEOM_LINE, 2, from[0], from[1], from[2],
                        to[0], to[1], to[2]);
      } else {
        pos[0] = point.pos[0];
        pos[1] = point.pos[1];
        pos[2] = point.pos[2];
        mjvGeom *geomtest = scn.geoms + scn.ngeom++;
        mjv_initGeom(geomtest, mjGEOM_SPHERE, size, pos, NULL, color);
        scn.geoms[scn.ngeom].category = mjCAT_DECOR;
      }
    }
  }
}

void MujocoSimulator::store_trajectory(){
  double size[3] = {0.01};
  double pos[3];
  pos[0] = data->qpos[0];
  pos[1] = data->qpos[1];
  pos[2] = data->qpos[2] + 0.05;
  double pos_previous[3] = {pos[0], pos[1], pos[2]};

  // At time data-time, get rotationmatrix for world frame velocity reference.
  const mjtNum axis[3] = {0., 0., 1.}; // z-axis (yaw)
  Matrix3d R_tmp = Matrix3d::Zero();
  mjtNum R_data[9];
  mjtNum quat_tmp[4];
  // Get quaternion projected on z-axis (only yaw component).
  mju_mulQuatAxis(quat_tmp, &data->qpos[3],
                  axis);          // Convert axis-angle to quaternion
  mju_quat2Mat(R_data, quat_tmp); // Convert quaternion to rotation matrix
  updateMatrix(R_tmp, R_data);

  Vector3d vel_world = Vector3d::Zero();
  Vector3d vel_tmp = Vector3d::Zero();
  Vector3d dx = Vector3d::Zero();
  Matrix3d dR = Matrix3d::Zero();

  double t_min = data->time;
  double t_max = observer.pcVel_.max() - t_min;
  double dt = 0.02;
  int n_points = int(t_max / dt);
  int ratio_prints = 4;
  std::vector<TrajectoryPoint> traj_points;

  for (int i = 0; i < n_points -1; i++) {
    if (i > 0) {
      pos_previous[0] = pos[0];
      pos_previous[1] = pos[1];
      pos_previous[2] = pos[2];
    }
    double t = t_min + t_max * (-0.001 + float(i+1) / float(n_points));
    vel_tmp = observer.pcVel_(t);

    // Vector3d rot = observer.pcRot_.derivate(t,1);
    Vector3d rot = observer.pcRot_(t);
    // R_tmp = pinocchio::rpy::rpyToMatrix(0., 0., rot(2));
    vel_world = vel_tmp;

    dR = pinocchio::rpy::rpyToMatrix(rot(0), rot(1), rot(2));
    Vector3d dx = dR * dt * vel_world;
    pos[0] = pos[0] + dx(0);
    pos[1] = pos[1] + dx(1);
    pos[2] = pos[2] + dx(2);

    // Store trajectory point in traj_points vector
    TrajectoryPoint point;
    // Set point properties (pos, color, etc.)
    point.pos[0] = pos[0];
    point.pos[1] = pos[1];
    point.pos[2] = pos[2];
    point.from[0] = pos[0];
    point.from[1] = pos[1];
    point.from[2] = pos[2];

    if (i % ratio_prints == 0 || i == n_points - 2){
      if (i > 0) {
        // std::cout << traj_points.back().from[0] << std::endl;
        point.from[0] = traj_points.back().pos[0];
        point.from[1] = traj_points.back().pos[1];
        point.from[2] = traj_points.back().pos[2];
        point.is_connector = true;
      }
      traj_points.push_back(point);
    }
  }
  trajectories_.push_back(traj_points);
}

void MujocoSimulator::put_robot_on_floor(int n_steps, VectorXd qref) {
  if (qref.size() != 12) {
    throw std::runtime_error("qref should be size 12");
  }
  simstart = data->time;
  for (int k = 0; k < n_steps; k++) {
    for (int i = 0; i < model->nu; ++i) {
      double error = qref[i] - data->qpos[i + 7];
      double vel_error =
          data->qvel[i + 6]; // Assuming you have access to velocity information
      // double control_signal = data_i->qfrc_inverse[i+6] + kp_ * error -
      // kd_ * vel_error;
      double control_signal = settings.kp * error - settings.kd * vel_error;
      // double control_signal = qref[i];

      // Apply control signal to actuators or joints
      data->ctrl[i] = control_signal;
      // data->ctrl[i] = 0.;
    }
    mj_step(model, data);
    if (RENDERING_ && (data->time - simstart > 1.0 / 120.)) {
      update_viewer();
      simstart = data->time;
    }
  }
}

void MujocoSimulator::update_ref_curve(std::vector<double> points) {
  // Update the reference curve inside the task planner.
  int indexes[2];
  ParameterIndexes(indexes, model, "residual_nn_updated");
  task_->parameters[indexes[0]] = 1.; // Boolean for update

  ParameterIndexes(indexes, model, "residual_nn");
  // Velocity point target (x3) + angle position target.
  task_->parameters[indexes[0]] = points[0];
  task_->parameters[indexes[0] + 1] = points[1];
  task_->parameters[indexes[0] + 2] = points[2];
  task_->parameters[indexes[0] + 3] = points[3];
  task_->parameters[indexes[0] + 4] = points[4];
  task_->parameters[indexes[0] + 5] = points[5];

  // Update task
  task_->UpdateResidual();

  // Reset boolean to not update curve on next Update().
  ParameterIndexes(indexes, model, "residual_nn_updated");
  task_->parameters[indexes[0]] = -1.;

  // Update observer references curves.
  observer.update_ref_curve(points);
}

void MujocoSimulator::reset_task(std::vector<double> q) {
  if (q.size() != 6) {
    throw std::runtime_error("q0 should be size 6, [x,y,z,r,p,y]");
  }
  // Reset the reference curve inside task planner.
  int indexes[2];
  ParameterIndexes(indexes, model, "residual_nn_updated");
  task_->parameters[indexes[0]] = 0.; // Boolean for reset

  ParameterIndexes(indexes, model, "residual_nn_reset");
  // Angle position on reset.
  task_->parameters[indexes[0]] = q[3];
  task_->parameters[indexes[0] + 1] = q[4];
  task_->parameters[indexes[0] + 2] = q[5];

  // Update environement
  ParameterIndexes(indexes, model, "residual_nn_envId");
  task_->parameters[indexes[0]] = heightmap_.getCurrentEnvironment();

  // Update task
  task_->UpdateResidual();

  // Reset boolean to not update curve on next Update().
  ParameterIndexes(indexes, model, "residual_nn_updated");
  task_->parameters[indexes[0]] = -1.;
}

void MujocoSimulator::first_step(std::vector<double> actions){
  observer.update_filter(model, data);
  observer.update_collision_status(col.getCollisionStatus());
  observer.update_contact_status(mcontactData);
  observer.update_final_pose(model, data);

  // Trick Here. TODO: Use a horizon variable to update_ref_curve.
  set_horizon_nn(settings.horizon_planner);
  // Update task
  task_->UpdateResidual();
  update_ref_curve(actions); // Extend reference curve with point.
  set_horizon_nn(settings.horizon_nn);
  task_->UpdateResidual();
}

void MujocoSimulator::step(std::vector<double> actions) {
  if (actions.size() != 6) {
    throw std::runtime_error("Action size should be 6.");
  }
  update_ref_curve(actions); // Extend reference curve with point.
  h_actions.push_back(actions);
  // store_trajectory();

  // if (n_iteration == 0) {
  //   // Robot initilized with put_on_floor function.
  //   // Extend horizon with actions.
  //   observer.update_filter(model, data);
  //   observer.update_collision_status(col.getCollisionStatus());
  //   observer.update_contact_status(mcontactData);
  //   observer.update_final_pose(model, data);
  //   n_iteration++;
  //   return;
  // }

  for (int kk = 0; kk < settings.horizon_nn / settings.timestep;
       kk++) {
    // Reset the contact status to 0.
    // mcontactData.update(model, data);
    if (LOGGING_) {
      logger_.log(model, data, &mcontactData);
    }

    if (k_wbc_ % mpc_ratio_max_wbc_ == 0) {
      // std::cout << "data->time MPC : " << data->time << std::endl;
      int indexes[2];
      std::string prefix = "residual_air_time_";
      for (const auto &name : foot_names_) {
        ParameterIndexes(indexes, model, prefix + name);
        task_->parameters[indexes[0]] = mcontactData.air_timings[name];
      }
      // Update time0.
      ParameterIndexes(indexes, model, prefix + "time0");
      task_->parameters[indexes[0]] = data->time;

      // Get yaw orientation to define local frame for Vref_x and Vref_y
      // Use filtered yaw from observer.
      double yaw = observer.get_yaw_filtered();
      ParameterIndexes(indexes, model, "residual_yaw_local");
      task_->parameters[indexes[0]] = yaw;

      // Update task
      task_->UpdateResidual();

      state_.Set(model, data);
      planner.SetState(state_);
      task_->risk = 0.;

      // planner policy
      int n_max = mpc_min_iteration_; // 1
      if (k_mpc_ % mpc_ratio_max_iteration_ == 0) {
        n_max = mpc_max_iteration_;
      }
      for (int i = 0; i < n_max; i++) {
        // Setup model timestep.
        model->opt.timestep = settings.timestep_planner;
        planner.OptimizePolicyCustom(settings.n_steps, plan_pool);
        // planner.OptimizePolicy(settings.n_steps, plan_pool);
      }
      k_mpc_++;
      // print_planner_timings();
      // Log best OCP trajectory.
      if (LOGGING_) {
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
    k_wbc_++;

    // Update filtered for observations.
    observer.update_filter(model, data);
    observer.update_collision_status(col.getCollisionStatus());
    // TODO: move collision and contact data inside Observer.

    if (RENDERING_ && (data->time - simstart > 1.0 / 60.0)) {
      update_viewer();
      simstart = data->time;
    }
  }

  observer.update_contact_status(mcontactData);
  observer.update_final_pose(model, data);
  heightmap_.update_heightmap(model, data);
  n_iteration++;
  return;
}

void MujocoSimulator::save_logger(const std::string &fileName) {
  logger_.saveData(fileName);
}

ObserverData MujocoSimulator::getObervation() {
  return observer.getObervation();
}

void MujocoSimulator::update_goal_position(std::vector<double> q) {
  if (q.size() != 6) {
    throw std::runtime_error(
        "Error in update_goal_position(), q should be size 6.");
  }
  vz_pos[0] = q[0];
  vz_pos[1] = q[1];
  vz_pos[2] = q[2];
}

Data MujocoSimulator::getLoggerData() { return logger_.getData(); }

void MujocoSimulator::set_mpc_params(int min, int max, int ratio_iter, int ratio_wbc){
  mpc_min_iteration_ = min;
  mpc_max_iteration_ = max;
  mpc_ratio_max_iteration_ = ratio_iter;
  mpc_ratio_max_wbc_ = ratio_wbc;
}

void MujocoSimulator::set_horizon_nn(double horizon_nn){
  // Reset the reference curve inside task planner.
  int indexes[2];
  ParameterIndexes(indexes, model, "residual_nn_horizon");
  task_->parameters[indexes[0]] = horizon_nn; // Boolean for horizon switch

  ParameterIndexes(indexes, model, "residual_nn_updated");
  task_->parameters[indexes[0]] = 2.; // Boolean for update horizon nn.

    // Update task
  task_->UpdateResidual();

  // Reset boolean to not update curve on next Update().
  ParameterIndexes(indexes, model, "residual_nn_updated");
  task_->parameters[indexes[0]] = -1.;
}

void MujocoSimulator::set_horizon_reset(double horizon_reset){
  // Reset the reference curve inside task planner.
  int indexes[2];
  ParameterIndexes(indexes, model, "residual_nn_horizon");
  task_->parameters[indexes[0]] = horizon_reset; // Boolean for horizon switch

  ParameterIndexes(indexes, model, "residual_nn_updated");
  task_->parameters[indexes[0]] = 3.; // Boolean for update horizon nn.

    // Update task
  task_->UpdateResidual();

  // Reset boolean to not update curve on next Update().
  ParameterIndexes(indexes, model, "residual_nn_updated");
  task_->parameters[indexes[0]] = -1.;
}

void MujocoSimulator::activate_rendering(){
  RENDERING_ = true;
}

void MujocoSimulator::set_node(stateNode node){
  // TODO : Set logger.
  if (LOGGING_){
    throw std::runtime_error("Cannot use get_node() with logger active.");
  }
  // std::cout << "node.q0 : " << node.q0[0] << std::endl;

  // Restart only necessary structures.
  mj_resetData(model, data); // reset Data
  h_actions.clear();
  h_actions = node.actions;
  h_q0 = node.q0;
  // Reset task
  task_->Reset(model);
  task_->SetParameters(model);

  // Reset state
  state_.Reset();
  state_.Set(model, data);

  // Reset planner
  planner.Reset(settings.n_steps);
  task_->UpdateResidual();
  mj_forward(model, data);
  reset_task(node.q0); // Warning q is size 6 and rpy are the last 3 elements.

  col.resetCollisionStatus();
  observer.reset(node.q0);
  observer.update_final_pose(model, data);
  observer.update_filter(model, data);

  for (int j = 0; j < node.actions.size(); j++) {
    if (j == 0) {
      first_step(node.actions.at(j));
    } else {
      update_ref_curve(node.actions.at(j));
    }
  }


  unsigned int spec = mjSTATE_INTEGRATION;
  mj_setState(model,data,node.state,spec);
  mj_forward(model, data);
  planner.policy.CopyFrom(node.policy, settings.n_steps);

  // Backward pass parameters.
  planner.backward_pass.regularization = node.regularization;  // regularization
  planner.backward_pass.regularization_rate =
      node.regularization_rate;  // regularization_rate
  planner.backward_pass.regularization_factor =
      node.regularization_factor;  // regularization_factor

  // planner.previous_policy.CopyFrom(node.previous_policy, settings.n_steps);
  // for (int i = 0; i < planner.num_trajectory_; i++) {
    // planner.candidate_policy[i].CopyFrom(node.candidate_policy[i], settings.n_steps);
    // planner.candidate_policy[i].representation = node.candidate_policy[i].representation;
    // planner.trajectory[i] = node.trajectory[i]; // candidate trajectories
  // }
  // planner.winner = node.winner;
  // planner.action_step = node.action_step;
  // planner.feedback_scaling = node.feedback_scaling;
  // planner.improvement = node.improvement;
  // planner.expected = node.expected;
  // planner.surprise = node.surprise;
  // planner.backward_pass = node.backward_pass;
  // planner.boxqp = node.boxqp;

  simstart = data->time;
  k_mpc_ = node.k_mpc;
  k_wbc_ = node.k_wbc;
  n_iteration = node.n_iteration;
  if (RENDERING_){
    update_viewer();
  }
}

stateNode MujocoSimulator::get_node(){
  // TODO : Set logger.
  if (LOGGING_){
    throw std::runtime_error("Cannot use get_node() with logger active.");
  }
  stateNode node;

  unsigned int spec = mjSTATE_INTEGRATION;
  int stateSize = mj_stateSize(model, spec);
  node.state_size = stateSize;

  // Allocating memory for state
  node.state = new mjtNum[stateSize];
  node.k_wbc = k_wbc_;
  node.k_mpc = k_mpc_;
  node.n_iteration = n_iteration;
  node.q0 = h_q0;
  node.actions = h_actions;
  node.policy.Allocate(model, *task_, mjpc::kMaxTrajectoryHorizon);
  node.policy.CopyFrom(planner.policy, settings.n_steps);

  // Backward pass parameters.
  node.regularization = planner.backward_pass.regularization;  // regularization
  node.regularization_rate =
      planner.backward_pass.regularization_rate;  // regularization_rate
  node.regularization_factor =
      planner.backward_pass.regularization_factor;  // regularization_factor

  // node.previous_policy.Allocate(model, *task_, mjpc::kMaxTrajectoryHorizon);
  // node.previous_policy.CopyFrom(planner.previous_policy, settings.n_steps);
  // node.winner = planner.winner;
  // node.action_step = planner.action_step;
  // node.feedback_scaling = planner.feedback_scaling;
  // node.improvement = planner.improvement;
  // node.expected = planner.expected;
  // node.surprise = planner.surprise;

  // dimensions
  int dim_state = model->nq + model->nv + model->na;  // state dimension
  int dim_state_derivative =
      2 * model->nv + model->na;    // state derivative dimension
  int dim_action = model->nu;           // action dimension
  int dim_sensor = model->nsensordata;  // number of sensor values
  int dim_max =
      mju_max(mju_max(mju_max(dim_state, dim_state_derivative), dim_action),
              model->nuser_sensor);

  node.nq = model->nq;
  node.na = model->na;
  node.nv = model->nv;
  node.nu = model->nu;
  // for (int i = 0; i < planner.num_trajectory_; i++) {
  //     node.candidate_policy[i].Allocate(model, *task_, mjpc::kMaxTrajectoryHorizon);
  //     node.candidate_policy[i].CopyFrom(planner.candidate_policy[i], settings.n_steps);
  //     node.candidate_policy[i].representation = planner.candidate_policy[i].representation;
  //     node.trajectory[i].Initialize(dim_state, dim_action, task_->num_residual,
  //                            task_->num_trace, mjpc::kMaxTrajectoryHorizon);
  //     node.trajectory[i].Allocate(mjpc::kMaxTrajectoryHorizon);
  //     node.trajectory[i] = planner.trajectory[i]; // candidate trajectories
  // }
  // node.backward_pass.Allocate(dim_state_derivative, dim_action,
  //                        mjpc::kMaxTrajectoryHorizon);
  // node.backward_pass = planner.backward_pass;
  // node.boxqp.Allocate(dim_action);
  // node.boxqp = planner.boxqp;

  mj_getState(model, data, node.state, spec);


  // size_t memoryInBytes = sizeof(node);
  // double memoryInMB = static_cast<double>(memoryInBytes) / (1024.0 * 1024.0); // Convert to megabytes.
  // std::cout << "State takes approximately " << memoryInMB << " MB of memory." << std::endl;

  return node;
}