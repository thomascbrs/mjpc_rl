#include "custom_planner.h"
using mjpc::LogScale;
using mjpc::GetDuration;
using mjpc::DataAt;

void CustomiLQGPlanner::OptimizePolicyCustom(int horizon, mjpc::ThreadPool& pool){
  this->NominalTrajectoryCustom(horizon, pool);
  this->IterationCustom(horizon, pool);
}

void CustomiLQGPlanner::NominalTrajectoryCustom(int horizon, mjpc::ThreadPool& pool){
  if (num_trajectory_ == 0) {
    return;
  }
  // resize data for rollouts
  ResizeMjData(model, pool.NumThreads());

  // step sizes (log scaling)
  LogScale(linesearch_steps, 1.0, settings.min_linesearch_step,
           num_trajectory_ - 1);
  linesearch_steps[num_trajectory_ - 1] = 0.0;

  // ----- nominal rollout ----- //
  // start timer
  auto nominal_start = std::chrono::steady_clock::now();

  // no one else should be writing, but we lock just in case:
  {
    const std::shared_lock<std::shared_mutex> lock(mtx_);
    for (int i = 0; i < num_trajectory_; i++) {
      candidate_policy[i].CopyFrom(policy, horizon);
      candidate_policy[i].representation = policy.representation;
    }
  }

  // feedback rollouts (parallel)
  this->FeedbackRollouts(horizon, pool);

  // evaluate rollouts
  int best_nominal = this->BestRollout();

  // check for all rollout failures
  if (best_nominal == -1) {
    // The policy's traj's first state is not the correct initial state, but
    // what alternative do we have, if all rollouts from the current initial
    // state failed?
    {
      const std::shared_lock<std::shared_mutex> lock(mtx_);
      candidate_policy[0].trajectory = policy.trajectory;
    }

    // set feedback scaling
    feedback_scaling = 0.0;
  } else {
    // update nominal with winner
    candidate_policy[0].trajectory = trajectory[best_nominal];

    // set feedback scaling
    feedback_scaling = linesearch_steps[best_nominal];
  }

  // end timer
  nominal_compute_time = GetDuration(nominal_start);
}

// single iLQG iteration
void CustomiLQGPlanner::IterationCustom(int horizon, mjpc::ThreadPool& pool) {
  // set previous best cost
  double previous_return = candidate_policy[0].trajectory.total_return;

  // ----- setup ----- //
  // resize data for rollouts
  ResizeMjData(model, pool.NumThreads());

  // step sizes (log scaling)
  LogScale(linesearch_steps, 1.0, settings.min_linesearch_step,
           num_trajectory_ - 1);
  linesearch_steps[num_trajectory_ - 1] = 0.0;

  // ----- model derivatives ----- //
  // start timer
  auto model_derivative_start = std::chrono::steady_clock::now();

  // compute model and sensor Jacobians
  model_derivative.Compute(
      model, data_, candidate_policy[0].trajectory.states.data(),
      candidate_policy[0].trajectory.actions.data(),
      candidate_policy[0].trajectory.times.data(), dim_state,
      dim_state_derivative, dim_action, dim_sensor, horizon,
      settings.fd_tolerance, settings.fd_mode, pool);

  // stop timer
  double model_derivative_time = GetDuration(model_derivative_start);

  // ----- cost derivatives ----- //
  // start timer
  auto cost_derivative_start = std::chrono::steady_clock::now();

  // cost derivatives
  cost_derivative.Compute(
      candidate_policy[0].trajectory.residual.data(), model_derivative.C.data(),
      model_derivative.D.data(), dim_state_derivative, dim_action, dim_max,
      dim_sensor, task->num_residual, task->dim_norm_residual.data(),
      task->num_term, task->weight.data(), task->norm.data(),
      task->norm_parameter.data(), task->num_norm_parameter.data(), task->risk,
      horizon, pool);

  // end timer
  double cost_derivative_time = GetDuration(cost_derivative_start);

  // ----- backward pass ----- //
  // start timer
  auto backward_pass_start = std::chrono::steady_clock::now();

  // initialize backward pass
  int regularization_iteration = 0;
  int backward_pass_status = 0;
  int t;
  while (regularization_iteration < settings.max_regularization_iterations &&
         backward_pass_status == 0) {
    // reset cost-to-go approximation difference
    mju_zero(backward_pass.dV, 2);

    // terminal time step cost-to-go
    mju_copy(DataAt(backward_pass.Vx, (horizon - 1) * dim_state_derivative),
             DataAt(cost_derivative.cx, (horizon - 1) * dim_state_derivative),
             dim_state_derivative);
    mju_copy(DataAt(backward_pass.Vxx, (horizon - 1) * dim_state_derivative *
                                           dim_state_derivative),
             DataAt(cost_derivative.cxx, (horizon - 1) * dim_state_derivative *
                                             dim_state_derivative),
             dim_state_derivative * dim_state_derivative);

    // backward recursion
    for (t = horizon - 2; t >= 0; t--) {
      int status = backward_pass.RiccatiStep(
          dim_state_derivative, dim_action, backward_pass.regularization,
          DataAt(backward_pass.Vx, (t + 1) * dim_state_derivative),
          DataAt(backward_pass.Vxx,
                 (t + 1) * dim_state_derivative * dim_state_derivative),
          DataAt(model_derivative.A,
                 t * dim_state_derivative * dim_state_derivative),
          DataAt(model_derivative.B, t * dim_state_derivative * dim_action),
          DataAt(cost_derivative.cx, t * dim_state_derivative),
          DataAt(cost_derivative.cu, t * dim_action),
          DataAt(cost_derivative.cxx,
                 t * dim_state_derivative * dim_state_derivative),
          DataAt(cost_derivative.cxu, t * dim_state_derivative * dim_action),
          DataAt(cost_derivative.cuu, t * dim_action * dim_action),
          DataAt(backward_pass.Vx, t * dim_state_derivative),
          DataAt(backward_pass.Vxx,
                 t * dim_state_derivative * dim_state_derivative),
          DataAt(candidate_policy[0].action_improvement, t * dim_action),
          DataAt(candidate_policy[0].feedback_gain,
                 t * dim_action * dim_state_derivative),
          backward_pass.dV, DataAt(backward_pass.Qx, t * dim_state_derivative),
          DataAt(backward_pass.Qu, t * dim_action),
          DataAt(backward_pass.Qxx,
                 t * dim_state_derivative * dim_state_derivative),
          DataAt(backward_pass.Qxu, t * dim_state_derivative * dim_action),
          DataAt(backward_pass.Quu, t * dim_action * dim_action),
          backward_pass.Q_scratch.data(), boxqp,
          DataAt(candidate_policy[0].trajectory.actions, t * dim_action),
          model->actuator_ctrlrange, settings.regularization_type,
          settings.action_limits);

      // failure
      if (!status) {
        // information
        if (settings.verbose) {
          printf("Backward Pass Failure (%i / %i)\n", regularization_iteration,
                 settings.max_regularization_iterations);
          printf("  time index: %i\n", t);  // Note
          printf("  simulation time: %f\n", time);
          printf("  regularization: %f\n", backward_pass.regularization);
          printf("  regularization factor: %f\n",
                 backward_pass.regularization_factor);
        }
        break;
      }

      // complete
      if (t == 0) {
        // set feedback gains and improvement at final time step
        mju_copy(DataAt(candidate_policy[0].feedback_gain,
                        (horizon - 1) * dim_action * dim_state_derivative),
                 DataAt(candidate_policy[0].feedback_gain,
                        (horizon - 2) * dim_action * dim_state_derivative),
                 dim_action * dim_state_derivative);
        mju_copy(DataAt(candidate_policy[0].action_improvement,
                        (horizon - 1) * dim_action),
                 DataAt(candidate_policy[0].action_improvement,
                        (horizon - 2) * dim_action),
                 dim_action);

        // backward pass status -> success
        backward_pass_status = 1;
        break;
      }
    }

    // increase regularization
    if (backward_pass.regularization <= settings.max_regularization &&
        backward_pass_status == 0) {
      backward_pass.ScaleRegularization(backward_pass.regularization_factor,
                                        settings.min_regularization,
                                        settings.max_regularization);
      regularization_iteration += 1;
    }
  }

  // end timer
  double backward_pass_time = GetDuration(backward_pass_start);

  // terminate early if backward pass failure
  if (backward_pass_status == 0) {
    // set timers
    model_derivative_compute_time = model_derivative_time;
    cost_derivative_compute_time = cost_derivative_time;
    rollouts_compute_time = 0.0;
    backward_pass_compute_time = backward_pass_time;
    policy_update_compute_time = 0.0;
    return;
  }

  // ----- rollout policy ----- //
  auto rollouts_start = std::chrono::steady_clock::now();

  // copy policy
  for (int j = 1; j < num_trajectory_; j++) {
    candidate_policy[j].CopyFrom(candidate_policy[0], horizon);
    candidate_policy[j].representation = candidate_policy[0].representation;
  }


  //////////////////////////////////////////////////////
  // Modify properties of contact.
  // solimp="0.015 0.7 0.04 0.5 2" solref="0.02 1"
  // IN practise, mean of
  // model->opt.o_solimp[0] = 0.95;

  // model->opt.o_margin = 0.001;
  // model->opt.o_solimp[0] = 0.95;
  // model->opt.o_solimp[1] = 0.83;
  // model->opt.o_solimp[2] = 0.04;

  // model->opt.o_solimp[0] = 0.9;
  // model->opt.o_solimp[1] = 0.95;
  // model->opt.o_solimp[2] = 0.001;
  // model->opt.o_margin = 0.001;


  // feedback rollouts (parallel)
  this->ActionRollouts(horizon, pool);

  // ----- evaluate rollouts ----- //

  // get best rollout
  int best_rollout = this->BestRollout();
  if (best_rollout == -1) {
    return;
  } else {
    winner = best_rollout;
  }

  // update nominal with winner
  candidate_policy[0].trajectory = trajectory[winner];

  // improvement
  action_step = linesearch_steps[winner];
  expected = -1.0 * action_step *
                 (backward_pass.dV[0] + action_step * backward_pass.dV[1]) +
             1.0e-16;
  improvement = previous_return - trajectory[winner].total_return;
  surprise = mju_min(mju_max(0, improvement / expected), 2);

  // update regularization
  backward_pass.UpdateRegularization(settings.min_regularization,
                                     settings.max_regularization, surprise,
                                     action_step);

  if (settings.verbose) {
    std::cout << "iLQG Information\n" << '\n';
    std::cout << "  best return: " << trajectory[winner].total_return << '\n';
    std::cout << "  previous return: " << previous_return << '\n';
    std::cout << "  nominal return: " << policy.trajectory.total_return << '\n';
    std::cout << "  linesearch step size: " << action_step << '\n';
    std::cout << "  improvement: " << improvement << '\n';
    std::cout << "  regularization: " << backward_pass.regularization << '\n';
    std::cout << "  regularization factor: "
              << backward_pass.regularization_factor << '\n';
    std::cout << "  dV: " << expected << '\n';
    std::cout << "  dV[0]: " << backward_pass.dV[0] << '\n';
    std::cout << "  dV[1]: " << backward_pass.dV[1] << '\n';
    std::cout << std::endl;
  }

  // stop timer
  double rollouts_time = GetDuration(rollouts_start);

  // ----- policy update ----- //
  // start timer
  auto policy_update_start = std::chrono::steady_clock::now();
  {
    const std::shared_lock<std::shared_mutex> lock(mtx_);
    // improvement
    previous_policy = policy;
    policy.CopyFrom(candidate_policy[winner], horizon);

    // feedback scaling
    policy.feedback_scaling = 1.0;
  }

  // stop timer
  double policy_update_time = GetDuration(policy_update_start);

  // set timers
  model_derivative_compute_time = model_derivative_time;
  cost_derivative_compute_time = cost_derivative_time;
  rollouts_compute_time = rollouts_time;
  backward_pass_compute_time = backward_pass_time;
  policy_update_compute_time = policy_update_time;
}

