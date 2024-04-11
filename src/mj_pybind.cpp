#include "logger.h"
#include "mujoco_simulator.h"
#include "observer.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

// Non-member function for binding.
// TODO, remove loadData from Logger.
Data loadDataWithoutInstance(const std::string &fileName) {
  Logger logger;
  return logger.loadData(fileName);
}

py::dict serialize_backward_pass(const mjpc::iLQGBackwardPass& backward_pass) {
    py::dict bp_dict;
    bp_dict["Vx"] = backward_pass.Vx;
    bp_dict["Vxx"] = backward_pass.Vxx;
    bp_dict["dV"] = std::vector<double>{backward_pass.dV[0], backward_pass.dV[1]};
    bp_dict["Qx"] = backward_pass.Qx;
    bp_dict["Qu"] = backward_pass.Qu;
    bp_dict["Qxx"] = backward_pass.Qxx;
    bp_dict["Qxu"] = backward_pass.Qxu;
    bp_dict["Quu"] = backward_pass.Quu;
    bp_dict["Q_scratch"] = backward_pass.Q_scratch;
    bp_dict["regularization"] = backward_pass.regularization;
    bp_dict["regularization_rate"] = backward_pass.regularization_rate;
    bp_dict["regularization_factor"] = backward_pass.regularization_factor;
    return bp_dict;
}

mjpc::iLQGBackwardPass deserialize_backward_pass(const py::dict& bp_dict) {
    mjpc::iLQGBackwardPass backward_pass;
    backward_pass.Vx = bp_dict["Vx"].cast<std::vector<double>>();
    backward_pass.Vxx = bp_dict["Vxx"].cast<std::vector<double>>();
    std::vector<double> dV = bp_dict["dV"].cast<std::vector<double>>();
    backward_pass.dV[0] = dV[0];
    backward_pass.dV[1] = dV[1];
    backward_pass.Qx = bp_dict["Qx"].cast<std::vector<double>>();
    backward_pass.Qu = bp_dict["Qu"].cast<std::vector<double>>();
    backward_pass.Qxx = bp_dict["Qxx"].cast<std::vector<double>>();
    backward_pass.Qxu = bp_dict["Qxu"].cast<std::vector<double>>();
    backward_pass.Quu = bp_dict["Quu"].cast<std::vector<double>>();
    backward_pass.Q_scratch = bp_dict["Q_scratch"].cast<std::vector<double>>();
    backward_pass.regularization = bp_dict["regularization"].cast<double>();
    backward_pass.regularization_rate = bp_dict["regularization_rate"].cast<double>();
    backward_pass.regularization_factor = bp_dict["regularization_factor"].cast<double>();
    return backward_pass;
}

py::list serialize_trajectory(const mjpc::Trajectory& trajectory) {
  // Serialize trajectory
  py::list trajectory_list;
  trajectory_list.append(trajectory.horizon);
  trajectory_list.append(trajectory.dim_state);
  trajectory_list.append(trajectory.dim_action);
  trajectory_list.append(trajectory.dim_residual);
  trajectory_list.append(trajectory.dim_trace);
  trajectory_list.append(trajectory.states);
  trajectory_list.append(trajectory.actions);
  trajectory_list.append(trajectory.times);
  trajectory_list.append(trajectory.residual);
  trajectory_list.append(trajectory.costs);
  trajectory_list.append(trajectory.trace);
  trajectory_list.append(trajectory.total_return);
  trajectory_list.append(trajectory.failure);
  return trajectory_list;
}

mjpc::Trajectory deserialize_trajectory(const py::list& trajectory_list) {
  // Deserialize trajectory
  mjpc::Trajectory trajectory;
  trajectory.horizon = trajectory_list[0].cast<int>();
  trajectory.dim_state = trajectory_list[1].cast<int>();
  trajectory.dim_action = trajectory_list[2].cast<int>();
  trajectory.dim_residual = trajectory_list[3].cast<int>();
  trajectory.dim_trace = trajectory_list[4].cast<int>();
  // reference trajectory
  // Initialize(int dim_state, int dim_action, int dim_residual,
  // No need for Initialize, since it fills horizon, dim_trace ...etc
  trajectory.Allocate(trajectory.horizon);
  trajectory.states = trajectory_list[5].cast<std::vector<double>>();
  trajectory.actions = trajectory_list[6].cast<std::vector<double>>();
  trajectory.times = trajectory_list[7].cast<std::vector<double>>();
  trajectory.residual = trajectory_list[8].cast<std::vector<double>>();
  trajectory.costs = trajectory_list[9].cast<std::vector<double>>();
  trajectory.trace = trajectory_list[10].cast<std::vector<double>>();
  trajectory.total_return = trajectory_list[11].cast<double>();
  trajectory.failure = trajectory_list[12].cast<bool>();
  return trajectory;
}

py::dict serialize_policy(const mjpc::iLQGPolicy& policy){
  py::dict bp_policy;
  bp_policy["trajectory"] = serialize_trajectory(policy.trajectory);
  bp_policy["feedback_gain"] = policy.feedback_gain;
  bp_policy["action_improvement"] = policy.action_improvement;
  // bp_policy["state_scratch"] = policy.state_scratch;
  // bp_policy["action_scratch"] = policy.action_scratch;
  // bp_policy["feedback_gain_scratch"] = policy.feedback_gain_scratch;
  // bp_policy["state_interp"] = policy.state_interp;
  // bp_policy["representation"] = policy.representation;
  // bp_policy["feedback_scaling"] = policy.feedback_scaling;
  return bp_policy;
}

mjpc::iLQGPolicy deserialize_policy(const py::dict& bp_policy,const py::dict& bp_model){
  mjpc::iLQGPolicy policy;
  policy.trajectory = deserialize_trajectory(bp_policy["trajectory"].cast<py::tuple>());
  int kMaxTrajectoryHorizon = policy.trajectory.horizon;
  int nq = bp_model["nq"].cast<int>();
  int nv = bp_model["nv"].cast<int>();
  int na = bp_model["na"].cast<int>();
  int nu = bp_model["nu"].cast<int>();
  // Allocate memory.
  policy.feedback_gain.resize(nu * (2 * nv + na) *
                       kMaxTrajectoryHorizon);
  policy.action_improvement.resize(nu * kMaxTrajectoryHorizon);
  // policy.state_scratch.resize(nq + nv + na);
  // policy.action_scratch.resize(nu);
  // policy.feedback_gain_scratch.resize(nu * (2 * nv + na));
  // policy.state_interp.resize(nq + nv + na);

  policy.feedback_gain = bp_policy["feedback_gain"].cast<std::vector<double>>();
  policy.action_improvement = bp_policy["action_improvement"].cast<std::vector<double>>();
  // policy.state_scratch = bp_policy["state_scratch"].cast<std::vector<double>>();
  // policy.action_scratch = bp_policy["action_scratch"].cast<std::vector<double>>();
  // policy.feedback_gain_scratch = bp_policy["feedback_gain_scratch"].cast<std::vector<double>>();
  // policy.state_interp = bp_policy["state_interp"].cast<std::vector<double>>();
  // policy.representation = bp_policy["representation"].cast<int>();
  // policy.feedback_scaling = bp_policy["feedback_scaling"].cast<double>();
  return policy;
}

py::dict serialize_model_dimension(const stateNode& node){
  py::dict bp_model;
  bp_model["nq"] = node.nq;
  bp_model["nv"] = node.nv;
  bp_model["na"] = node.na;
  bp_model["nu"] = node.nu;
  return bp_model;
}

py::dict serialize_backward_infos(const stateNode& node){
  py::dict bp_infos;
  bp_infos["regularization"] = node.regularization;
  bp_infos["regularization_rate"] = node.regularization_rate;
  bp_infos["regularization_factor"] = node.regularization_factor;
  return bp_infos;
}

// Define the reduce_stateNode function for serialization
py::tuple reduce_stateNode(const stateNode& node) {
    // Copy state array to vector
    std::vector<mjtNum> state_vector(node.state, node.state + node.state_size);

    py::dict bp_model = serialize_model_dimension(node);
    py::dict bp_policy = serialize_policy(node.policy);
    py::dict bp_backward_infos = serialize_backward_infos(node);
    // 15 Mb. Otherwise 2kB
    // py::dict backward_pass = serialize_backward_pass(node.backward_pass);

    return py::make_tuple(state_vector, node.k_wbc, node.k_mpc, node.n_iteration, node.q0, node.actions, bp_model, bp_policy, bp_backward_infos);
}

// Define the setstate_stateNode function for deserialization
stateNode setstate_stateNode(const py::tuple& state) {
    stateNode node;

    // Deserialize state vector
    std::vector<mjtNum> state_vec = state[0].cast<std::vector<mjtNum>>();
    node.state = new mjtNum[state_vec.size()];
    std::copy(state_vec.begin(),
              state_vec.end(),
              node.state);
    // node.state_size = state[0].cast<std::vector<mjtNum>>().size();

    node.k_wbc = state[1].cast<int>();
    node.k_mpc = state[2].cast<int>();
    node.n_iteration = state[3].cast<int>();
    node.q0 = state[4].cast<std::vector<double>>();
    node.actions = state[5].cast<std::vector<std::vector<double>>>();

    // Get model dimension to allocate memory for policy.
    py::dict bp_model = state[6].cast<py::dict>();
    node.nq = bp_model["nq"].cast<int>();
    node.nv = bp_model["nv"].cast<int>();
    node.na = bp_model["na"].cast<int>();
    node.nu = bp_model["nu"].cast<int>();
    py::dict bp_policy = state[7].cast<py::dict>();
    node.policy = deserialize_policy(bp_policy, bp_model);

    py::dict bp_infos = state[8].cast<py::dict>();
    node.nq = bp_infos["regularization"].cast<double>();
    node.nv = bp_infos["regularization_rate"].cast<double>();
    node.na = bp_infos["regularization_factor"].cast<double>();

    // py::dict backward_pass = state[7].cast<py::dict>();
    // node.backward_pass = deserialize_backward_pass(backward_pass);

    return node;
}

PYBIND11_MODULE(libmjpc_rl_pywrap, m) {
  m.doc() = "MuJoCo Simulator";

  py::class_<stateNode>(m, "stateNode")
      .def(py::pickle(&reduce_stateNode, &setstate_stateNode));

  py::class_<MujocoSimulator>(m, "MujocoSimulator")
      .def(py::init<int, bool, bool, const char *>())
      .def("initialize_viewer", &MujocoSimulator::initialize_viewer)
      .def("reset", (void (MujocoSimulator::*)(const std::vector<double>&, int, const std::vector<double>&)) &MujocoSimulator::reset)
      .def("reset", (void (MujocoSimulator::*)(const std::vector<double>&, int)) &MujocoSimulator::reset0)
      .def("step", &MujocoSimulator::step)
      .def("save_logger", &MujocoSimulator::save_logger)
      .def("update_goal_position", &MujocoSimulator::update_goal_position)
      .def("getLoggerData", &MujocoSimulator::getLoggerData)
      .def("getHeightmap", &MujocoSimulator::getHeightmap)
      .def("set_mpc_params", &MujocoSimulator::set_mpc_params)
      .def("get_horizon_nn", &MujocoSimulator::get_horizon_nn)
      .def("get_node", &MujocoSimulator::get_node)
      .def("set_node", &MujocoSimulator::set_node)
      .def("getObervation", &MujocoSimulator::getObervation);

  py::class_<Data>(m, "Data")
      .def(py::init<>())
      .def_readwrite("size", &Data::size)
      .def_readwrite("mpc_iteration", &Data::mpc_iteration)
      .def_readwrite("horizon", &Data::horizon)
      .def_readwrite("dt_mpc", &Data::dt_mpc)
      .def_readwrite("k_mpc", &Data::k_mpc)
      .def_readwrite("dt_simu", &Data::dt_simu)
      .def_readwrite("qpos", &Data::qpos)
      .def_readwrite("qvel", &Data::qvel)
      .def_readwrite("foot_status", &Data::foot_status)
      .def_readwrite("foot_position", &Data::foot_position)
      .def_readwrite("foot_velocity", &Data::foot_velocity)
      .def_readwrite("contact_forces", &Data::contact_forces)
      .def_readwrite("qvel_fil", &Data::qvel_fil)
      .def_readwrite("qpos_fil", &Data::qpos_fil)
      .def_readwrite("contact_forces_sensors", &Data::contact_forces_sensors)
      .def_readwrite("mpc_traj", &Data::mpc_traj);

  py::class_<ObserverData>(m, "ObserverData")
      .def(py::init<>())
      .def_readwrite("foot_names", &ObserverData::foot_names)
      .def_readwrite("end_pose", &ObserverData::end_pose)
      .def_readwrite("end_vel", &ObserverData::end_vel)
      .def_readwrite("end_acc", &ObserverData::end_acc)
      .def_readwrite("filtered_pose", &ObserverData::filtered_pose)
      .def_readwrite("filtered_vel", &ObserverData::filtered_vel)
      .def_readwrite("feet_pos", &ObserverData::feet_pos)
      .def_readwrite("feet_vel", &ObserverData::feet_vel)
      .def_readwrite("lfeet_pos", &ObserverData::lfeet_pos)
      .def_readwrite("lfeet_vel", &ObserverData::lfeet_pos)
      .def_readwrite("lvref", &ObserverData::lvref)
      .def_readwrite("orientation_ref", &ObserverData::orientation_ref)
      .def_readwrite("sq_height", &ObserverData::sq_height)
      .def_readwrite("sq_angle", &ObserverData::sq_angle)
      .def_readwrite("sq_vel", &ObserverData::sq_vel)
      .def_readwrite("sq_control", &ObserverData::sq_control)
      .def_readwrite("collision_status", &ObserverData::collision_status)
      .def_readwrite("contact_status", &ObserverData::contact_status);

  m.def("loadData", &loadDataWithoutInstance,
        "Load data from file and return as Data struct");
}