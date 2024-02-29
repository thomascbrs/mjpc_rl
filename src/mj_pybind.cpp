#include "logger.h"
#include "mujoco_simulator.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

// Non-member function for binding.
// TODO, remove loadData from Logger.
Data loadDataWithoutInstance(const std::string &fileName) {
  Logger logger;
  return logger.loadData(fileName);
}

PYBIND11_MODULE(libmjpc_rl_pywrap, m) {
  m.doc() = "MuJoCo Simulator";

  py::class_<MujocoSimulator>(m, "MujocoSimulator")
      .def(py::init<int, bool, bool, const char *>())
      .def("initialize_viewer", &MujocoSimulator::initialize_viewer)
      .def("reset", &MujocoSimulator::reset)
      .def("run_simulation", &MujocoSimulator::runSimulation)
      .def("step", &MujocoSimulator::step)
      .def("get_logged_joint_positions",
           &MujocoSimulator::getLoggedJointPositions);

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
      .def_readwrite("contact_forces_sensors", &Data::contact_forces_sensors)
      .def_readwrite("mpc_traj", &Data::mpc_traj);

  m.def("loadData", &loadDataWithoutInstance,
        "Load data from file and return as Data struct");
}