#include "mujoco_simulator.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

PYBIND11_MODULE(libmjpc_rl_pywrap, m) {
  m.doc() = "MuJoCo Simulator";

  py::class_<MujocoSimulator>(m, "MujocoSimulator")
      .def(py::init<const char *>())
      .def("initialize", &MujocoSimulator::initialize)
      .def("run_simulation", &MujocoSimulator::runSimulation)
      .def("get_logged_joint_positions",
           &MujocoSimulator::getLoggedJointPositions);
}