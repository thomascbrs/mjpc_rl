#include "mujoco_simulator.h"
#include "logger.h"
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

// Non-member function for binding.
// TODO, remove loadData from Logger.
Data loadDataWithoutInstance(const std::string& fileName) {
    Logger logger;
    return logger.loadData(fileName);
}

PYBIND11_MODULE(libmjpc_rl_pywrap, m) {
  m.doc() = "MuJoCo Simulator";

  py::class_<MujocoSimulator>(m, "MujocoSimulator")
      .def(py::init<const char *>())
      .def("initialize", &MujocoSimulator::initialize)
      .def("run_simulation", &MujocoSimulator::runSimulation)
      .def("get_logged_joint_positions",
           &MujocoSimulator::getLoggedJointPositions);

  py::class_<Data>(m, "Data")
        .def(py::init<>())
        .def_readwrite("size", &Data::size)
        .def_readwrite("P", &Data::P)
        .def_readwrite("D", &Data::D)
        .def_readwrite("foot_status", &Data::foot_status)
        .def_readwrite("foot_position", &Data::foot_position);

  m.def("loadData", &loadDataWithoutInstance, "Load data from file and return as Data struct");
}