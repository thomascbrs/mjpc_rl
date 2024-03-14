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

PYBIND11_MODULE(libmjpc_rl_pywrap, m) {
  m.doc() = "MuJoCo Simulator";

  py::class_<MujocoSimulator>(m, "MujocoSimulator")
      .def(py::init<int, bool, bool, const char *>())
      .def("initialize_viewer", &MujocoSimulator::initialize_viewer)
      .def("reset", &MujocoSimulator::reset)
      .def("step", &MujocoSimulator::step)
      .def("save_logger", &MujocoSimulator::save_logger)
      .def("update_goal_position", &MujocoSimulator::update_goal_position)
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