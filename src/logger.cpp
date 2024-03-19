#include "logger.h"


#include "pinocchio/math/rpy.hpp"
#include "pinocchio/spatial/se3.hpp"
#include <pinocchio/math/quaternion.hpp>
#include <Eigen/Geometry>
#include <filesystem>  // For C++17 and later

namespace fs = std::filesystem;

Logger::Logger()
    : filter_pos_(cutoff_vel, fs_vel, order_vel),
      filter_vel_(cutoff_vel, fs_vel, order_vel) {
  int max_size = 5000;
  data_.size = 0;
  data_.mpc_iteration = 0;
  data_.qpos.reserve(5000);
  data_.qpos_fil.reserve(5000);
  data_.qvel.reserve(5000);
  data_.qvel_fil.reserve(5000);
}

Logger::~Logger() {}

void Logger::Initialize(const std::vector<std::string> &foot_names,
                        const double dt_mpc, const int horizon, const int k_mpc,
                        const double dt_simu) {
  foot_names_ = foot_names;
  data_.dt_mpc = dt_mpc;
  data_.horizon = horizon;
  data_.k_mpc = k_mpc;
  data_.dt_simu = dt_simu;

  // Reserve memory for the foot contact status in the data.
  for (auto &name : foot_names) {
    std::vector<int> tmp;
    std::vector<int> tmp_touch;
    std::vector<std::array<double, 3>> tmp_pos;
    std::vector<std::array<double, 3>> tmp_vel;
    std::vector<std::array<double, 3>> tmp_forces;
    std::vector<std::array<double, 3>> tmp_sforces;
    tmp.reserve(10000);
    tmp_pos.reserve(10000);
    tmp_touch.reserve(10000);
    tmp_vel.reserve(10000);
    tmp_forces.reserve(10000);
    tmp_sforces.reserve(10000);
    data_.foot_status[name] = tmp;
    data_.foot_status_touch[name] = tmp;
    data_.foot_position[name] = tmp_pos;
    data_.foot_velocity[name] = tmp_vel;
    data_.contact_forces[name] = tmp_forces;
    data_.contact_forces_sensors[name] = tmp_sforces;
  }
}

void Logger::log(const mjModel *model, mjData *data,
                 const ContactData *mcontactData) {
  logState(model, data);
  logFeetStatus(mcontactData->contact_status);
  logFeetPosition(model, data);
  logFeetVelocity(model, data);
  logFeetTouch(model, data);
  logFeetForces(mcontactData->contact_forces);
  logFeetForcesSensors(mcontactData->contact_forces_sensors);
};

void Logger::logFeetStatus(
    const std::unordered_map<std::string, int> &contact_status) {
  for (const auto &status : contact_status) {
    data_.foot_status[status.first].push_back(status.second);
  }
  data_.size++;
}

void Logger::logFeetForces(
    const std::unordered_map<std::string, std::array<double, 3>>
        &contact_forces) {
  for (const auto &status : contact_forces) {
    data_.contact_forces[status.first].push_back(status.second);
  }
}

void Logger::logFeetForcesSensors(
    const std::unordered_map<std::string, std::array<double, 3>>
        &contact_forces_sensors) {
  for (const auto &status : contact_forces_sensors) {
    data_.contact_forces_sensors[status.first].push_back(status.second);
  }
};

void Logger::logFeetPosition(const mjModel *model, mjData *data) {
  for (const auto &name : foot_names_) {
    const double *foot_pos = mjpc::SensorByName(model, data, name);
    std::array<double, 3> tmp_arr;

    // Copy the elements from foot_pos to temp_array
    std::copy(foot_pos, foot_pos + 3, tmp_arr.begin());
    data_.foot_position[name].push_back(tmp_arr);
  }
}

void Logger::logFeetVelocity(const mjModel *model, mjData *data) {
  for (const auto &name : foot_names_) {
    const double *foot_pos = mjpc::SensorByName(model, data, name + "_vel");
    std::array<double, 3> tmp_arr;

    // Copy the elements from foot_pos to temp_array
    std::copy(foot_pos, foot_pos + 3, tmp_arr.begin());
    data_.foot_velocity[name].push_back(tmp_arr);
  }
}

void Logger::logFeetTouch(const mjModel *model, mjData *data) {
  for (const auto &name : foot_names_) {
    // Disabled to reduce computing time. Each derivative wrt sensor is computed.
    // const double *touch = mjpc::SensorByName(model, data, name + "_touch");
    double touch[1];
    touch[0] = 0.;
    if (touch[0] < 0.01) {
      data_.foot_status_touch[name].push_back(1);
    } else {
      data_.foot_status_touch[name].push_back(0);
    }
  }
}

void Logger::logMPC(const mjpc::Trajectory *trajectory) {
  std::vector<std::array<double, 37>> tmp_states;
  for (int n = 0; n < trajectory->horizon; n++) {
    std::array<double, 37> tmp_;
    std::copy(trajectory->states.begin() + 37 * n,
              trajectory->states.begin() + 37 * (n + 1), tmp_.begin());
    tmp_states.push_back(tmp_);
  }
  data_.mpc_traj.push_back(tmp_states);
  data_.mpc_iteration += 1;
}

void Logger::logState(const mjModel *model, mjData *data) {
  std::array<double, 19> qpos;
  std::copy(data->qpos, data->qpos + 19, qpos.begin());
  data_.qpos.emplace_back(qpos);

  // Need to compute RPY.
  mjtNum R_data[9];
  mjtNum quat_tmp[4];
  Matrix3d R_tmp;
  mju_quat2Mat(R_data,&data->qpos[3]);  // Convert quaternion to rotation matrix
  updateMatrix(R_tmp, R_data);
  Vector3d rpy;
  rpy = pinocchio::rpy::matrixToRpy(Eigen::Quaterniond(data->qpos[3], data->qpos[4], data->qpos[5], data->qpos[6]).toRotationMatrix());

  std::array<double, 18> qpos_tmp;
  // Reconstruct qpos array with RPY.
  std::copy(data->qpos, data->qpos + 3, qpos_tmp.begin()); // Copy the first 3 elements from data->qpos
  std::copy(rpy.data(), rpy.data() + 3, qpos_tmp.begin() + 3); // Copy RPY angles
  std::copy(data->qpos + 6, data->qpos + 19, qpos_tmp.begin() + 6); // Copy the remaining elements from data->qpos
  std::array<double,6> tmp_pos = filter_pos_.filter(qpos_tmp);
  data_.qpos_fil.emplace_back(tmp_pos);

  std::array<double, 18> qvel;
  std::copy(data->qvel, data->qvel + 18, qvel.begin());
  data_.qvel.emplace_back(qvel);

  // Log qvel filtered
  std::array<double,6> tmp_test = filter_vel_.filter(qvel);
  data_.qvel_fil.emplace_back(tmp_test);
}

void Logger::saveData(const std::string &fileName) {
  // Extract directory path from the file path
  fs::path directoryPath = fs::path(fileName).parent_path();

  // Check if the directory exists
  if (!fs::exists(directoryPath)) {
      // Create the directory if it doesn't exist
      if (!fs::create_directories(directoryPath)) {
          std::cerr << "Failed to create directory: " << directoryPath << std::endl;
          return;  // Exit the function if directory creation fails
      }
  }
  std::ofstream file(fileName,
                     std::ios::out | std::ios::binary | std::ios::trunc);
  if (file.is_open()) {
    file.write(reinterpret_cast<const char *>(&data_.size), sizeof(int));
    file.write(reinterpret_cast<const char *>(&data_.mpc_iteration),
               sizeof(int));
    file.write(reinterpret_cast<const char *>(&data_.horizon), sizeof(int));
    file.write(reinterpret_cast<const char *>(&data_.dt_mpc), sizeof(double));
    file.write(reinterpret_cast<const char *>(&data_.k_mpc), sizeof(double));
    file.write(reinterpret_cast<const char *>(&data_.dt_simu), sizeof(double));

    // Save foot_status_ data
    for (const auto &entry : data_.foot_status) {
      // Save the key (foot name)
      file.write(entry.first.c_str(), entry.first.size() * sizeof(char));
      file.write("\0", sizeof(char)); // Null-terminate the string

      // Save the vector of integers
      file.write(reinterpret_cast<const char *>(entry.second.data()),
                 entry.second.size() * sizeof(int));
    }

    // Save foot_position data
    for (const auto &entry : data_.foot_position) {
      // Save the key (foot name)
      file.write(entry.first.c_str(), entry.first.size() * sizeof(char));
      file.write("\0", sizeof(char)); // Null-terminate the string

      // Save the vector of integers
      file.write(reinterpret_cast<const char *>(entry.second.data()),
                 entry.second.size() * sizeof(std::array<double, 3>));
    }

    // Save foot_velocity data
    for (const auto &entry : data_.foot_velocity) {
      // Save the key (foot name)
      file.write(entry.first.c_str(), entry.first.size() * sizeof(char));
      file.write("\0", sizeof(char)); // Null-terminate the string

      // Save the vector of integers
      file.write(reinterpret_cast<const char *>(entry.second.data()),
                 entry.second.size() * sizeof(std::array<double, 3>));
    }

    // Save contact forces data
    for (const auto &entry : data_.contact_forces) {
      // Save the key (foot name)
      file.write(entry.first.c_str(), entry.first.size() * sizeof(char));
      file.write("\0", sizeof(char)); // Null-terminate the string

      // Save the vector of integers
      file.write(reinterpret_cast<const char *>(entry.second.data()),
                 entry.second.size() * sizeof(std::array<double, 3>));
    }

    // Save contact forces from sensors
    for (const auto &entry : data_.contact_forces_sensors) {
      // Save the key (foot name)
      file.write(entry.first.c_str(), entry.first.size() * sizeof(char));
      file.write("\0", sizeof(char)); // Null-terminate the string

      // Save the vector of integers
      file.write(reinterpret_cast<const char *>(entry.second.data()),
                 entry.second.size() * sizeof(std::array<double, 3>));
    }

    // Load qpos and qvel
    file.write(reinterpret_cast<const char *>(data_.qpos.data()),
               data_.qpos.size() * sizeof(std::array<double, 19>));
    file.write(reinterpret_cast<const char *>(data_.qpos_fil.data()),
               data_.qpos_fil.size() * sizeof(std::array<double, 6>));
    file.write(reinterpret_cast<const char *>(data_.qvel.data()),
               data_.qvel.size() * sizeof(std::array<double, 18>));
    file.write(reinterpret_cast<const char *>(data_.qvel_fil.data()),
               data_.qvel_fil.size() * sizeof(std::array<double, 6>));

    // Save mpc trajectories
    for (size_t i = 0; i < data_.mpc_traj.size(); i++) {
      file.write(reinterpret_cast<char *>(data_.mpc_traj.at(i).data()),
                 data_.mpc_traj.at(i).size() * sizeof(std::array<double, 37>));
    }

    file.close();
  } else {
    std::cout << "Error while opening the file" << std::endl;
  }
}

Data Logger::loadData(const std::string &fileName) {
  Data data;

  std::ifstream file(fileName, std::ios::binary);
  if (file.is_open()) {
    file.read(reinterpret_cast<char *>(&data.size), sizeof(int));
    file.read(reinterpret_cast<char *>(&data.mpc_iteration), sizeof(int));
    file.read(reinterpret_cast<char *>(&data.horizon), sizeof(int));
    file.read(reinterpret_cast<char *>(&data.dt_mpc), sizeof(double));
    file.read(reinterpret_cast<char *>(&data.k_mpc), sizeof(double));
    file.read(reinterpret_cast<char *>(&data.dt_simu), sizeof(double));

    // Load data specific to contact status
    for (size_t i = 0; i < 4; i++) {
      // Read the key
      std::string footName;
      char c;
      while ((file.get(c)) && (c != '\0')) {
        footName += c;
      }
      std::vector<int> footData(data.size);
      file.read(reinterpret_cast<char *>(footData.data()),
                footData.size() * sizeof(int));
      data.foot_status[footName] = footData;
    }

    // Load data specific to foot position.
    for (size_t i = 0; i < 4; ++i) {
      // Read the key
      std::string footName;
      char c;
      while ((file.get(c)) && (c != '\0')) {
        footName += c;
      }
      std::vector<std::array<double, 3>> footData(data.size);
      file.read(reinterpret_cast<char *>(footData.data()),
                footData.size() * sizeof(std::array<double, 3>));
      data.foot_position[footName] = footData;
    }

    // Load data specific to foot velocity.
    for (size_t i = 0; i < 4; ++i) {
      // Read the key
      std::string footName;
      char c;
      while ((file.get(c)) && (c != '\0')) {
        footName += c;
      }
      std::vector<std::array<double, 3>> footData(data.size);
      file.read(reinterpret_cast<char *>(footData.data()),
                footData.size() * sizeof(std::array<double, 3>));
      data.foot_velocity[footName] = footData;
    }

    // Load data specific to contact forces.
    for (size_t i = 0; i < 4; ++i) {
      // Read the key
      std::string footName;
      char c;
      while ((file.get(c)) && (c != '\0')) {
        footName += c;
      }
      std::vector<std::array<double, 3>> footData(data.size);
      file.read(reinterpret_cast<char *>(footData.data()),
                footData.size() * sizeof(std::array<double, 3>));
      data.contact_forces[footName] = footData;
    }

    // Load data specific to contact forces sensors.
    for (size_t i = 0; i < 4; ++i) {
      // Read the key
      std::string footName;
      char c;
      while ((file.get(c)) && (c != '\0')) {
        footName += c;
      }
      std::vector<std::array<double, 3>> footData(data.size);
      file.read(reinterpret_cast<char *>(footData.data()),
                footData.size() * sizeof(std::array<double, 3>));
      data.contact_forces_sensors[footName] = footData;
    }

    data.qpos.resize(data.size);
    data.qpos_fil.resize(data.size);
    data.qvel.resize(data.size);
    data.qvel_fil.resize(data.size);
    file.read(reinterpret_cast<char *>(data.qpos.data()),
              data.qpos.size() * sizeof(std::array<double, 19>));
    file.read(reinterpret_cast<char *>(data.qpos_fil.data()),
              data.qpos_fil.size() * sizeof(std::array<double, 6>));
    file.read(reinterpret_cast<char *>(data.qvel.data()),
              data.qvel.size() * sizeof(std::array<double, 18>));
    file.read(reinterpret_cast<char *>(data.qvel_fil.data()),
              data.qvel_fil.size() * sizeof(std::array<double, 6>));

    // Read MPC trajectories.
    for (size_t i = 0; i < data.mpc_iteration; i++) {
      std::vector<std::array<double, 37>> mpc_data(data.horizon);
      file.read(reinterpret_cast<char *>(mpc_data.data()),
                mpc_data.size() * sizeof(std::array<double, 37>));
      data.mpc_traj.push_back(mpc_data);
    }
    file.close();
  }

  return data;
}

void Logger::writeToCsvFile(const std::string &fileName) {
  std::ofstream outputFile;
  outputFile.open(fileName, std::ios::out | std::ios::trunc);
  if (!outputFile.is_open()) {
    std::cerr << "Error: Unable to open file '" << fileName << "' for logging."
              << std::endl;
  } else {
    // Write header to the CSV file
    for (auto it = foot_names_.begin(); it != foot_names_.end(); ++it) {
      outputFile << *it;
      outputFile << (std::next(it) != foot_names_.end() ? "," : "\n");
    }
  }
  int size = data_.foot_status[foot_names_[0]].size();
  for (int k = 0; k < size; k++) {
    for (auto it = foot_names_.begin(); it != foot_names_.end(); ++it) {
      outputFile << data_.foot_status.at(*it)[k];
      outputFile << (std::next(it) != foot_names_.end() ? "," : "\n");
    }
  }
  outputFile.flush(); // Flush the stream after each step
  outputFile.close();
}
