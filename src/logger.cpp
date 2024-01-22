#include "logger.h"

Logger::Logger() {
  int max_size = 5000;
  data_.size = 0;
  data_.qpos.reserve(5000);
  data_.qvel.reserve(5000);
}

Logger::~Logger() {}

void Logger::Initialize(const std::vector<std::string> &foot_names,
                        const double dt_mpc, const int nsteps_mpc) {
  foot_names_ = foot_names;
  data_.dt_mpc = dt_mpc;
  data_.nsteps_mpc = nsteps_mpc;

  // Reserve memory for the foot contact status in the data.
  for (auto &name : foot_names) {
    std::vector<int> tmp;
    std::vector<int> tmp_touch;
    std::vector<std::array<double, 3>> tmp_pos;
    std::vector<std::array<double, 3>> tmp_vel;
    std::vector<std::array<double, 3>> tmp_forces;
    tmp.reserve(5000);
    tmp_pos.reserve(5000);
    tmp_touch.reserve(5000);
    tmp_vel.reserve(5000);
    tmp_forces.reserve(5000);
    data_.foot_status[name] = tmp;
    data_.foot_status_touch[name] = tmp;
    data_.foot_position[name] = tmp_pos;
    data_.foot_velocity[name] = tmp_vel;
    data_.contact_forces[name] = tmp_forces;
  }
}

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
    const double *touch = mjpc::SensorByName(model, data, name + "_touch");
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
}

void Logger::logState(const mjModel *model, mjData *data) {
  std::array<double, 19> qpos;
  std::copy(data->qpos, data->qpos + 19, qpos.begin());
  data_.qpos.emplace_back(qpos);

  std::array<double, 18> qvel;
  std::copy(data->qvel, data->qvel + 18, qvel.begin());
  data_.qvel.emplace_back(qvel);
}

void Logger::saveData(const std::string &fileName) {
  std::ofstream file(fileName,
                     std::ios::out | std::ios::binary | std::ios::trunc);
  if (file.is_open()) {
    file.write(reinterpret_cast<const char *>(&data_.size), sizeof(int));

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

    // Load qpos and qvel
    std::cout << "qpos size : " << data_.qpos.size() << std::endl;
    file.write(reinterpret_cast<const char *>(data_.qpos.data()),
               data_.qpos.size() * sizeof(std::array<double, 19>));
    file.write(reinterpret_cast<const char *>(data_.qvel.data()),
               data_.qvel.size() * sizeof(std::array<double, 18>));

    file.close();
  } else {
    std::cout << "Error while opening the file" << std::endl;
  }
}

Data Logger::loadData(const std::string &fileName) {
  Data data;

  std::ifstream file(fileName, std::ios::binary);
  if (file.is_open()) {
    // file.read(reinterpret_cast<char*>(data.q_mes.data()), data.q_mes.size() *
    // sizeof(double)); file.read(reinterpret_cast<char*>(data.q_des.data()),
    // data.q_des.size() * sizeof(double)); int size; // meta-data.
    // file.read(reinterpret_cast<char*>(&data.size), sizeof(int));
    file.read(reinterpret_cast<char *>(&data.size), sizeof(int));

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

    data.qpos.resize(data.size);
    data.qvel.resize(data.size);
    file.read(reinterpret_cast<char *>(data.qpos.data()),
              data.qpos.size() * sizeof(std::array<double, 19>));
    file.read(reinterpret_cast<char *>(data.qvel.data()),
              data.qvel.size() * sizeof(std::array<double, 18>));

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
