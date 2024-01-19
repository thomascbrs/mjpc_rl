#ifndef LOGGER_H
#define LOGGER_H

#include <vector>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include "mujoco/mujoco.h"
#include "mjpc/utilities.h"

struct Data {
    int size = 0; // Usefull for loading.
    std::vector<std::array<double,19>> qpos; // qpos of the CoM/Trunk in world frame.
    std::vector<std::array<double,18>> qvel; // qvel of the CoM/Trunk in world frame.
    std::unordered_map<std::string, std::vector<int>> foot_status;
    std::unordered_map<std::string, std::vector<int>> foot_status_touch;
    std::unordered_map<std::string, std::vector<std::array<double, 3>>> foot_position;
    std::unordered_map<std::string, std::vector<std::array<double, 3>>> foot_velocity;
    std::unordered_map<std::string, std::vector<std::array<double, 3>>> contact_forces;
};

class Logger {
public:
    Logger();
    void Initialize(const std::vector<std::string>& foot_names);
    ~Logger();

    void logState(const mjModel* model, mjData* data);

    void logFeetStatus(const std::unordered_map<std::string, int>& contact_status);
    void logFeetForces(const std::unordered_map<std::string, std::array<double,3>>& contact_forces);
    void logFeetPosition(const mjModel *model, mjData *data);
    void logFeetVelocity(const mjModel *model, mjData *data);
    void logFeetTouch(const mjModel *model, mjData *data);

    void writeToCsvFile(const std::string &fileName);
    void saveData(const std::string& fileName);
    Data loadData(const std::string& fileName);

private:
    Data data_;
    std::vector<std::string> foot_names_;
};


#endif // LOGGER_H
