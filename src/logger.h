#ifndef LOGGER_H
#define LOGGER_H

#include <vector>
#include <fstream>
#include <iostream>

struct FootStatus {
    const char* name;
    bool inContact;
};

class Logger {
public:
    Logger(const std::string& fileName);
    ~Logger();

    void logFeetStatus(const std::vector<const char*>& footNames, const std::vector<bool>& feetInContact);
    // std::vector<FootStatus> getSimulationResults() const;

private:
    std::ofstream outputFile;
    // Assuming a known maximum number of simulation steps (e.g., 1000)
    const size_t maxSimulationSteps = 5000;

    std::vector<FootStatus> simulationResults;
};

#endif // LOGGER_H
