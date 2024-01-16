#include "logger.h"


Logger::Logger(const std::string& fileName) {
    outputFile.open(fileName);
    if (!outputFile.is_open()) {
        std::cerr << "Error: Unable to open file '" << fileName << "' for logging." << std::endl;
    }
    simulationResults.reserve(maxSimulationSteps);
}

Logger::~Logger() {
    if (outputFile.is_open()) {
        outputFile.close();
    }
}

void Logger::logFeetStatus(const std::vector<const char*>& footNames, const std::vector<bool>& feetInContact) {
    if (!outputFile.is_open()) {
        std::cerr << "Error: Log file not open." << std::endl;
        return;
    }

    // Log current simulation step
    outputFile << "Simulation Step" << std::endl;

    // Log feet status
    for (size_t i = 0; i < footNames.size(); ++i) {
        const char* footName = footNames[i];
        bool inContact = feetInContact[i];

        outputFile << "Foot '" << footName << "': " << (inContact ? "In contact" : "Not in contact") << std::endl;
    }

    // Add a separator for better readability
    outputFile << "------------------------" << std::endl;
}