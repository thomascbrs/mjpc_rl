#ifndef LOGGER_H
#define LOGGER_H

#include <vector>
#include <fstream>
#include <iostream>
#include <unordered_map>

struct Data {
    int size = 0; // Usefull for loading.
    std::vector<double> P;
    std::vector<double> D;
    std::unordered_map<std::string, std::vector<int>> foot_status;
};

class Logger {
public:
    Logger();
    void Initialize(const std::vector<std::string>& foot_names);
    ~Logger();

    void logFeetStatus(const std::unordered_map<std::string, int>& contact_status);
    void writeToCsvFile(const std::string &fileName);
    void saveData(const std::string& fileName);
    Data loadData(const std::string& fileName);

private:
    Data data_;
    std::vector<std::string> foot_names_;
};


#endif // LOGGER_H
