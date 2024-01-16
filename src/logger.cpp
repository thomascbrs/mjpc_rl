#include "logger.h"

Logger::Logger(){
    int max_size = 5000;
    // data_.size = 0;
    data_.D.reserve(max_size);
    data_.P.reserve(max_size);
}
Logger::~Logger(){}

void Logger::Initialize(const std::vector<std::string>& foot_names){
    foot_names_ = foot_names;
    for (auto& name: foot_names){
        std::vector<int> tmp;
        tmp.reserve(5000);
        foot_status_[name] = tmp;
    }
}

void Logger::logFeetStatus(const std::unordered_map<std::string, int>& contact_status) {
    for (const auto& status : contact_status) {
        foot_status_[status.first].push_back(status.second);
    }
    data_.P.push_back(3.);
    data_.D.push_back(0.2);
    data_.size ++;
}

void Logger::saveData(const std::string& fileName) {
    std::ofstream file(fileName, std::ios::out | std::ios::binary | std::ios::trunc);
    if (file.is_open()) {
        file.write(reinterpret_cast<const char*>(&data_.size), sizeof(int));
        file.write(reinterpret_cast<const char*>(data_.P.data()), data_.P.size() * sizeof(double));
        file.write(reinterpret_cast<const char*>(data_.D.data()), data_.P.size() * sizeof(double));

        // Save foot_status_ data
        // for (const auto& entry : data_.foot_status_) {
        //     // Save the key (foot name)
        //     file.write(entry.first.c_str(), entry.first.size() * sizeof(char));
        //     file.write("\0", sizeof(char));  // Null-terminate the string

        //     // Save the vector of integers
        //     file.write(reinterpret_cast<const char*>(entry.second.data()), entry.second.size() * sizeof(int));
        // }

        file.close();
    }
    else{
        std::cout << "Error while opening the file" << std::endl;
    }

}


Data Logger::loadData(const std::string& fileName) {
    Data data;

    std::ifstream file(fileName, std::ios::binary);
    if (file.is_open()) {
        // file.read(reinterpret_cast<char*>(data.q_mes.data()), data.q_mes.size() * sizeof(double));
        // file.read(reinterpret_cast<char*>(data.q_des.data()), data.q_des.size() * sizeof(double));
        // int size; // meta-data.
        // file.read(reinterpret_cast<char*>(&data.size), sizeof(int));
        file.read(reinterpret_cast<char*>(&data.size), sizeof(int));

        // Resize vectors before reading data
        data.P.resize(data.size);
        data.D.resize(data.size);
        file.read(reinterpret_cast<char*>(data.P.data()), size_t(data.size) * sizeof(double));
        file.read(reinterpret_cast<char*>(data.D.data()), size_t(data.size) * sizeof(double));

        // Load foot_status_ data
        // while (!file.eof()) {
        //     std::string footName;
        //     char c;
        //     while ((file.get(c)) && (c != '\0')) {
        //         footName += c;
        //     }

        //     std::vector<int> footData(data.foot_status_[footName].size());
        //     file.read(reinterpret_cast<char*>(footData.data()), footData.size() * sizeof(int));

        //     data.foot_status_[footName] = footData;
        // }

        file.close();
    }

    return data;
}


void Logger::writeToCsvFile(const std::string& fileName){
    std::ofstream outputFile;
    outputFile.open(fileName, std::ios::out | std::ios::trunc);
    if (!outputFile.is_open()) {
        std::cerr << "Error: Unable to open file '" << fileName << "' for logging." << std::endl;
    }
    else {
        // Write header to the CSV file
        for (auto it = foot_names_.begin(); it != foot_names_.end(); ++it) {
            outputFile << *it;
            outputFile << (std::next(it) != foot_names_.end() ? "," : "\n");
        }
    }
    int size = foot_status_[foot_names_[0]].size();
    for (int k=0; k < size;k++){
        for (auto it = foot_names_.begin(); it != foot_names_.end(); ++it) {
            outputFile << foot_status_.at(*it)[k];
            outputFile << (std::next(it) != foot_names_.end() ? "," : "\n");
        }
    }
    outputFile.flush();  // Flush the stream after each step
    outputFile.close();
}
