#include "logger.h"

Logger::Logger(){
    int max_size = 5000;
    data_.size = 0;
    data_.D.reserve(max_size);
    data_.P.reserve(max_size);
}

Logger::~Logger(){}

void Logger::Initialize(const std::vector<std::string>& foot_names){
    foot_names_ = foot_names;

    // Reserve memory for the foot contact status in the data.
    for (auto& name: foot_names){
        std::vector<int> tmp;
        std::vector<std::array<double, 3>> tmp_pos;
        tmp.reserve(5000);
        tmp_pos.reserve(5000);
        data_.foot_status[name] = tmp;
        data_.foot_position[name] = tmp_pos;
    }
}

void Logger::logFeetStatus(const std::unordered_map<std::string, int>& contact_status) {
    data_.P.push_back(3.);
    data_.D.push_back(0.2);
    for (const auto& status : contact_status) {
        data_.foot_status[status.first].push_back(status.second);
    }
    data_.size ++;
}

void Logger::logFeetPosition(const mjModel* model, mjData* data) {
  for (const auto& name : foot_names_) {
    const double* foot_pos = mjpc::SensorByName(model, data, name);
    std::array<double, 3> tmp_arr;

    // Copy the elements from foot_pos to temp_array
    std::copy(foot_pos, foot_pos + 3, tmp_arr.begin());
    data_.foot_position[name].push_back(tmp_arr);
  }
}

void Logger::saveData(const std::string& fileName) {
    std::ofstream file(fileName, std::ios::out | std::ios::binary | std::ios::trunc);
    if (file.is_open()) {
        file.write(reinterpret_cast<const char*>(&data_.size), sizeof(int));
        file.write(reinterpret_cast<const char*>(data_.P.data()), data_.P.size() * sizeof(double));
        file.write(reinterpret_cast<const char*>(data_.D.data()), data_.P.size() * sizeof(double));

        // Save foot_status_ data
        for (const auto& entry : data_.foot_status) {
            // Save the key (foot name)
            file.write(entry.first.c_str(), entry.first.size() * sizeof(char));
            file.write("\0", sizeof(char));  // Null-terminate the string

            // Save the vector of integers
            file.write(reinterpret_cast<const char*>(entry.second.data()), entry.second.size() * sizeof(int));
        }

        // Save foot_position data
        for (const auto& entry : data_.foot_position) {
            // Save the key (foot name)
            file.write(entry.first.c_str(), entry.first.size() * sizeof(char));
            file.write("\0", sizeof(char));  // Null-terminate the string

            // Save the vector of integers
            file.write(reinterpret_cast<const char*>(entry.second.data()), entry.second.size() * sizeof(std::array<double,3>));
        }

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

        // Load data specific to contact status
        for (size_t i = 0; i < 4; i++) {
            // Read the key
            std::string footName;
            char c;
            while ((file.get(c)) && (c != '\0')) {
                footName += c;
            }
            std::vector<int> footData(data.size);
            file.read(reinterpret_cast<char*>(footData.data()), footData.size() * sizeof(int));
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
            std::vector<std::array<double,3>> footData(data.size);
            file.read(reinterpret_cast<char*>(footData.data()), footData.size() * sizeof(std::array<double,3>));
            data.foot_position[footName] = footData;
        }

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
    int size = data_.foot_status[foot_names_[0]].size();
    for (int k=0; k < size;k++){
        for (auto it = foot_names_.begin(); it != foot_names_.end(); ++it) {
            outputFile << data_.foot_status.at(*it)[k];
            outputFile << (std::next(it) != foot_names_.end() ? "," : "\n");
        }
    }
    outputFile.flush();  // Flush the stream after each step
    outputFile.close();
}
