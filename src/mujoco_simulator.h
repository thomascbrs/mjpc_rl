#include "mujoco/mujoco.h"
#include <vector>
#include <GLFW/glfw3.h>
#include <mujoco/mjvisualize.h>

class MujocoSimulator {
public:
    MujocoSimulator(const char* modelFile);
    ~MujocoSimulator();

    void initialize();
    void runSimulation(int numSteps);
    // void mycontroller(const mjModel* m, mjData* d);
    std::vector<std::vector<double>> getLoggedJointPositions() const;

private:
    mjModel* model;
    mjData* data;
    mjModel* model_i;
    mjData* data_i;
    mjvCamera cam;                      // abstract camera
    mjvOption opt;                      // visualization options
    mjvScene scn;                       // abstract scene
    mjrContext con;                     // custom GPU context

    GLFWwindow* window;

    // Define PD controller parameters
    double kp_ = 5.;  // Proportional gain
    double kd_ = 0.3;  // Derivative gain
    std::vector<double> q0_;

    std::vector<std::vector<double>> jointPositionsLog;
};