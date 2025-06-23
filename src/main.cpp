#include <iostream>
#include <Eigen/Dense>

#include "simulation/physics/dynamics/EquationsOfMotion.h"

using Eigen::MatrixXd;

using namespace std;


int main() {


    cout << "Hi" << endl;

    MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);
    std::cout << m << std::endl;


    Cesium::Sim::Physics::State state;
    cout << state.q << endl;
    return 0;
}