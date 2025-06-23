#ifndef EQUATIONSOFMOTION_H
#define EQUATIONSOFMOTION_H

#include <Eigen/Dense>


namespace Cesium::Sim::Physics {
    
struct State {
    Eigen::Vector3d r{};
    Eigen::Vector3d v{};
    Eigen::Quaterniond q{};
    Eigen::Vector3d w{};
};

struct StateDerivative {
    Eigen::Vector3d drdt{};
    Eigen::Vector3d dvdt{};
    Eigen::Quaterniond dqdt{};
    Eigen::Vector3d dwdt{};
};

//////////////////////////////////////////////////
//                Motion Helpers                //
//////////////////////////////////////////////////

//////////////////////////////////////////////////
//                  RigidBody                   //
//////////////////////////////////////////////////

struct RigidBody {
    State state;
    double mass = 0.0;
    Eigen::Matrix3d InertiaTensor = Eigen::Matrix3d::Zero(3,3);

    double epoch_s;

    StateDerivative derivative(Eigen::Vector3d force, Eigen::Vector3d torque);
};




// struct RigidBody {
//     double mass;
//     Eigen::Matrix3d I{};
// };

// struct Disturbances {
//     Eigen::Vector3d force;
//     Eigen::Vector3d torque;
// };


// StateDerivative compute_state_derivative(RigidBody force, Disturbances disturbances);






} // namespace Cesium::Sim::Physics

#endif