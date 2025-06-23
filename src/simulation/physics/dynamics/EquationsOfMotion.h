#ifndef EQUATIONSOFMOTION_H
#define EQUATIONSOFMOTION_H

#include <Eigen/Dense>


namespace Cesium::Sim::Physics {
    
//////////////////////////////////////////////////
//                    State                     //
//////////////////////////////////////////////////

struct State {
    Eigen::Vector3d r{};
    Eigen::Vector3d v{};
    Eigen::Quaterniond q{};
    Eigen::Vector3d w{};

    Eigen::VectorXd to_vec() const {
        Eigen::VectorXd rslt(13);
        rslt << r, v, q.w(), q.x(), q.y(), q.z(), w;
        return rslt;
    }
};

//////////////////////////////////////////////////
//               StateDerivative                //
//////////////////////////////////////////////////

struct StateDerivative {
    Eigen::Vector3d drdt{};
    Eigen::Vector3d dvdt{};
    Eigen::Quaterniond dqdt{};
    Eigen::Vector3d dwdt{};

    static StateDerivative from_vec(Eigen::VectorXd dot) {
        StateDerivative rslt;
        rslt.drdt = dot.segment<3>(0);
        rslt.dvdt = dot.segment<3>(3);
        rslt.dqdt = Eigen::Quaterniond(dot(6), dot(7), dot(8), dot(9));
        rslt.dwdt = dot.segment<3>(10);

        return rslt;
    }
};

//////////////////////////////////////////////////
//                  RigidBody                   //
//////////////////////////////////////////////////

struct RigidBody {
    State state;
    double mass = 0.0;
    Eigen::Matrix3d InertiaTensor = Eigen::Matrix3d::Zero(3,3);

    Eigen::Vector3d force{};
    Eigen::Vector3d torque{};
    
    double epoch_s;

    StateDerivative derivative(Eigen::Vector3d force, Eigen::Vector3d torque);

    Eigen::VectorXd compute_derivative(double t, const Eigen::VectorXd& state) const;
};

} // namespace Cesium::Sim::Physics

#endif