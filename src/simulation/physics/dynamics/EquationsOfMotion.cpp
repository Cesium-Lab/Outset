#include "EquationsOfMotion.h"
#include "math/utilities.h"


using namespace Eigen;

namespace Cesium::Sim::Physics {

StateDerivative RigidBody::derivative(Vector3d force, Vector3d torque)
{

    this->force = force;
    this->torque = torque;

    StateDerivative state_dot = StateDerivative::from_vec(
        compute_derivative(0.0, this->state.to_vec())
    );
    
    return state_dot;
}

VectorXd RigidBody::compute_derivative(double t, const VectorXd &state) const
{
    Vector3d r = state.segment<3>(0);
    Vector3d v = state.segment<3>(3);
    Quaterniond q(state(6), state(7), state(8), state(9));
    Vector3d w = state.segment<3>(10);

    VectorXd dot(13);

    StateDerivative state_dot;

    // Position derivative is velocity 
    dot.segment<3>(0) = v;

    // Velocity derivative is acceleration (Schaub 2.15)
    dot.segment<3>(3) = this->force / this->mass;

    // Quaternion derivative is based on hamilton prodict (Schaub 3.109)
    Quaterniond product = Cesium::Math::hamilton_product(this->state.q, this->state.w);
    dot(6) = product.w();
    dot(7) = product.x();
    dot(8) = product.y();
    dot(9) = product.z();

    // Angular velocity derivative based on (Schaub 4.34)
    // dwdt = I.inv * (τ - w x (Iw) )
    dot.segment<3>(10) = this->InertiaTensor.inverse() * (this->torque - this->state.w.cross(this->InertiaTensor * this->state.w));

    return dot;
}

} // namespace Cesium::Sim::Physics
