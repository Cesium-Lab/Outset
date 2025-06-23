#include "EquationsOfMotion.h"
#include "math/utilities.h"


using namespace Eigen;

namespace Cesium::Sim::Physics {

StateDerivative RigidBody::derivative(Vector3d force, Vector3d torque)
{
    StateDerivative state_dot;

    // Position derivative is velocity 
    state_dot.drdt = this->state.v;

    // Velocity derivative is acceleration (Schaub 2.15)
    state_dot.dvdt = force / this->mass;

    // Quaternion derivative is based on hamiltonian prodict (Schaub 3.109)
    state_dot.dqdt = Cesium::Math::hamilton_product(this->state.q, this->state.w);

    // Angular velocity derivative based on (Schaub 4.34)
    // dwdt = I.inv * (τ - w x (Iw) )
    state_dot.dwdt = this->InertiaTensor.inverse() * (torque - this->state.w.cross(this->InertiaTensor * this->state.w));

    return state_dot;
}

} // namespace Cesium::Sim::Physics
