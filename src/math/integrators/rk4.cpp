#include "rk4.h"
#include "simulation/physics/dynamics/EquationsOfMotion.h"

using namespace Eigen;
using namespace Cesium::Sim::Physics;

namespace Cesium::Math {

VectorXd rk4_step(double dt, double t, VectorXd state, VectorXd (*state_dot)(double, VectorXd)) {
    VectorXd k1 = state_dot(t, state);
    VectorXd k2 = state_dot(t + 0.5*dt, state + 0.5*dt * k1);
    VectorXd k3 = state_dot(t + 0.5*dt, state + 0.5*dt * k2);
    VectorXd k4 = state_dot(t + dt, state + dt * k3);

    return state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4);
}

} // namespace Cesium::Math
