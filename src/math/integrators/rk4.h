#ifndef RK4_H
#define RK4_H

#include <Eigen/Dense>

#include "math/rotation/utilities.h"

namespace Cesium::Math {

// Derivative function:
//     f(t, y) --> dy
Eigen::VectorXd rk4_step(double dt, double t, Eigen::VectorXd state, Eigen::VectorXd (*state_dot)(double, Eigen::VectorXd));

} // namespace Cesium::Math

#endif