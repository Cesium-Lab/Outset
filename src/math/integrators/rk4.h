#ifndef RK4_H
#define RK4_H

#include <Eigen/Dense>

#include "math/utilities.h"

namespace Cesium::Math {

// Derivative function:
//     f(t, y) --> dy
Eigen::VectorXd rk4_step(double dt, double t, const Eigen::VectorXd& state, Eigen::VectorXd (*state_dot)(double, const Eigen::VectorXd&));

} // namespace Cesium::Math

#endif