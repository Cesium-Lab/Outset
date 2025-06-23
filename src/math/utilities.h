#ifndef UTILITIES_H
#define UTILITIES_H

#include <Eigen/Dense>

namespace Cesium::Math {

inline Eigen::Quaterniond hamilton_product(const Eigen::Quaterniond& q, const Eigen::Vector3d& w) {
    double wx = w(0);
    double wy = w(1);
    double wz = w(2);

    return Eigen::Quaterniond(
        -0.5 * (q.x() * wx + q.y() * wy + q.z() * wz),
        0.5 * (q.w() * wx + q.y() * wz - q.z() * wy),
        0.5 * (q.w() * wy + q.z() * wx - q.x() * wz),
        0.5 * (q.w() * wz + q.x() * wy - q.y() * wx)
    );
}

} // namespace Cesium::Math



#endif