#include <gtest/gtest.h>

#include "math/utilities.h"
#include "test_helpers.h"


using namespace Eigen;
using namespace Cesium::Math;

//////////////////////////////////////////////////
//               hamilton_product               //
//////////////////////////////////////////////////

TEST(QuaternionDerivativeTest, Zero) {
    Quaterniond q(1,0,0,0);
    Vector3d w(0,0,0);
    Quaterniond expected(0,0,0,0);

    Quaterniond result = hamilton_product(q, w);

    test_quat_equal(result, expected);
}

TEST(QuaternionDerivativeTest, PI_RadPerSec) {
    Eigen::Quaterniond q(1, 0, 0, 0);  // Identity quaternion
    q.normalize();

    Eigen::Vector3d omega(0, 0, M_PI);  // Rotating around Z at π rad/s

    Eigen::Quaterniond dq = hamilton_product(q, omega);

    Eigen::Quaterniond expected(0.0, 0.0, 0.0, 0.5*M_PI);

    test_quat_equal(dq, expected);
}

TEST(QuaternionDerivativeTest, SmallTimeStep) {
    Eigen::Quaterniond q(1, 0, 0, 0);

    Eigen::Vector3d omega(0, 0, M_PI); // π rad/s around Z
    
    Eigen::Quaterniond dq = hamilton_product(q, omega);
    Eigen::Quaterniond q_next = q;

    // Integrate
    double dt = 0.01;
    q_next.coeffs() += dq.coeffs() * dt;
    q_next.normalize();

    // Expected rotation is about 
    //      0.01 [s] * π [rad/s] ≈ 0.0314 [rad]
    //      around Z
    Eigen::AngleAxisd expected_aa(dt * M_PI, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond expected_q(expected_aa);

    test_quat_equal(q_next, expected_q, 1e-3);
}

TEST(QuaternionDerivativeTest, RandomHandCalc) {
    Eigen::Quaterniond q(AngleAxisd(M_PI / 2.0, Vector3d::UnitX())); // 90deg around Y --> (1/sqrt(2), 1/sqrt(2), 0, 0) 

    Eigen::Vector3d omega(10, -2, 17); // Just plain horrible numbers

    Eigen::Quaterniond dq = hamilton_product(q, omega);

    Eigen::Quaterniond expected_q(
        -5.0        * M_SQRT1_2, 
        5.0         * M_SQRT1_2, 
        -19.0/2.0   * M_SQRT1_2, 
        15.0/2.0    * M_SQRT1_2
    ); // (Schaub 3.109) !!!
 
    test_quat_equal(dq, expected_q, 1e-3);
}