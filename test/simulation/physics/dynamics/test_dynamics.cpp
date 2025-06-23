#include <gtest/gtest.h>
#include <Eigen/Dense>

#include "simulation/physics/dynamics/EquationsOfMotion.h"
#include "test_helpers.h"

#include <iostream> // DELETE BEFORE COMMITTING
using namespace std; // DELETE BEFORE COMMITTING

using namespace Eigen;
using namespace Cesium::Sim::Physics;

TEST(RigidBodyTest, ForceNoTorque) {
    RigidBody rb;
    rb.state.q = Quaterniond(1, 0, 0, 0);
    rb.state.w = Vector3d(0, 0, M_PI);
    rb.mass = 1.0;
    rb.InertiaTensor = Matrix3d::Identity();

    Vector3d force(1, 0, 0);
    Vector3d torque = Vector3d::Zero();

    StateDerivative dot = rb.derivative(force, torque);

    SCOPED_TRACE("drdt");
    test_vec_equal(dot.drdt, Vector3d::Zero());                     // No velocity
    SCOPED_TRACE("dvdt");
    test_vec_equal(dot.dvdt, Vector3d(1,0,0));                      // Only X acceleration
    SCOPED_TRACE("dqdt");
    test_quat_equal(dot.dqdt, Quaterniond(0, 0, 0, 0.5 * M_PI));    // Z axis rotation via omega
    SCOPED_TRACE("dwdt");
    test_vec_equal(dot.dwdt, Vector3d(0,0,0));                      // No angular velocity
}

TEST(RigidBodyTest, TorqueNoForce) {
    RigidBody rb;
    rb.state.q = Quaterniond(1, 0, 0, 0);
    rb.state.w = Vector3d(0, 0, 0);
    rb.mass = 1.0;
    rb.InertiaTensor = Matrix3d::Identity();

    Vector3d force = Vector3d::Zero();
    Vector3d torque(1, 0, 0);

    StateDerivative dot = rb.derivative(force, torque);

    SCOPED_TRACE("drdt");
    test_vec_equal(dot.drdt, Vector3d::Zero());         // No velocity
    SCOPED_TRACE("dvdt");
    test_vec_equal(dot.dvdt, Vector3d::Zero());         // No acceleration
    SCOPED_TRACE("dqdt");
    test_quat_equal(dot.dqdt, Quaterniond(0,0,0,0));    // No change in q via omega
    SCOPED_TRACE("dwdt");
    test_vec_equal(dot.dwdt, Vector3d(1,0,0));          // Only X angular velocity
}

TEST(RigidBodyTest, HardCodeRandom) {
    RigidBody rb;
    rb.state.v = Vector3d(4,4,4);
    rb.state.q = Quaterniond(AngleAxisd(M_PI / 2.0, Vector3d::UnitY())); // 90deg around Y --> (1/sqrt(2), 0, 1/sqrt(2), 0) 
    rb.state.w = Vector3d(1, 2, 3); // Random angular acceleration
    rb.mass = 10.0;
    rb.InertiaTensor = Vector3d(3, 2, 1).asDiagonal();

    Vector3d force = Vector3d(10, 20, 30);
    Vector3d torque(1, 0, 0);

    StateDerivative dot = rb.derivative(force, torque);

    // Do the hand calcs because it is fun!

    SCOPED_TRACE("drdt");
    test_vec_equal(dot.drdt, Vector3d(4,4,4));                                          // :)
    SCOPED_TRACE("dvdt");
    test_vec_equal(dot.dvdt, Vector3d(1, 2, 3));                                        // (Schaub 2.15)
    SCOPED_TRACE("dqdt");
    test_quat_equal(dot.dqdt, Quaterniond(-M_SQRT1_2, M_SQRT2, M_SQRT1_2, M_SQRT1_2));  // (Schaub 3.109)
    SCOPED_TRACE("dwdt");
    test_vec_equal(dot.dwdt, Vector3d(7.0 / 3.0, -3, 2));                               // (Schaub 4.34)
}