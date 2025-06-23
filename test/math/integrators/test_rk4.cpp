#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include "math/integrators/rk4.h"
#include "test_helpers.h"

using namespace Eigen;
using namespace Cesium::Math;

//////////////////////////////////////////////////
//                   Constant                   //
//////////////////////////////////////////////////

VectorXd constant_deriv(double t, const VectorXd& x) {
    return VectorXd::Zero(x.size());
}

TEST(RK4Test, Constant) {
    VectorXd x0(5);
    x0 << 1.0, -2.0, 3.5, 100.3, 12.3;

    double dt = 1.0;

    VectorXd x1 = rk4_step(dt, 0, x0, constant_deriv);

    EXPECT_NEAR(x1(0), x0(0), 1e-9);
    EXPECT_NEAR(x1(1), x0(1), 1e-9);
    EXPECT_NEAR(x1(2), x0(2), 1e-9);
    EXPECT_NEAR(x1(3), x0(3), 1e-9);
    EXPECT_NEAR(x1(4), x0(4), 1e-9);
}

//////////////////////////////////////////////////
//                Constant Slope                //
//////////////////////////////////////////////////

VectorXd slope_2(double t, const VectorXd& x) {
    VectorXd out(1);
    out << 2.0;
    return out;
}

TEST(RK4Test, ConstSlope) {
    VectorXd x0(1);
    x0 << 0.0;

    double dt = 2;

    VectorXd x1 = rk4_step(dt, 0, x0, slope_2);

    // Since dx/dt = 2, exact solution: x = 0.0 + 2*2 = 4.0
    SCOPED_TRACE("ConstantDerivative");
    EXPECT_NEAR(x1(0), 4.0, 1e-10);
}

//////////////////////////////////////////////////
//             Exponential Decay                //
//////////////////////////////////////////////////

VectorXd exponential_decay_deriv(double t, const VectorXd& x) {
    return -x;
}

TEST(RK4Test, ExponentialSingleStep) {
    VectorXd x0(1);
    x0 << 4.0;

    double dt = 0.2;

    VectorXd x1 = rk4_step(dt, 0, x0, exponential_decay_deriv);

    // SCOPED_TRACE("Exponential");
    EXPECT_NEAR(x1(0), x0(0) * pow(M_E, -dt), 1e-2);

}

TEST(RK4Test, ExponentialMultiStep) {
    VectorXd x(1);
    x << 4.0;

    double dt = 0.1;
    int tmax = 10;  // Simulates from t = 0 to t = 1.0
    double t = 0.0;

    while (t < tmax) {
        x = rk4_step(dt, 0, x, exponential_decay_deriv);
        t += dt;
    }

    double expected = 4.0 * std::exp(-tmax);
    EXPECT_NEAR(x(0), expected, 1e-4);
}

//////////////////////////////////////////////////
//                     Sine!                    //
//////////////////////////////////////////////////

VectorXd sine_deriv(double t, const VectorXd& x) {
    VectorXd rslt(1);
    rslt << cos(t);
    return rslt;
}

TEST(RK4Test, SineSingleStep) {
    VectorXd x0(1);
    x0 << 4.0;

    double dt = 0.1;

    VectorXd x1 = rk4_step(dt, 0, x0, sine_deriv);

    EXPECT_NEAR(x1(0), x0(0) + sin(dt), 1e-3);
}

TEST(RK4Test, SineMultiStep_1_second) {
    VectorXd x(1);
    x << 4.0;

    double dt = 0.01;
    int tmax = 1;  // Simulates from t = 0 to t = 1.0
    double t = 0.0;

    while (t < tmax) {
        x = rk4_step(dt, t, x, sine_deriv);
        t += dt;
    }

    double expected = 4 + sin(tmax);
    EXPECT_NEAR(x(0), expected, 1e-3);
}

TEST(RK4Test, SineMultiStep_15_second) {
    VectorXd x(1);
    x << 4.0;

    double dt = 0.001;
    int tmax = 15;  // Simulates from t = 0 to t = 1.0
    double t = 0.0;

    while (t < tmax) {
        x = rk4_step(dt, t, x, sine_deriv);
        t += dt;
    }

    double expected = 4 + sin(tmax);
    EXPECT_NEAR(x(0), expected, 1e-3);
}

//////////////////////////////////////////////////
//                  Logistic                    //
//////////////////////////////////////////////////


VectorXd logistic_deriv_3(double t, const VectorXd& x) {
    double r = 1.0;
    double K = 10.0;
    VectorXd dxdt(3);
    dxdt << r * x.array() * (1.0 - x.array() / K);

    return dxdt;
}

TEST(RK4Test, LogisticGrowth) {
    VectorXd x(3);
    x << 1.0, 2.0, 5.0; // start small

    double dt = 0.001;
    double tmax = 15.0;
    double t = 0.0;

    while (t < tmax) {
        x = rk4_step(dt, t, x, logistic_deriv_3);
        t += dt;
    }

    // Should be (10, 10, 10) eventually
    EXPECT_NEAR(x(0), 10.0, 1e-4);
    EXPECT_NEAR(x(1), 10.0, 1e-4);
    EXPECT_NEAR(x(2), 10.0, 1e-4);
}
