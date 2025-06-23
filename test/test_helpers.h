#ifndef TEST_HELPERS_H
#define TEST_HELPERS_H

#include <gtest/gtest.h>
#include <Eigen/Dense>


inline void test_vec_equal(Eigen::VectorXd vec1, Eigen::VectorXd vec2, double tolerance = 1e-10) {
    
    EXPECT_NEAR(vec1.x(), vec2.x(), tolerance);
    EXPECT_NEAR(vec1.y(), vec2.y(), tolerance);
    EXPECT_NEAR(vec1.z(), vec2.z(), tolerance);
}

inline void test_quat_equal(Eigen::Quaterniond q1, Eigen::Quaterniond q2, double tolerance = 1e-10) {

    EXPECT_NEAR(q1.w(), q2.w(), tolerance);
    EXPECT_NEAR(q1.x(), q2.x(), tolerance);
    EXPECT_NEAR(q1.y(), q2.y(), tolerance);
    EXPECT_NEAR(q1.z(), q2.z(), tolerance);
}
#endif