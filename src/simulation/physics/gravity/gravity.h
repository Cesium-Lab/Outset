#ifndef GRAVITY_H
#define GRAVITY_H

#include <Eigen/Dense>

namespace Cesium::Sim::Physics {

struct Gravity {

    

    static Eigen::Vector3d compute_base_acceleration(double mu_m3_s2, const Eigen::Vector3d& r_m) {

        double r = r_m.norm();

        // Base Gravitational Acceleration (Schaub 2.15)
        Eigen::Vector3d base_a =  -mu_m3_s2 * r_m / (r * r * r); 

        return base_a;
    }

    static Eigen::Vector3d compute_J2_acceleration(double mu_m3_s2, double r_eq_m, double J2, const Eigen::Vector3d& r_m) {

        // J2 Gravitational Acceleration (Schaub 11.64)

        // Explicit optimizing operations
        double r = r_m.norm();
        double r2 = r * r;
        double r4 = r2 * r2;
        double r_eq2 = r_eq_m * r_eq_m;
        double z2 = r_m.z() * r_m.z();

        double term1 = - 1.5 * J2 * mu_m3_s2 * r_eq2 / r4; // [m/s2]
        
        Eigen::Vector3d term2; // [unitless]
        term2.x() = (1 - 5*z2/r2) * r_m.x() / r;
        term2.y() = (1 - 5*z2/r2) * r_m.y() / r;
        term2.z() = (3 - 5*z2/r2) * r_m.z() / r;

        Eigen::Vector3d J2_a = term1 * term2;

        return J2_a; 
    }

    // Because why not :P
    static Eigen::Vector3d compute_J3_acceleration(double mu_m3_s2, double r_eq_m, double J3, const Eigen::Vector3d& r_m) {

        // J3 Gravitational Acceleration (Schaub 11.65)

        // Explicit optimizing operations
        double r = r_m.norm();
        double r2 = r * r;
        double r3 = r2 * r;
        double r4 = r2 * r2;
        double r6 = r4 * r2;
        double r_eq2 = r_eq_m * r_eq_m;
        double r_eq3 = r_eq2 * r_eq_m;
        double z2 = r_m.z() * r_m.z();
        double z3 = z2 * r_m.z();
        double z4 = z2 * z2;

        double term1 = 0.5 * J3 * mu_m3_s2 * r_eq3 / r6; // [1/s2]
        
        Eigen::Vector3d term2; // [m]
        term2.x() = 5 * (7*z3/r3 - 3*r_m.z()/r) * r_m.x();
        term2.y() = 5 * (7*z3/r3 - 3*r_m.z()/r) * r_m.y();
        term2.z() = 3 * (1 - 10*z2/r2 + 35.0/3.0*z4/r4) * r_m.z();

        Eigen::Vector3d J2_a = term1 * term2;

        return J2_a; 
    }

};

}

#endif