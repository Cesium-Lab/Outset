#ifndef GRAVITY_H
#define GRAVITY_H

#include <Eigen/Dense>

namespace Cesium::Sim::Physics {

class Gravity {

    static Eigen::Vector3d compute_base_acceleration(double mu_km3_s2, const Eigen::Vector3d& r_km) {

        double r = r_km.norm();

        // Base Gravitational Acceleration (Schaub 2.15)
        Eigen::Vector3d base_a =  -mu_km3_s2 * r_km / (r * r * r); 

        return base_a;
    }

    static Eigen::Vector3d compute_J2_acceleration(double mu_km3_s2, double r_eq, double J2, const Eigen::Vector3d& r_km) {

        // J2 Gravitational Acceleration (Schaub 11.64)

        // Explicit optimizing operations
        double r = r_km.norm();
        double r2 = r * r;
        double r4 = r2 * r2;
        double r_eq2 = r_eq * r_eq;
        double z2 = r_km.z() * r_km.z();

        double term1 = - 1.5 * J2 * mu_km3_s2 * r_eq2 / r4; // [km/s2]
        
        Eigen::Vector3d term2; // [unitless]
        term2.x() = (1 - 5*z2/r2) * r_km.x() / r;
        term2.y() = (1 - 5*z2/r2) * r_km.y() / r;
        term2.z() = (3 - 5*z2/r2) * r_km.z() / r;

        Eigen::Vector3d J2_a = term1 * term2;

        return J2_a; 
    }

    // Because why not :P
    static Eigen::Vector3d compute_J3_acceleration(double mu_km3_s2, double r_eq, double J3, const Eigen::Vector3d& r_km) {

        // J3 Gravitational Acceleration (Schaub 11.65)

        // Explicit optimizing operations
        double r = r_km.norm();
        double r2 = r * r;
        double r3 = r2 * r;
        double r4 = r2 * r2;
        double r6 = r4 * r2;
        double r_eq2 = r_eq * r_eq;
        double r_eq3 = r_eq2 * r_eq;
        double z2 = r_km.z() * r_km.z();
        double z3 = z2 * r_km.z();
        double z4 = z2 * z2;

        double term1 = 0.5 * J3 * mu_km3_s2 * r_eq3 / r6; // [1/s2]
        
        Eigen::Vector3d term2; // [m]
        term2.x() = 5 * (7*z3/r3 - 3*r_km.z()/r) * r_km.x();
        term2.y() = 5 * (7*z3/r3 - 3*r_km.z()/r) * r_km.y();
        term2.z() = 3 * (1 - 10*z2/r2 + 35.0/3.0*z4/r4) * r_km.z();

        Eigen::Vector3d J2_a = term1 * term2;

        return J2_a; 
    }

};

}

#endif