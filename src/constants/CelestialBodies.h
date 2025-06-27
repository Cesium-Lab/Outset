#ifndef GRAVITY_H
#define GRAVITY_H

#include <Eigen/Dense>

namespace Cesium::Constants {
    
    /*
    Sources
    - mu
        - NASA Astrodynamic Parameters
            - https://ssd.jpl.nasa.gov/astro_par.html
    - Earth R_eq
        - NASA Earth Fact Sheet
            - https://nssdc.gsfc.nasa.gov/planetary/factsheet/earthfact.html
    - Earth J2, J3
        - Wikipedia (apparently EGM96)
            - https://en.wikipedia.org/wiki/Geopotential_spherical_harmonic_model#Available_models
    */


constexpr double G_m2_kgs3 = 6.67430e-11;

namespace Earth {
    constexpr double MU_m2_s2 = 3.98600435507e11;
    constexpr double R_EQ_m = 6378137;
    constexpr double J2 = -0.108262982131e-2; 
    constexpr double J3 = 0.2532435346e-5;
};

namespace Sun {
    constexpr double MU_m2_s2 = 1.32712440041279419e20;
};

}

#endif