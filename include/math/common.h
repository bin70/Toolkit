#pragma once
#include <cmath>

inline double r2d(double rad)
{
    //double Rad_to_deg = 45.0 / atan(1.0);
    
    //return rad * Rad_to_deg;
    return rad*180.0/M_PI;
}

inline double d2r(double degree)
{
    return degree*M_PI/180.0;
}
