#pragma once
#include <cmath>

inline double r2d(double rad)
{
    return rad*180.0/M_PI;
}

inline double d2r(double degree)
{
    return degree*M_PI/180.0;
}
