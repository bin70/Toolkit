#pragma once
#include <cmath>
#include <Eigen/Dense>

inline double r2d(double rad)
{
    return rad*180.0/M_PI;
}

inline double d2r(double degree)
{
    return degree*M_PI/180.0;
}

inline double get_angle(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
    return r2d(atan2(v1.cross(v2).norm(), v1.transpose() * v2));
}

inline double get_angle2(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
{
    double x = (v1.transpose()*v2);
    return  r2d(acos( x / ( v1.norm()*v2.norm() ) ));
}

inline double get_distance(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2)
{
    return sqrt( pow(p1.x()-p2.x(), 2.0) 
               + pow(p1.y()-p2.y(), 2.0)
               + pow(p1.z()-p2.z(), 2.0));
}