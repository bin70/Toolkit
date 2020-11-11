#pragma once
#include <math.h>
#include <eigen3/Eigen/Dense>

/********************************
 * 对位姿进行转换的函数库
 * 
 ********************************/
class TransformTool
{
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    typedef Eigen::Matrix<double, 7, 1> Vector7d;
    typedef Eigen::Vector3d Vector3d;
    typedef Eigen::Matrix4d Matrix4d;
    typedef Eigen::Matrix3d Matrix3d;
    typedef Eigen::Quaterniond Qua;

public:
    // 核心函数
    // LOAM中的欧拉角表示->位姿矩阵表示
    Eigen::Matrix4d euler2matrix(const Vector6d &pose)
    {
        Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
        Eigen::Matrix3d Rx = Eigen::AngleAxisd(pose[0], Eigen::Vector3d(1, 0, 0)).toRotationMatrix();
        Eigen::Matrix3d Ry = Eigen::AngleAxisd(pose[1], Eigen::Vector3d(0, 1, 0)).toRotationMatrix();
        Eigen::Matrix3d Rz = Eigen::AngleAxisd(pose[2], Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
        Eigen::Matrix3d R = Ry * Rx * Rz; //位姿转成全局矩阵的关键
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
            m(i, j) = R(i, j); //旋转部分
        for (int i = 0; i < 3; ++i)
            m(i, 3) = pose[3 + i]; //平移部分
        return m;
    }

    Vector7d euler2tq(const Vector6d &pose)
    {
        Matrix4d m = euler2matrix(pose);
        return matrix2tq(m);
    }

    Vector7d matrix2tq(Matrix4d &m)
    {
        Matrix3d r = m.block(0,0,3,3);
        Vector3d t = m.block(0,3,3,1);
        Qua q(r);
        
        Vector7d tq;
        tq.block(0,0,3,1) = t;
        tq(3) = q.x();
        tq(4) = q.y();
        tq(5) = q.z();
        tq(6) = q.w();
        return tq;
    }

    Vector6d matrix2euler(Matrix4d &m)
    {
        Vector6d e = m2e(m);
        Matrix4d m2 = euler2matrix(e);
        return m2e(m2);
    }

    Matrix4d tq2matrix(const Vector7d &tq)
    {
        Matrix4d m = Matrix4d::Identity();

        Qua q(tq(6), tq(3), tq(4), tq(5));
        Matrix3d r = q.toRotationMatrix();
        m.block(0,0,3,3) = r;

        Vector3d t(tq(0), tq(1), tq(2));
        m.block(0,3,3,1) = t;

        return m;
    }

    Vector6d tq2euler(Vector7d &tq)
    {
        
    }

    Vector6d m2e(Eigen::Matrix4d m)
    {
        Vector6d e;
        // rotation
        e[0] = atan2(m(0, 2), m(2, 2));
        e[1] = asin(-m(1, 2));
        e[2] = atan2(m(1, 0), m(1, 1));

        // translation
        for (int i = 0; i < 3; ++i)
            e[3 + i] = m(i, 3);
        return e;
    }
};
