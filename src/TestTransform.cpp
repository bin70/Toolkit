#include <iostream>
#include <math/TransformTool.hpp>
#include <math/common.hpp>

using namespace Eigen;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;

int main()
{
    Vector7d tq;
    tq << 17.0043, -11.0251, -4.16833, 0.725032, -0.0500148, -0.0446839, 0.685442;

    Vector6d euler;
    euler << -0.081868, 0.101829, 19.3411, 20.0906, 13.5508, 3.14428;

    for(int i=0; i<3; ++i)
        std::cout << r2d(euler[i]) << ",";
    std::cout << std::endl;

    TransformTool tt;
    Matrix4d matrix = tt.euler2matrix(euler);

    std::cout << matrix << std::endl;

    Vector6d r_euler = tt.matrix2euler(matrix);
    
    for(int i=0; i<3; ++i)
        std::cout << r2d(r_euler[i]) << ",";
    std::cout << std::endl;
    
    std::cout << tt.euler2matrix(r_euler) << std::endl;
    return 0;
}