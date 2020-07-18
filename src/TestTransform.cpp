#include <math/TransformTool.hpp>

using namespace Eigen;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 7, 1> Vector7d;

int main()
{
    Vector7d tq;
    tq << 17.0043, -11.0251, -4.16833, 0.725032, -0.0500148, -0.0446839, 0.685442;

    TransformTool tt;
       
}