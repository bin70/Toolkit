#include <iostream>
#include <math/TransformTool.hpp>
#include <math/common.hpp>
#include <boost/algorithm/string.hpp>

using namespace Eigen;
using namespace std;

typedef Eigen::Matrix<double, 7, 1> Vector7d;

int main(int argc, const char** argv)
{
    if(argc != 8) return -1;

    Vector7d tq;
    for(int i=1; i<argc; ++i)
        tq[i-1] = atof(argv[i]);

    TransformTool tt;
    cout << tt.tq2matrix(tq) << endl;
    return 0;
}