#include <iostream>
#include <math/common.hpp>

using namespace Eigen;
using namespace std;

int main()
{
    Vector3d v1, v2, v3;
    v1 << 0.101217, -0.992407, -0.0698851;
    v2 << 0.994589, 0.103209, -0.0118995;
    v3 << -0.106501, 0.991406, 0.0759694;

    std::cout << get_angle(v1, v2) << std::endl;
    std::cout << get_angle(v1, v3) << std::endl;
    
    cout << get_angle2(v1, v2) << endl;
    cout << get_angle2(v1, v3) << endl;
    
    return 0;

}