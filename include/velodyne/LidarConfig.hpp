#pragma once
#include <common.hpp>
#include <point_cloud/common.h>

// 对应于实验室采用的三代不同类型的激光雷达
enum LidarType
{
    VLP16, HDL32, VLP32
};

//=========================================//
// 线号对应表
//=========================================//
std::vector<int> vlp16_scanID = {
	0, 8, 
	1, 9,
	2, 10,
	3, 11,
	4, 12,
	5, 13,
	6, 14,
	7, 15
};

std::vector<int> vlp32_map = {
	0,  3, 4,  7,
	8,  11, 12,  16,
	15,  19, 20,  24,
	23,  27, 28,  2,
	31,  1, 6,  10,
	5, 9, 14, 18,
	13, 17, 22, 21,
	26, 25, 30, 29
};

std::vector<int> vlp32_scanID(32, 0);

std::vector<int> hdl32_scanID = {
	0,  16,
	1,  17,
	2,  18,
	3,  19,
	4,  20,
	5,  21,
	6,  22,
	7,  23,
	8,  24,
	9,  25,
	10, 26,
	11, 27,
	12, 28,
	13, 29,
	14, 30,
	15, 31
};

class LidarConfig
{
public:
    LidarConfig(int nScan = 32, float upperBound = 15.0, float lowerBound = -25.0):
        _nScanRings(nScan),
        _upperBound(upperBound), 
        _lowerBound(lowerBound) {}
    
    int getScanID(pcl::PointXYZI &p)
    {
        pcl::PointXYZI point;
        point.x = p.y;
        point.y = p.z;
        point.z = p.x;
        
        float _factor = (_nScanRings - 1) / (_upperBound - _lowerBound);
        float angle = std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z));
        int scanID = int(((angle * 180 / M_PI) - _lowerBound) * _factor + 0.5); // 加0.5表示四舍五入
        if(scanID > _nScanRings) scanID = _nScanRings;
        else if(scanID < 0) scanID = 0;
        return scanID;
    }

    int getScanID(PointType &p)
    {
        PointType point;
        point.x = p.y;
        point.y = p.z;
        point.z = p.x;
        
        float _factor = (_nScanRings - 1) / (_upperBound - _lowerBound);
        float angle = std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z));
        int scanID = int(((angle * 180 / M_PI) - _lowerBound) * _factor + 0.5); // 加0.5表示四舍五入
        if(scanID > _nScanRings) scanID = _nScanRings;
        else if(scanID < 0) scanID = 0;
        return scanID;
    }
private:
    int _nScanRings;
    float _upperBound;
    float _lowerBound;
};