#pragma once
#include <common.hpp>

// 对应于实验室采用的三代不同类型的激光雷达
enum LidarType
{
    VLP16, HDL32, VLP32
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