#pragma once
//------------pcl headers------------------------
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

//typedef pcl::PointXYZI PointType;
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloud;

inline float getDistance(PointType p1, PointType p2)
{
    return sqrt(pow(p1.x-p2.x, 2)+pow(p1.y-p2.y, 2)+pow(p1.z-p2.z, 2));
}