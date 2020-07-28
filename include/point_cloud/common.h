#pragma once
//------------pcl headers------------------------
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

//typedef pcl::PointXYZI PointType;
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloud;