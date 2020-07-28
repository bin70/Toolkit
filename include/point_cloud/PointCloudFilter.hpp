#pragma once
#include <common.hpp>
#include <point_cloud/common.h>
#include <point_cloud/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/conditional_removal.h>

// pcl实现的点云滤波器
// 设置某个滤波器的参数，则代表激活它

class PointCloudFilter{
public:
    PointCloudFilter(){}
    ~PointCloudFilter(){}

    void setRangeFilter(float x1 = -20.0, float x2 = 20.0, 
                        float y1 = -20.0, float y2 = 20.0,
                        float z1 = -2.0, float z2 = 7.0) { _x1 = x1; _x2 = x2; _y1 = y1; _y2 = y2; _z1 = z1; _z2 = z2;}

    void setGridFilter(float res = 0.03) { grid_filter = true; grid_res = res;  }
    void setRandFilter(float percent = 0.3) { random_filter = true; decimate_percentage = percent; }
    void setOutlierFilter(int knn = 10, float std = 1.5) { outlier_knn = knn; std = outlier_std; }
    void setRadiusFilter(int knn = 20, float r = 0.5) { radius_knn = knn; radius = r; }
    
    bool filter(PointCloud::Ptr cloud, PointCloud::Ptr cloud_filtered)
    {
        if(!cloud_filtered) return false;

        PointCloud::Ptr temp(new PointCloud);
        *temp = *cloud;
        
        if(range_filter)
        {
        // 范围滤波器
        pcl::ConditionalRemoval<PointType> cond_removal;
        PointType minP, maxP;
        pcl::getMinMax3D<PointType>(*cloud, minP, maxP);
        pcl::ConditionAnd<PointType>::Ptr cond(new pcl::ConditionAnd<PointType>());
        // x,y 只取20米距离
        cond->addComparison(pcl::FieldComparison<PointType>::ConstPtr(new pcl::FieldComparison<PointType>("x", pcl::ComparisonOps::GT, (minP.x+maxP.x)/2+_x1 )));
        cond->addComparison(pcl::FieldComparison<PointType>::ConstPtr(new pcl::FieldComparison<PointType>("x", pcl::ComparisonOps::LT, (minP.x+maxP.x)/2+_x2 )));
        cond->addComparison(pcl::FieldComparison<PointType>::ConstPtr(new pcl::FieldComparison<PointType>("y", pcl::ComparisonOps::GT, (minP.y+maxP.y)/2+_y1 )));
        cond->addComparison(pcl::FieldComparison<PointType>::ConstPtr(new pcl::FieldComparison<PointType>("y", pcl::ComparisonOps::LT, (minP.y+maxP.y)/2+_y2 )));
        // z轴只取(-2.0, 7.0)米的范围
        cond->addComparison(pcl::FieldComparison<PointType>::ConstPtr(new pcl::FieldComparison<PointType>("z", pcl::ComparisonOps::GT, _z1 ))); 
        cond->addComparison(pcl::FieldComparison<PointType>::ConstPtr(new pcl::FieldComparison<PointType>("z", pcl::ComparisonOps::LT, _z2 )));
        cond_removal.setCondition(cond);
        cond_removal.setInputCloud(cloud);
        cond_removal.filter(*cloud_filtered);
        }

        if(random_filter)
        {
            pcl::RandomSample<PointType> random_filter;
            int n_points = cloud->points.size() * (1.0 - decimate_percentage);
            random_filter.setSample(n_points);
            random_filter.setInputCloud(temp);
            random_filter.filter(*cloud_filtered);
            *temp = *cloud_filtered;
        }

        if(grid_filter)
        {
            pcl::VoxelGrid<PointType> grid;
            grid.setLeafSize(grid_res, grid_res, grid_res);
            grid.setInputCloud(temp);
            grid.filter(*cloud_filtered);
            *temp = *cloud_filtered;
        }

        if(outlier_filter)
        {
            pcl::StatisticalOutlierRemoval<PointType> sor;
            sor.setInputCloud(temp);
            sor.setMeanK(outlier_knn);
            sor.setStddevMulThresh(outlier_std);
            sor.filter(*cloud_filtered);
            *temp = *cloud_filtered;
        }

        if(radius_filter)
        {
            pcl::RadiusOutlierRemoval<PointType> rad;
            rad.setInputCloud(temp);
            rad.setRadiusSearch(radius);
            rad.setMinNeighborsInRadius(radius_knn);
            rad.filter(*cloud_filtered);
        }

        return true;

    }

private:
    bool range_filter = false;
    bool grid_filter = false;
    bool random_filter = false;
    bool outlier_filter = false;
    bool radius_filter = false;

    float grid_res; // 体素分辨率
    float decimate_percentage; // 0~1.0, 去除总点数的百分比
    float outlier_std; // 离群点滤除阈值
    int outlier_knn; // 计算离群点的近邻数
    float radius; // 半径滤波器的大小
    int radius_knn; // 半径滤波器的点数阈值
    float _x1, _x2, _y1, _y2, _z1, _z2;
};