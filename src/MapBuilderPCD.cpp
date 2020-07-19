#include <io/PCDOperator.hpp>
#include <io/TrajIO.hpp>
#include <io/FileOperator.hpp>
#include <build_map/MapManager.hpp>
#include <argparse.hpp>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <visualization/ShowCloud.hpp>

using namespace std;
using namespace Eigen;
using namespace vis_utils;
using namespace pcl::visualization;

FileOperator fop;
bool show_cloud = false;
float resolution = 0.03;
PCLVisualizer *viewer;

// vlp32的配置
int _nScanRings = 32;
float _upperBound = 15.0;
float _lowerBound = -25.0;

int getLOAMScanID(PointType &p)
{
    PointType point;
    point.x = p.y;
    point.y = p.z;
    point.z = p.x;
    
    float _factor = (_nScanRings - 1) / (_upperBound - _lowerBound);
    float angle = std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z));
    return int(((angle * 180 / M_PI) - _lowerBound) * _factor + 0.5);;
}

int main(int argc, const char** argv)
{
    ArgumentParser parser;
    parser.addArgument("-i", "--input_dir", true);
    parser.addArgument("-t", "--traj_type", true);
    parser.addArgument("-b", "--begin_id");
    parser.addArgument("-e", "--end_id");
    parser.addArgument("-s", "--show_cloud");
    parser.addArgument("-r", "--resolution");
    parser.parse(argc, argv);

    string input_dir = parser.get("input_dir");    
    ;

    if(parser.count("show_cloud"))
    {
        show_cloud = parser.get<bool>("show_cloud");
        viewer = new PCLVisualizer("show map");
    }

    if(parser.count("resolution"))
        resolution = parser.get<float>("resolution");

    PCDReader reader(input_dir+"/PCD");
    reader.setBinary(true);
    
    TrajIO traj(input_dir+"/traj_with_timestamp.txt", 
                (TrajType)parser.get<int>("traj_type"));
    // 不指定建图范围的话，直接用轨迹中的ID建图
    int begin_id = traj.getStartID();
    int end_id = traj.getEndID();
    if(parser.count("begin_id"))
        begin_id =parser.get<int>("begin_id");
    if(parser.count("end_id"))
        end_id = parser.get<int>("end_id");
    traj.checkID(begin_id, end_id);

    PointCloud::Ptr cloud(new PointCloud);
    int frame_id = begin_id;

    MapManager map(resolution);
    map.update();

    consoleProgress(0);
    while(reader.readPointCloud(cloud, frame_id))
    {
        // TODO: 增加重建LOAM的特征地图并保存的功能
        
        PointCloud::Ptr cloud_filtered(new PointCloud);
        
        #if RANGE_LIMIT
        // 范围滤波器
        pcl::ConditionalRemoval<PointType> cond_removal;
        PointType minP, maxP;
        pcl::getMinMax3D<PointType>(*cloud, minP, maxP);
        pcl::ConditionAnd<PointType>::Ptr cond(new pcl::ConditionAnd<PointType>());
        // x,y 只取15米距离
        cond->addComparison(pcl::FieldComparison<PointType>::ConstPtr(new pcl::FieldComparison<PointType>("x", pcl::ComparisonOps::GT, (minP.x+maxP.x)/2-15.0 )));
        cond->addComparison(pcl::FieldComparison<PointType>::ConstPtr(new pcl::FieldComparison<PointType>("x", pcl::ComparisonOps::LT, (minP.x+maxP.x)/2+15.0 )));
        cond->addComparison(pcl::FieldComparison<PointType>::ConstPtr(new pcl::FieldComparison<PointType>("y", pcl::ComparisonOps::GT, (minP.y+maxP.y)/2-15.0 )));
        cond->addComparison(pcl::FieldComparison<PointType>::ConstPtr(new pcl::FieldComparison<PointType>("y", pcl::ComparisonOps::LT, (minP.y+maxP.y)/2+15.0 )));
        // z轴只取(-1.0, 5.0)米的范围
        cond->addComparison(pcl::FieldComparison<PointType>::ConstPtr(new pcl::FieldComparison<PointType>("z", pcl::ComparisonOps::GT, -2.0 ))); 
        cond->addComparison(pcl::FieldComparison<PointType>::ConstPtr(new pcl::FieldComparison<PointType>("z", pcl::ComparisonOps::LT, 6.0 )));
        cond_removal.setCondition(cond);
        cond_removal.setInputCloud(cloud);
        cond_removal.filter(*cloud_filtered);
        #else
            pcl::copyPointCloud(*cloud, *cloud_filtered);
        #endif

        // 离群点滤波器
        pcl::StatisticalOutlierRemoval<PointType> stat_removal;
        stat_removal.setMeanK(20);
        stat_removal.setStddevMulThresh(1.5);
        stat_removal.setInputCloud(cloud_filtered);
        cloud->clear();
        stat_removal.filter(*cloud);

        for(int i=0; i<cloud->points.size(); ++i)
        {
            PointType &p =  cloud->points[i];
            p.curvature = getLOAMScanID(p);
        }

        Matrix4d m = traj.getPoseMatrix(frame_id);
        pcl::transformPointCloud(*cloud, *cloud, m);
                
        map.addFrame(cloud);
        if(show_cloud)
            ShowCloud(map.getMapPtr(), viewer, "curvature", 2);

        frame_id += traj.getFrameGap();
        if(frame_id>end_id) break;
        consoleProgress(frame_id, begin_id, end_id);
    }

    // 跟loam一样的存储方式，为了方便对比显示
    // for(int i=0; i<map.getMapPtr()->points.size(); ++i)
    // {
    //     auto &p = map.getMapPtr()->points[i];
    //     float temp = p.intensity;
    //     p.intensity = p.data_n[2];
    //     p.data_n[2] = temp;
    // }

    string out_path = input_dir+"/map_"+to_string(begin_id)+"_"+to_string(end_id)+".pcd";
    pcl::io::savePCDFileBinaryCompressed(out_path, *map.getMapPtr());
    cout << "压缩的PCD点云地图保存到: " << out_path << endl; 
    return 0;
}



