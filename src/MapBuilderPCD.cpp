#include <utils/argparse.hpp>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <io/PCDOperator.hpp>
#include <io/TrajIO.hpp>
#include <build_map/MapManager.hpp>

#include <visualization/ShowUtils.hpp>
#include <velodyne/LidarConfig.hpp>

using namespace std;
using namespace Eigen;
using namespace pcl::visualization;

FileOperator fop;
bool show_cloud = false;
float resolution = 0.03;

ShowUtils su("Map Builder", true);

// vlp32的配置
LidarConfig lidar_config(VLP32);

int main(int argc, const char** argv)
{
    ArgumentParser parser;
    parser.addArgument("-i", "--input_dir", true);
    parser.addArgument("-t", "--traj_path", true);
    parser.addArgument("-b", "--begin_id");
    parser.addArgument("-e", "--end_id");
    parser.addArgument("-s", "--show_cloud");
    parser.addArgument("-r", "--resolution");
    parser.parse(argc, argv);

    string input_dir = parser.get("input_dir");    
    string traj_path = parser.get("traj_path");

    if(parser.count("show_cloud"))
    {
        show_cloud = parser.get<bool>("show_cloud");
    }

    if(parser.count("resolution"))
        resolution = parser.get<float>("resolution");

    PCDReader reader(input_dir);
    TrajIO traj(traj_path);

    std::cout << "================ runing information ===============\n" << std::endl
              << "\tpcd directory = " << input_dir << std::endl
              << "\ttraj_path = " << traj_path << std::endl;
    pLine();

    // 不指定建图范围的话，直接用轨迹中的ID建图
    int begin_id = traj.getStartID();
    int end_id = traj.getEndID();

    if(parser.count("begin_id"))
        begin_id =parser.get<int>("begin_id");
    if(parser.count("end_id"))
        end_id = parser.get<int>("end_id");
    traj.checkID(begin_id, end_id);
    
    std::cout << "\tbegin_id = "  << begin_id << std::endl
              << "\tend_id = " << end_id << std::endl
              << "=====================================================" << std::endl;

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
            p.curvature = lidar_config.getScanID(p);
        }

        Matrix4d m = traj.getPoseMatrix(frame_id);
        pcl::transformPointCloud(*cloud, *cloud, m);
                
        map.addFrame(cloud);
        if(show_cloud)
        {
            su.ShowCloud(map.getMapPtr(), "map", "curvature");
            su.waitSpace();
        }
        frame_id += traj.getFrameGap();
        if(frame_id>end_id) break;
        consoleProgress(frame_id, begin_id, end_id);
    }

    string out_path = input_dir+"/map_"+to_string(begin_id)+"_"+to_string(end_id)+".pcd";
    pcl::io::savePCDFileBinaryCompressed(out_path, *map.getMapPtr());
    cout << "压缩的PCD点云地图保存到: " << out_path << endl; 
    return 0;
}



