#include <utils/argparse.hpp>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <io/TrajIO.hpp>
#include <io/PCDOperator.hpp>
#include <io/LasOperator.hpp>
#include <build_map/MapManager.hpp>

#include <visualization/ShowUtils.hpp>
#include <velodyne/LidarConfig.hpp>

using namespace std;
using namespace Eigen;
using namespace pcl::visualization;

FileOperator fop;
ShowUtils su;

// vlp32的配置
LidarConfig lidar_config(VLP32);
inline float norm(PointType &p){ return std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);}

int main(int argc, const char** argv)
{
    ArgumentParser parser;
    parser.addArgument("-i", "--input_dir", true);
    parser.addArgument("-t", "--traj_path", true);
    parser.addArgument("-o", "--out_dir", true);
    parser.addArgument("-v", "--valid_distance");
    parser.addArgument("-b", "--begin_id");
    parser.addArgument("-e", "--end_id");
    parser.addArgument("-s", "--show_cloud");
    parser.addArgument("-r", "--resolution");
    parser.parse(argc, argv);

    string input_dir = parser.get("input_dir");    
    string traj_path = parser.get("traj_path");
    string out_dir = parser.get("out_dir");
    fop.makeDir(out_dir);

    float resolution = parser.getOpt<float>("resolution", 0.03);
    bool show_cloud = parser.getOpt<bool>("show_cloud", false);
    float valid_distance = parser.getOpt<float>("valid_distance", 25.0);

    if(show_cloud) su.init("Map Builder", true);

    PCDReader reader(input_dir);
    TrajIO traj(traj_path);

    std::cout << "================ runing information ===============\n"
              << "\tpcd directory = " << input_dir << std::endl
              << "\ttraj_path = " << traj_path << std::endl
              << "\toutput_dir = " << out_dir << std::endl;

    // 不指定建图范围的话，直接用轨迹中的ID建图
    int begin_id = parser.getOpt<int>("begin_id", traj.getStartID());
    int end_id = parser.getOpt<int>("end_id", traj.getEndID());

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
        
        /*
        for(int i=0; i<cloud->points.size(); ++i)
        {
            PointType p;

            if(frame_id >= 11044 && frame_id <= 11074 
                || frame_id >= 11616 && frame_id <= 11666) 
                valid_distance = 20.0;
            else valid_distance = parser.getOpt<float>("valid_distance", 25.0);

            if(norm(cloud->points[i]) < valid_distance)
            {
                pcl::copyPoint(cloud->points[i], p);
                cloud_filtered->points.push_back(p);
            }
        }*/

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
            su.ShowCloud(map.getMapPtr(), 0, "map", "custom");
            // 保留初始坐标系，显示每一帧的坐标系
            
            //if(frame_id == begin_id) su.ShowPose(m, 1);
            //else su.ShowPose(m);

            // 空格控制暂停
            su.waitSpace();
        }
        frame_id += traj.getFrameGap();
        if(frame_id>end_id) break;
        consoleProgress(frame_id, begin_id, end_id);
    }

    string out_path = out_dir+"/map_"+to_string(begin_id)+"_"+to_string(end_id)+".pcd";
    pcl::io::savePCDFileBinaryCompressed(out_path, *map.getMapPtr());
    cout << "压缩的PCD点云地图保存到: " << out_path << endl; 

    out_path = out_dir+"/map_"+to_string(begin_id)+"_"+to_string(end_id)+".las";
    saveLasFile(out_path, map.getMapPtr());
    cout << "对应的Las点云地图保存到: " << out_path << endl;
    return 0;
}



