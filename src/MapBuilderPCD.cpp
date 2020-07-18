#include <io/PCDReader.hpp>
#include <io/TrajIO.hpp>
#include <io/FileOperator.hpp>
#include <build_map/MapManager.hpp>
#include <argparse.hpp>
#include <pcl/common/transforms.h>

using namespace std;
using namespace Eigen;

FileOperator fop;
bool show_cloud = false;
float resolution = 0.03;

int main(int argc, const char** argv)
{
    ArgumentParser parser;
    parser.addArgument("-i", "--input_dir", true);
    parser.addArgument("-b", "--begin_id", true);
    parser.addArgument("-e", "--end_id", true);
    parser.addArgument("-t", "--traj_type", true);
    parser.addArgument("-o", "--output_dir");
    parser.addArgument("-s", "--show_cloud");
    parser.addArgument("-r", "--resolution");
    parser.parse(argc, argv);

    string input_dir = parser.get("input_dir");    
    int begin_id = parser.get<int>("begin_id");
    int end_id = parser.get<int>("end_id");

    if(parser.count("show_cloud"))
        show_cloud = parser.get<bool>("show_cloud");

    if(parser.count("resolution"))
        resolution = parser.get<float>("resolution");
    
    string output_dir = input_dir+"/output_"+to_string(begin_id)+"_"+to_string(end_id);
    if(parser.count("output_dir"))
        output_dir = parser.get("output_dir");
    fop.makeDir(output_dir);

    PCDReader reader(input_dir+"/PCD");
    reader.setBinary(true);
    
    TrajIO traj(input_dir+"/traj_with_timestamp.txt", 
                parser.get<int>("traj_type"));

    PointCloud::Ptr cloud(new PointCloud);
    int frame_id = begin_id;

    MapManager map(resolution);
    map.update();

    while(reader.readPointCloud(cloud, frame_id))
    {
        // TODO: 增加重建LOAM的特征地图并保存的功能

        Matrix4d m = traj.getPoseMatrix(frame_id);
        pcl::transformPointCloud(cloud, cloud, m);
        
        

        frame_id += traj.getFrameGap();
        if(frame_id>end_id) break;
    }
}



