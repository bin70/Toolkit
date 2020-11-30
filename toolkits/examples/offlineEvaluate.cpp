#include <common.hpp>
#include <utils/argparse.hpp>
#include <io/TrajIO.hpp>
#include <io/PCDOperator.hpp>
#include <point_cloud/common.hpp>
#include <build_map/MapManager.hpp>
#include <visualization/ShowUtils.hpp>

using namespace std;

FileOperator fop;
ShowUtils su;
bool ShowUtils::isPause = true;
bool show_cloud = false;
float resolution = 0.03;

vector<float> calculateError(PointCloud::Ptr refer, PointCloud::Ptr test)
{
    float sum = 0;
    int n = refer->points.size();
    
    vector<float> dis_vec(n);
    for(int i=0; i<n; ++i)
    {
        dis_vec[i] = getDistance(refer->points[i], test->points[i]);
        sum += dis_vec[i];
    }
    
    float mean = sum / n;
    float std_deviation = 0;
    for(int i=0; i<n; ++i)
    {
        std_deviation += pow(dis_vec[i]-mean, 2);
    }
    std_deviation = sqrt(std_deviation)/n;

    vector<float> error;
    error.push_back(mean);
    error.push_back(std_deviation);
    return error;
}

int main(int argc, const char **argv)
{
    ArgumentParser parser;
    parser.addArgument("-i", "--input_dir", true);
    parser.addArgument("-r", "--resolution");
    parser.addArgument("-s", "--show_cloud");
    parser.addArgument("-c", "--calib_matrix");
    parser.parse(argc, argv);

    consoleProgress(0);

    if(parser.count("resolution"))
        resolution = parser.get<float>("resolution");
    
    if(parser.count("show_cloud"))
    {
        show_cloud = parser.get<bool>("show_cloud");
        su.init("Test Cloud", &ShowUtils::keyboardEvent);
    }

    string input_dir = parser.get("input_dir");
    string keypoints_path = input_dir+"/keypoints.txt";
    string test_cloud_path = input_dir+"/test_cloud.pcd";
    string calib_matrix = input_dir+"matrix.txt";

    PointCloud::Ptr keypoints(new PointCloud);
    PointCloud::Ptr test_cloud(new PointCloud);

    pcl::io::loadPCDFile(keypoints_path, *keypoints);
    pcl::io::loadPCDFile(test_cloud_path, *test_cloud);

    if(parser.count("calib_matrix"))
    {
        Eigen::Matrix4d m = fop.loadMatrix(parser.get("calib_matrix"));
        pcl::transformPointCloud(*test_cloud, *test_cloud, m);
    }

    MapManager map(resolution);
    map.UpdateMap(test_cloud);

    su.ShowCloud(map.getMapPtr());
    su.waitSpace();

    PointType pointSel;
    PointCloud::Ptr correspond(new PointCloud);

    for(auto &point : *keypoints)
    {
        point.curvature = (float)map.getNearestPoint(point, pointSel);
        if(point.curvature)
            correspond->points.push_back(pointSel);
    }

    vector<float> error = calculateError(keypoints, correspond);
    
    std::cout << "Mean: " << error[0] << ", Deration: "<< error[1] << std::endl;

    return 0;
}