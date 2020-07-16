#include <pcl/common/transforms.h>
#include <io/PcapReader.hpp>
#include <io/TrajIO.hpp>
#include <io/LasOperator.hpp>
#include <visualization/ShowCloud.hpp>
#include <build_map/MapManager.hpp>
#include <argparse.hpp>

#define USE_OCTO_MAP 1
#define USE_LOAM_POSE 0 // 是否使用LOAM的位姿表示方法(欧拉角)
#define TEST_TF_2_EULER 0

#if USE_LOAM_POSE
#include <loam/transform.hpp>
Twist _transformSum;
#endif

using namespace std;
using namespace Eigen;

string getCalibFile(int data_type)
{
    switch (data_type)
    {
    case 0:
        return "../resource/VLP-16.xml";
    case 1:
        return "../resource/HDL-32.xml";
    case 2:
        return "../resource/VLP-32c.xml";
    default:
        cout << "Data type error!" << endl;
    }
}

Matrix4d loadCalibMatrix()
{
    ifstream matrix_file("../resource/autoCalibMatrix.txt");
    Matrix4d matrix;
    for(int i=0; i<4; ++i)
        for(int j=0; j<4; ++j)
            matrix_file >> matrix(i,j);
    return matrix;
}

pcl::visualization::PCLVisualizer *viewer;
bool is_show = false;
float resolution = 0.03;
int begin_id = 0;
int end_id = -1;
string out_dir = "./output";

int main(int argc, const char **argv)
{
    ArgumentParser parser;
    parser.addArgument("-p", "--pcap", true);
    parser.addArgument("-d", "--data_type", true);
    parser.addArgument("-g", "--gap_of_frame", true);
    parser.addArgument("-o", "--out_dir", true);
    parser.addArgument("-m", "--pcap2"); //可选双头
    parser.addArgument("-b", "--begin_id");
    parser.addArgument("-e", "--end_id");
    parser.addArgument("-s", "--show");
    parser.addArgument("-r", "--resolution");
    parser.addArgument("-c", "--calib_matrix"); //是否使用一个初始的标定矩阵变换雷达帧（斜装头建图时才用到）
    parser.parse(argc, argv);

    string::out_dir = parser.get("out_dir") + "/" + getFileName(parser.get("pcap"));
    CreateDir(out_dir.c_str());

    if (parser.count("show"))
    {   
        is_show = parser.get<bool>("show");
        viewer = new pcl::visualization::PCLVisualizer("default");
    }

    if(parser.count("begin_id"))
        begin_id = parser.get<int>("begin_id");
    
    if(parser.count("end_id"))
        end_id = parser.get<int>("end_id");

    PointCloudReader reader;
    reader.setPcapFile(parser.get("pcap"));
    reader.setCalibFile(getCalibFile(parser.get<int>("data_type")));
    reader.setVoxelSize(0.03); // 单帧分辨率
    reader.setValidDistance(25.0);
    reader.init();

    PointCloudReader reader2;
    if(parser.count("pcap2"))
    {
        reader2.setPcapFile(parser.get("pcap2"));
        reader2.setCalibFile(getCalibFile(0)); // 第二个雷达一定是16线
        reader2.setVoxelSize(0.03); // 单帧分辨率
        reader2.setValidDistance(25.0);
        reader2.init();
    }

    PointCloud::Ptr cloud(new PointCloud);
    PointCloud::Ptr cloud2(new PointCloud);
    long long frameID = begin_id;
    consoleProgress(0);

    // 把201标定到202上的矩阵
    Matrix4d calibMatrix = loadCalibMatrix();

    while (reader.readPointCloud(cloud, frameID))
    {
        // 使用双头的数据建图
        if(parser.count("pcap2"))
        {
            if(!reader2.readPointCloud(cloud2, frameID))
            {
                cout << "pcap2 is end!" << endl;
                break;
            }
            // 合并两帧
            pcl::transformPointCloud(*cloud2, *cloud2, calibMatrix);
            *cloud += *cloud2;
        }

        //  单独使用斜着的头建图时才会用到
        if(parser.count("calib_matrix"))
            pcl::transformPointCloud(*cloud, *cloud, calibMatrix);

        if (is_show)
            vis_utils::ShowCloud(cloud, viewer, "intensity", 3);

        string out_path = out_dir + "/" + to_string(frameID) + ".pcd";
        pcl::io::savePCDFileBinaryCompressed<PointType>(out_path, *cloud);
    
        // 间隔几帧
        frameID += parser.get<int>("gap_of_frame");

        // 结束帧
        if (frameID > end_id)
            break;

        consoleProgress(frameID, begin_id, end_id);
    }

    cout << "压缩的PCD文件保存至: " << out_dir << endl;

    return 0;
}
