#include <pcl/common/transforms.h>
#include <io/PcapReader.hpp>
#include <io/PCDOperator.hpp>
#include <io/FileOperator.hpp>
#include <visualization/ShowCloud.hpp>
#include <point_cloud/PointCloudFilter.hpp>
#include <point_cloud/Synchrotimer.hpp>
#include <argparse.hpp>

using namespace std;
using namespace Eigen;

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
float limit_distance = 25.0;
int begin_id = 0;
int end_id = -1;
FileOperator fop;

int main(int argc, const char **argv)
{
    ArgumentParser parser;
    parser.addArgument("-p", "--pcap", true);
    parser.addArgument("-d", "--data_type", true);
    parser.addArgument("-g", "--gap_of_frame", true);
    parser.addArgument("-o", "--out_dir", true);
    parser.addArgument("-l", "--limit_distance");
    parser.addArgument("-f", "--filter_cloud"); // 是否对点云进行滤波
    parser.addArgument("-m", "--pcap2"); //可选双头
    parser.addArgument("-b", "--begin_id");
    parser.addArgument("-e", "--end_id");
    parser.addArgument("-s", "--show");
    parser.addArgument("-r", "--resolution");
    parser.addArgument("-c", "--calib_matrix"); //是否使用一个初始的标定矩阵变换雷达帧（斜装头建图时才用到）
    parser.parse(argc, argv);

    string out_dir = parser.get("out_dir") + "/" + getFileName(parser.get("pcap"));
    if(parser.count("begin_id"))
        begin_id = parser.get<int>("begin_id");
    if(parser.count("end_id"))
        end_id = parser.get<int>("end_id");

    out_dir += "_" + to_string(begin_id) + "_" + to_string(end_id) + "/PCD";
    fop.makeDir(out_dir);

    if (parser.count("show"))
    {   
        is_show = parser.get<bool>("show");
        viewer = new pcl::visualization::PCLVisualizer("default");
    }

    PointCloudReader reader;
    reader.setPcapFile(parser.get("pcap"));
    reader.setDataType(parser.get<int>("data_type"));
    reader.setVoxelSize(resolution); // 单帧分辨率
    reader.setValidDistance(limit_distance);
    reader.init();

    PointCloudReader reader2;
    Synchrotimer sync;
    int frameOffset = 0;
    if(parser.count("pcap2"))
    {
        reader2.setPcapFile(parser.get("pcap2"));
        reader2.setDataType(0); // 第二个雷达一定是16线
        reader2.setVoxelSize(resolution); // 单帧分辨率
        reader2.setValidDistance(limit_distance);
        reader2.init();

        sync.setStartFrameID(begin_id);
    }

    PointCloud::Ptr cloud(new PointCloud);
    PointCloud::Ptr cloud2(new PointCloud);
    long long frameID = begin_id;
    consoleProgress(0);

    // 把201标定到202上的矩阵
    Matrix4d calibMatrix = loadCalibMatrix();

    while (reader.readPointCloud(cloud, frameID))
    {
        if(parser.count("filter_cloud"))
        {
            PointCloudFilter pcf;
            pcf.setOutlierFilter();
            pcf.filter(cloud, cloud);
        }

        copyScanID(cloud);
        
        // 使用双头的数据建图
        if(parser.count("pcap2"))
        {
            if(!reader2.readPointCloud(cloud2, frameID+frameOffset))
            {
                cout << "pcap2 is end!" << endl;
                break;
            }

            if(parser.count("filter_cloud"))
            {
                PointCloudFilter pcf;
                pcf.setOutlierFilter();
                pcf.filter(cloud2, cloud2);
            }

            copyScanID(cloud2, false);
            
            switch(sync.correctFrameOffset(cloud, cloud2, frameID, frameOffset))
            {
                case 0:
                    continue;
                case 1:
                    pcl::transformPointCloud(*cloud2, *cloud2, calibMatrix);
                    // 分开存同步好的双雷达数据
                    savePCD(cloud, out_dir + "/201", frameID);
                    savePCD(cloud2, out_dir + "/202", frameID);
                
                    // 合并两帧
                    *cloud += *cloud2;
                    break;
                default: // 直接丢弃202的数据，用201的代替
                    savePCD(cloud, out_dir + "/201", frameID);
                    savePCD(cloud, out_dir + "/202", frameID);
                    break;
            }
        }

        //  单独使用斜着的头建图时才会用到
        if(parser.count("calib_matrix"))
            pcl::transformPointCloud(*cloud, *cloud, calibMatrix);

        if (is_show)
        {
            vis_utils::showText(viewer, to_string(frameID), "frameID");
            vis_utils::ShowCloud(cloud, viewer, "intensity", 3);
        }
        savePCD(cloud, out_dir, frameID);
        
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
