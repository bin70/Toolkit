#include <visualization/ShowUtils.hpp>

bool ShowUtils::stopBySpace = true;
bool ShowUtils::isPause = false;

ShowUtils::ShowUtils(){}
ShowUtils::ShowUtils(std::string name, bool _stopBySpace){ init(name, _stopBySpace);}

bool ShowUtils::isInited() { return inited; }

void ShowUtils::waitSpace()
{
    checkInited();

    if (!stopBySpace)
        isPause = true;

    while (isPause)
        viewer->spinOnce();
}


void ShowUtils::checkInited() //防止可视化类未分配空间
{
    if (!isInited())
    {
        std::cout << "ShowUtils is not be inited!" << std::endl;
        exit(-1);
    }
}

void ShowUtils::init(std::string name, KeyboardEventFunc func)
{
    viewer = new PCLViewer(name);
    viewer->registerKeyboardCallback(func, (void *)NULL);
    inited = true;
}

void ShowUtils::init(std::string name, bool _stopBySpace)
{
    stopBySpace = _stopBySpace;
    init(name, &ShowUtils::keyboardEvent);
}

PCLViewer *ShowUtils::getViewer() { return viewer; }

using namespace pcl::visualization;
using namespace std;
using namespace Eigen;

void ShowUtils::ShowPose(const Matrix4d& t, int pose_id)
{
    Transform<double, 3, Affine> tf(t);

    string poseid = "reference" + to_string(pose_id);
    
    if (viewer->contains(poseid))
        viewer->removeCoordinateSystem(poseid);

    viewer->addCoordinateSystem(1.0, (const Affine3f)tf, poseid);
    //viewer->addText(std::to_string(pose_id), t(0,3), t(1,3), "text"+std::to_string(pose_id), 0);
    //pcl::PointXYZ position;
    //position.x = t(0, 3);
    //position.y = t(1, 3);
    //position.z = t(2, 3);
    // viewer->addText3D(std::to_string(pose_id), position, 0.5, 1.0, 1.0, 1.0, "text" + std::to_string(pose_id), 0);
    viewer->spinOnce();
}

void ShowUtils::ShowCloud(const PointCloud::Ptr cloud,
    int id, string cloud_name, std::string show_field, int point_size)
{
    checkInited();

    std::string cloud_id = cloud_name + to_string(id);

    if (viewer->contains(cloud_id))
        viewer->removePointCloud(cloud_id);
    
    if(show_field == "custom")
    {
        PointCloudColorHandlerCustom<PointType> color_handler(cloud, 43, 213, 179);
        viewer->addPointCloud(cloud, color_handler, cloud_id);
        viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_OPACITY, 0.3, cloud_id);
    }
    else
    {
        PointCloudColorHandlerGenericField<PointType> cloud_handle(cloud, show_field);
        viewer->addPointCloud(cloud, cloud_handle, cloud_id);
    }

    viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, point_size, cloud_id);
    viewer->spinOnce();
}

void ShowUtils::ShowPath3D(
    const std::vector<pcl::PointXYZI> &path,
    int path_id,
    int line_size,
    int label)
{
    std::string pathid = "path" + std::to_string(path_id) + "_";
    int r, g, b;

    switch(label)
    {
        case 0: 
            r = 255; g = 0; b = 153; break;
        case 1:
            r = 9; g = 151; b = 247; break;
        case 2:
            r = 153; g = 255; b = 0; break;
    };

    pcl::PointXYZI start, end;
    for (int i = 0; i < path.size() - 1; ++i)
    {
        start = path[i];
        end = path[i + 1];
        viewer->addLine(start, end, pathid + std::to_string(i));
        viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_LINE_WIDTH, line_size, pathid + std::to_string(i));
        viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_COLOR, r, g, b, pathid + std::to_string(i));
        viewer->spinOnce();
    }
}

void ShowUtils::ShowText(
    std::string text, std::string display_id,
    Vector3d position,
    Vector3d display_color,
    float display_size)
{
    pcl::PointXYZ pos;
    pos.x = position(0);
    pos.y = position(1);
    pos.z = position(2);

    if (viewer->contains(display_id))
        viewer->removeShape(display_id);

    viewer->addText3D(text, pos, display_color(0), display_color(1), display_color(2),
                      display_size, display_id);
    viewer->spinOnce();
}