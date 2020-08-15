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

void ShowUtils::ShowCloud(const pcl::PointCloud<PointType>::Ptr cloud,
    std::string cloud_id, std::string show_field, int point_size)
{
    checkInited();
    if (viewer->contains(cloud_id))
        viewer->removePointCloud(cloud_id);

    //从电云intensity字段生成颜色信息
    pcl::visualization::PointCloudColorHandlerGenericField<PointType> cloud_handle(cloud, show_field);
    viewer->addPointCloud(cloud, cloud_handle, cloud_id);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, cloud_id);
    viewer->spinOnce();
}

void ShowUtils::ShowPath3D(
    const std::vector<pcl::PointXYZI> &path,
    int path_id,
    int line_size)
{
    std::string pathid = "path" + std::to_string(path_id) + "_";
    pcl::PointXYZI start, end;
    for (int i = 0; i < path.size() - 1; ++i)
    {
        start = path[i];
        end = path[i + 1];
        viewer->addLine(start, end, pathid + std::to_string(i));
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, line_size, pathid + std::to_string(i));
        viewer->spinOnce();
    }
}

void ShowUtils::showText(
    std::string text, std::string display_id,
    Eigen::Vector3d position,
    Eigen::Vector3d display_color,
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