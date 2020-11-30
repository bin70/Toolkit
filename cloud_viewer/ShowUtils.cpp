#include <visualization/ShowUtils.hpp>

bool ShowUtils::stopBySpace = true;
bool ShowUtils::isPause = false;

ShowUtils::ShowUtils(){}
ShowUtils::ShowUtils(std::string name, bool _stopBySpace){ init(name, _stopBySpace);}

bool ShowUtils::isInited() const { return inited; }

// 第一次都会暂停，之后看参数设置是否由键盘来暂停
void ShowUtils::waitSpace()
{
    checkInited();

    static bool stop_once = false;

    if(!stop_once)
    {
        isPause = true;
        stop_once = true;
    }

    if (!stopBySpace)
        isPause = true;

    while (isPause)
    {
        viewer->spinOnce();
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

void ShowUtils::waitSpace(bool _isPause)
{
    checkInited();

    static bool stop_once = false;

    if(!stop_once)
    {
        isPause = _isPause;
        stop_once = true;
    }

    if (!stopBySpace)
        isPause = true;

    while (isPause)
    {
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        viewer->spinOnce();
    }
}

void ShowUtils::checkInited() const//防止可视化类未分配空间
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

void ShowUtils::setBackGroundColor(float r, float g, float b)
{
    viewer->setBackgroundColor(r, g, b);
}

PCLViewer *ShowUtils::getViewer() const { return viewer; }

using namespace pcl::visualization;
using namespace std;
using namespace Eigen;

void ShowUtils::ShowPose(const Matrix4d& t, int pose_id, bool is_keep) const
{
    Transform<double, 3, Affine> tf(t);

    static pcl::PointXYZ start = pcl::PointXYZ(0, 0, 0);
    static pcl::PointXYZ end;

    end = pcl::PointXYZ(t(0,3), t(1,3), t(2,3));
    string showid = "trajectory"+to_string(pose_id);
    viewer->addLine(start, end, 255, 255, 255, showid);
    viewer->setShapeRenderingProperties(PCL_VISUALIZER_LINE_WIDTH, 2, showid);

    string poseid = "reference";
    
    if(is_keep) poseid += to_string(pose_id);
    
    if (viewer->contains(poseid))
        viewer->removeCoordinateSystem(poseid);

    viewer->addCoordinateSystem(1.0, (const Affine3f)tf, poseid);
    viewer->spinOnce();

    start = end;
}

void ShowUtils::RemovePointCloud(string show_id)
{
    if(viewer->contains(show_id))
        viewer->removePointCloud(show_id);
}

void ShowUtils::RemoveShape(string show_id)
{
    if(viewer->contains(show_id))
        viewer->removeShape(show_id);
}

void ShowUtils::ShowCloud(const pcl::PointCloud<PointType>::Ptr& cloud,
        std::string show_id, 
        std::string show_field,
        int point_size) const
{
    checkInited();


     if (viewer->contains(show_id))
        //viewer->updatePointCloud<PointType>(cloud, show_id); 
        viewer->removePointCloud(show_id);
    //else
    //{    
        if(show_field == "custom")
        {
            PointCloudColorHandlerCustom<PointType> color_handler(cloud, 43, 213, 179);
            viewer->addPointCloud(cloud, color_handler, show_id);
            viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_OPACITY, 0.3, show_id);
        }
        else if(show_field == "black")
        {
            PointCloudColorHandlerCustom<PointType> color_handler(cloud, 0, 0, 0);
            viewer->addPointCloud(cloud, color_handler, show_id);
            //viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_OPACITY, 0.3, show_id);
        }
        else if(show_field == "dark blue")
        {
            PointCloudColorHandlerCustom<PointType> color_handler(cloud, 73, 90, 128);
            viewer->addPointCloud(cloud, color_handler, show_id);
        }
        else
        {
            PointCloudColorHandlerGenericField<PointType> cloud_handle(cloud, show_field);
            viewer->addPointCloud(cloud, cloud_handle, show_id);
        }

        viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_POINT_SIZE, point_size, show_id);
    //}
    viewer->spinOnce();
}

void ShowUtils::ShowCloud(const PointCloud::Ptr& cloud,
    int id, string cloud_name, std::string show_field, int point_size) const
{
    std::string cloud_id = cloud_name + to_string(id);
    ShowCloud(cloud, cloud_id, show_field, point_size);
}

void ShowUtils::ShowPlane(const Eigen::Vector4d& ABCD, const Eigen::Vector3d& center,
    const std::string& showid, bool only_show_name) const
{
    using XYZ = pcl::PointXYZ;

    if(viewer->contains("text_"+showid))
        viewer->removeShape("text_"+showid);
    
    // 显示名称
    viewer->addText3D(showid, XYZ(center[0], center[1], center[2]), 
        0.5, // text scale
        1.0, 1.0, 1.0, // color 
        "text_"+showid);
        
    if(!only_show_name)
    {
        if(viewer->contains(showid))
        {
            viewer->removeShape("normal_"+showid);
            viewer->removeShape(showid);
        }
        // 可视化平面
        pcl::ModelCoefficients coeff;
        coeff.values.push_back(ABCD[0]);
        coeff.values.push_back(ABCD[1]);
        coeff.values.push_back(ABCD[2]);
        coeff.values.push_back(ABCD[3]); //  注意Plane类中存储的D=-(Ax+By+Cz)
        
        viewer->addPlane(coeff, center[0], center[1], center[2], showid);

        // 可视化法向
        Vector3d end = center + ABCD.block(0,0,3,1); //.normalized().block(0,0,3,1);
        viewer->addArrow<XYZ>(XYZ(end[0], end[1], end[2]),
            XYZ(center[0], center[1], center[2]),
            1.0, 1.0, 1.0, // color
            false, // 显示法向量的模长
            "normal_"+showid);
    }

    viewer->spinOnce(); 
}

void ShowUtils::ShowLine(const pcl::PointXYZ& start,
    const pcl::PointXYZ& end, const std::string& showid, 
    int label, int line_size) const
{
    int r, g, b;

    switch(label)
    {
        case 0:
            r = 255; g = 0; b = 255; break;
            //r = 255; g = 0; b = 153; break;
        case 1:
            r = 255; g = 0; b = 0; break;
            //r = 0; g = 255; b = 0; break;
        case 2:
            r = 255; g = 255; b = 0; break;
    };

    if(viewer->contains(showid))
        viewer->removeShape(showid);
    viewer->addLine(start, end, r, g, b, showid);
    viewer->setShapeRenderingProperties(PCL_VISUALIZER_LINE_WIDTH, line_size, showid);
    viewer->spinOnce(); 
}

void ShowUtils::ShowPath3D(
    const std::vector<pcl::PointXYZI> &path,
    int path_id,
    int line_size,
    int label) const
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
    std::string showid;
    for (int i = 0; i < path.size() - 1; ++i)
    {
        start = path[i];
        end = path[i + 1];
        showid =  pathid + std::to_string(i);
        //if(!viewer->contains(showid))
        //{   
            viewer->addLine(start, end, r, g, b, showid);
            viewer->setShapeRenderingProperties(PCL_VISUALIZER_LINE_WIDTH, line_size, showid);
            viewer->spinOnce();
            //viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_LINE_WIDTH, line_size, showid);
            //viewer->setPointCloudRenderingProperties(PCL_VISUALIZER_COLOR, r, g, b, showid);
        //}
        
    }
}

void ShowUtils::ShowText(
    std::string text, std::string display_id,
    Vector3d position,
    Vector3d display_color,
    float display_size) const
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