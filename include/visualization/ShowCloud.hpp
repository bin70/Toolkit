#include <common.hpp>
#include <point_cloud/common.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace vis_utils
{
    bool isPause = true;

    void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                               void *nothing)
    {
        if (event.getKeySym() == "space" && event.keyDown())
            isPause = false;
    }

    void waitForSpace(pcl::visualization::PCLVisualizer *viewer)
    {
        isPause = true;
        while (isPause)
            viewer->spinOnce();
    }

    //以强度信息作为颜色显示点云
    inline void ShowCloud(const PointCloud::Ptr cloud,
                          pcl::visualization::PCLVisualizer *viewer,
                          std::string color_field = "intensity",
                          int size_of_point = 1)
    {
        std::string cloud_id = "cloud_" + color_field;
        if (viewer->contains(cloud_id))
            viewer->removePointCloud(cloud_id);

        //从电云intensity字段生成颜色信息
        pcl::visualization::PointCloudColorHandlerGenericField<PointType> cloud_handle(cloud, color_field);
        //viewer->setBackgroundColor(255, 255, 255);
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addPointCloud(cloud, cloud_handle, cloud_id);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size_of_point, cloud_id);
        viewer->spinOnce();
    }

    inline void ShowCloud(const PointCloud::Ptr cloud,
                          pcl::visualization::PCLVisualizer *viewer,
                          int r, int g, int b)
    {
        std::stringstream ss;
        ss << "cloud_" << r << g << b;
        std::string cloud_id = ss.str();
        if (viewer->contains(cloud_id))
        {
            viewer->removePointCloud(cloud_id);
        }

        pcl::visualization::PointCloudColorHandlerCustom<PointType> cloud_handle(cloud, r, g, b);
        viewer->addPointCloud(cloud, cloud_handle, cloud_id);
        viewer->spinOnce();
    }

    void showText(pcl::visualization::PCLVisualizer *viewer, 
        std::string text, std::string display_id, 
        Eigen::Vector3d position = Eigen::Vector3d::Zero(), 
        Eigen::Vector3d display_color = Eigen::Vector3d::Ones(),
        float display_size = 1.0)
    {
        pcl::PointXYZ pos;
        pos.x = position(0);
        pos.y = position(1);
        pos.z = position(2);

        if(viewer->contains(display_id))
            viewer->removeShape(display_id);

        viewer->addText3D(text, pos, display_color(0), display_color(1), display_color(2), 
            display_size, display_id);
        viewer->spinOnce();
    }
} // namespace vis_utils