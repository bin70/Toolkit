#pragma once
#include <common.hpp>
#include <point_cloud/common.hpp>
#include <pcl/visualization/pcl_visualizer.h>

class ShowUtilsHelper
{
public:
    
};

class ShowUtils
{
private:    
    bool inited = false;
    pcl::visualization::PCLVisualizer* viewer;
    typedef void (*KeyboardEventFunc)(const pcl::visualization::KeyboardEvent &, void*);

public:
    static bool isPause;
    // 继续执行view

    static void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing){
        if(event.getKeySym() == "space" && event.keyDown()){
            isPause = false;
        }
    }

    ShowUtils(){}
    //ShowUtils(std::string show_name) { init(show_name); }
    void checkInited() //防止可视化类未分配空间
    {
        if(!inited)
        {
            std::cout << "ShowUtils is not be inited!" << std::endl;
            exit(-1);
        }
    }



    void init(std::string name, KeyboardEventFunc func)
    {
        viewer = new pcl::visualization::PCLVisualizer(name);
        viewer->registerKeyboardCallback(func, (void*)NULL);
        inited = true; 
    }

    void init(std::string name, KeyboardEventFunc func, bool *_isPause)
    {
        viewer = new pcl::visualization::PCLVisualizer(name);
        viewer->registerKeyboardCallback(func, (void*)NULL);
        *_isPause = true; 
        inited = true; 
    }

    void waitForSpace()
    {
        checkInited();
        isPause = true;
        while (isPause)
            viewer->spinOnce();
    }

    pcl::visualization::PCLVisualizer* getViewer() { return viewer; }

    void waitForSpace(bool &isPause)
    {
        checkInited();
        isPause = true;
        while (isPause)
            viewer->spinOnce();
    }
    
    inline void
    ShowCloud(const pcl::PointCloud<PointType>::Ptr cloud,
              std::string cloud_id = "cloud", std::string show_field = "intensity")
    {
        checkInited();
        if (viewer->contains(cloud_id))
            viewer->removePointCloud(cloud_id);

        //从电云intensity字段生成颜色信息
        pcl::visualization::PointCloudColorHandlerGenericField<PointType> cloud_handle(cloud, show_field);
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
};