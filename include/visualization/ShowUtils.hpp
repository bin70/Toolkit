#pragma once
#include <common.hpp>
#include <point_cloud/common.hpp>
#include <pcl/visualization/pcl_visualizer.h>

/**********************************************
 * 可视化工具使用方法: 
 * 1.实例化
 *   ShowUtils su;
 *  
 * 2.初始化静态成员
 *   bool ShowUtils::isPause = true;
 * 
 * 3.初始化键盘响应
 *   su.init(name, ShowUtils::keyboardEvent); 
 *********************************************/

class ShowUtils
{
    using PCLViewer = pcl::visualization::PCLVisualizer;
    using KeyboardEvent = pcl::visualization::KeyboardEvent;

private:    
    bool inited = false;
    PCLViewer* viewer;
    typedef void (*KeyboardEventFunc)(const KeyboardEvent &, void*);

public:
    static bool isPause;
    // 继续执行view

    static void keyboardEvent(const KeyboardEvent& event, void* nothing){
        if(event.getKeySym() == "space" && event.keyDown()){
            isPause = false;
        }
    }

    ShowUtils(){}
    
    ShowUtils(std::string name, KeyboardEventFunc func)
    {
        init(name, func);
    }

    bool isInited() {return inited;}

    void checkInited() //防止可视化类未分配空间
    {
        if(!isInited())
        {
            std::cout << "ShowUtils is not be inited!" << std::endl;
            exit(-1);
        }
    }

    void init(std::string name, KeyboardEventFunc func)
    {
        viewer = new PCLViewer(name);
        viewer->registerKeyboardCallback(func, (void*)NULL);
        inited = true; 
    }

    PCLViewer* getViewer() { return viewer; }

    void waitSpace()
    {
        checkInited();
        isPause = true;
        while (isPause)
            viewer->spinOnce();
    }
    
    inline void ShowCloud(
        const pcl::PointCloud<PointType>::Ptr cloud,
        std::string cloud_id = "cloud", 
        std::string show_field = "intensity",
        int point_size = 1)
    {
        checkInited();
        if (viewer->contains(cloud_id)) viewer->removePointCloud(cloud_id);

        //从电云intensity字段生成颜色信息
        pcl::visualization::PointCloudColorHandlerGenericField<PointType> cloud_handle(cloud, show_field);
        viewer->addPointCloud(cloud, cloud_handle, cloud_id);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, cloud_id);
        viewer->spinOnce();
    }

    void ShowPath3D(
        const std::vector<pcl::PointXYZI>& path, 
        int path_id,
        int line_size = 2)
    {
        std::string pathid = "path" + std::to_string(path_id) + "_";
        pcl::PointXYZI start, end;
        for(int i=0; i<path.size()-1; ++i)
        {
            start = path[i];
            end = path[i+1];
            viewer->addLine(start, end, pathid+std::to_string(i));
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, line_size, pathid+std::to_string(i));
            viewer->spinOnce();
        }
    }

    void showText( 
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