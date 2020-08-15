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
using PCLViewer = pcl::visualization::PCLVisualizer;
using KeyboardEvent = pcl::visualization::KeyboardEvent;
typedef void (*KeyboardEventFunc)(const KeyboardEvent &, void*);

class ShowUtils
{
private:    
    bool inited = false;
    PCLViewer* viewer;
    static void keyboardEvent(const KeyboardEvent& event, void* nothing){
        if(event.getKeySym() == "space" && event.keyDown()){
            if(!stopBySpace)
                isPause = false;
            else
                isPause = !isPause;
        }
    }
    static bool stopBySpace;
    static bool isPause;
    void checkInited(); //防止可视化类未分配空间
    void init(std::string name, KeyboardEventFunc func);

public:
    ShowUtils();
    ShowUtils(std::string name, bool _stopBySpace = false);
    void init(std::string name, bool _stopBySpace = false);
    void waitSpace();
    bool isInited();
    PCLViewer* getViewer(); 
    
    void ShowCloud(const pcl::PointCloud<PointType>::Ptr cloud,
        std::string cloud_id = "cloud", std::string show_field = "intensity",
        int point_size = 1);

    void ShowPath3D(const std::vector<pcl::PointXYZI>& path, 
        int path_id, int line_size = 2);

    void showText(std::string text, std::string display_id, 
        Eigen::Vector3d position = Eigen::Vector3d::Zero(), 
        Eigen::Vector3d display_color = Eigen::Vector3d::Ones(),
        float display_size = 1.0);
};