#pragma once
#include <common.hpp>
#include <point_cloud/common.hpp>
#include <pcl/visualization/pcl_visualizer.h>

/**********************************************
 * 可视化工具使用方法: 
 * 1. CMakeList中的配置
 * - include_directories(Toolkit_DIR/include)
 * - link_directories(Toolkit_DIR/lib)
 * - target_link_libraries(executable ShowUtils)
 * 
 * 2. 代码中使用
 * - #include <visualization/ShowUtils.hpp>
 * - ShowUtils su(name, stopBySpace);
 *   参数说明:
 *   name 窗口名称
 * 　stopBySpace 是否通过空格键暂停（默认false,会自动在调用waitSpace处暂停）
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
        int id = 0, std::string cloud_name = "cloud", 
        std::string show_field = "intensity",
        int point_size = 1);
    
    void ShowLine(const pcl::PointXYZ start, const pcl::PointXYZ end, 
        std::string showid, int label = 0, int line_size = 2);

    void ShowPath3D(const std::vector<pcl::PointXYZI>& path, 
        int path_id, int line_size = 2, int label = 0);
    
    void ShowPose(const Eigen::Matrix4d& t, int pose_id = 0);

    void ShowText(std::string text, std::string display_id, 
        Eigen::Vector3d position = Eigen::Vector3d::Zero(), 
        Eigen::Vector3d display_color = Eigen::Vector3d::Ones(),
        float display_size = 1.0);
};