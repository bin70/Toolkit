#pragma once
#include <lidar_slam_toolkit/common.hpp>
#include <lidar_slam_toolkit/point_cloud/common.hpp>
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
        else if(event.getKeySym() == "Return" && event.keyDown())
        {
            std::cout << "Closed by user." << std::endl;
            exit(0);
        }
    }
    static bool stopBySpace;
    static bool isPause;
    void checkInited() const; //防止可视化类未分配空间
    void init(std::string name, KeyboardEventFunc func);

public:
    ShowUtils();
    ShowUtils(std::string name, bool _stopBySpace = false);
    void init(std::string name, bool _stopBySpace = false);
    void waitSpace();
    void waitSpace(bool _isPause);
    bool isInited() const;
    PCLViewer* getViewer() const; 
    
    void setBackGroundColor(float r, float g, float b);

    void RemovePointCloud(std::string show_id);
    void RemoveShape(std::string show_id);

    void ShowCloud(const pcl::PointCloud<PointType>::Ptr& cloud,
        std::string show_id = "cloud", 
        std::string show_field = "intensity",
        int point_size = 2) const;

    void ShowCloud(const pcl::PointCloud<PointType>::Ptr& cloud,
        int id, std::string cloud_name = "cloud", 
        std::string show_field = "intensity",
        int point_size = 1) const;
    
    void ShowPlane(const Eigen::Vector4d& ABCD, const Eigen::Vector3d& center,
        const std::string& showid, bool only_show_name = false) const;

    void ShowLine(const pcl::PointXYZ& start, const pcl::PointXYZ& end, 
        const std::string& showid, int label = 0, int line_size = 2) const;

    void ShowPath3D(const std::vector<pcl::PointXYZI>& path, 
        int path_id, int line_size = 2, int label = 0) const;
    
    void ShowPose(const Eigen::Matrix4d& t, int pose_id = 0, bool is_keep = false) const;

    void ShowText(std::string text, std::string display_id, 
        Eigen::Vector3d position = Eigen::Vector3d::Zero(), 
        Eigen::Vector3d display_color = Eigen::Vector3d::Ones(),
        float display_size = 1.0) const;
};