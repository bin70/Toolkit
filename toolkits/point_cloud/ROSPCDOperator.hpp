#pragma once 
#include <common.hpp>
#include <io/FileOperator.hpp>
#include <point_cloud/common.hpp>
#include <velodyne/LidarConfig.hpp>

double frametime(std::string path)
{
    int begin = path.rfind('/');
    int end = path.rfind('.');
    std::string timestamp_str = path.substr(begin + 1, end - begin - 1);
    return atof(timestamp_str.c_str());
}

class ROSPCDReader
{
public:
    FileOperator fop;
    std::vector<std::string> pcd_files;
    LidarType lidar;
    float validDistance;
    LidarConfig *lidar_config;

    // 只给定距离时，说明是双头建图，并且是第二个头，所以是16线的头
    ROSPCDReader(float valid_distance) { lidar = VLP16; validDistance = valid_distance;}

    ROSPCDReader(std::string pcd_dir, LidarType _lidar, float valid_distance = 25.0)
    { 
        lidar = _lidar;
        validDistance = valid_distance;
        openPCDDir(pcd_dir);     
    }
    
    bool readPointCloud(PointCloud::Ptr &cloud, int frame_id)
    {
        if(!cloud) return false;

        cloud->clear();

        if(frame_id < 0 || frame_id > getFilesNumber())
            return false;
        else
            return readPCD(pcd_files[frame_id], cloud);
    };

    void openPCDDir(std::string path)
    {
        assert( fop.isExist(path) );
        
        switch (lidar)
        {
        case VLP16:
            lidar_config = new LidarConfig(16, 15.0, -15.0);
            break;
        case HDL32:
            lidar_config = new LidarConfig(32, 10.67, -30.67);
            break;
        case VLP32:
            lidar_config = new LidarConfig(32, 15.0, -25.0);
            break;
        default:
            break;
        };
    

        DIR *dir = opendir(path.c_str());
        dirent *ptr;
        while ((ptr = readdir(dir)) != NULL)
        {
            if (ptr->d_name[0] == '.') continue;
            
            std::string filepath = path + "/" + std::string(ptr->d_name); 
            double ti = frametime(filepath);

            // 文件名添加到列表
            pcd_files.push_back(filepath);
        }
        closedir(dir);

        sort(pcd_files.begin(), pcd_files.end(), [](std::string a, std::string b){ return frametime(a)<frametime(b); });
        
        std::cout << "================================\n"
                  << path << " has " << pcd_files.size() << " files\n"
                  << "================================\n";
    };
    
    int getFilesNumber(){return pcd_files.size();}

    inline float norm(PointType &p){ return std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);}

    bool readPCD(std::string file, PointCloud::Ptr &cloud)
    {
        std::ifstream fs(file.c_str());
        if (!fs.is_open() || fs.fail())
        {
            PCL_ERROR("Could not open: %s", file.c_str());
            return false;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr ros_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        if(pcl::io::loadPCDFile(file, *ros_cloud) == -1)
        {
            std::cout << "Read pcd file: " << file << "failed." << std::endl;
            return false;
        }

        //std::cout << "size of ros-cloud: " << ros_cloud->points.size() << std::endl;

        for(int i=0; i<ros_cloud->points.size(); ++i)
        {
            pcl::PointXYZI p = ros_cloud->points[i];
            int scanID = lidar_config->getScanID(p);

            PointType pt;
            pt.y = p.x;
            pt.x = -p.y;
            pt.z = p.z;
            pt.intensity = p.intensity;

            float dist = norm(pt);
            if( dist > validDistance)
                continue;

            double timestamp = frametime(file);
            pt.data_n[0] = int(timestamp/1e6);						   //data_n[0]存储秒
            pt.data_n[1] = (timestamp - pt.data_n[0]*1e6)/1e6;		   //data_n[1]存储微秒
            pt.data_n[2] = scanID; //data_n[2]存储线编号
            pt.data_n[3] = 0; //这个字段存不下来，不知道为什么

            pt.intensity = pt.data_n[2];
            pt.curvature = dist;
            cloud->points.push_back(pt);
        }
        cloud->height = 1;
		cloud->width = cloud->points.size();
		cloud->is_dense = true;

        //std::cout << "size of cloud is: " << cloud->points.size() << std::endl;
        return true;
    }


    double getTimestamp(int frame_id)
    {
        return frametime(pcd_files[frame_id]);
    }

};
