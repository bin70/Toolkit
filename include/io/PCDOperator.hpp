#pragma once
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <stdio.h>
#include <common.hpp>
#include <point_cloud/common.h>
#include <algorithm>

inline int frameid(std::string path)
{
    int begin = path.rfind('/');
    int end = path.rfind('.');
    std::string id_str = path.substr(begin + 1, end - begin - 1);
    return atoi(id_str.c_str());
}


inline void copyScanID(PointCloud::Ptr cloud, bool isVLP32 = true)
{
    // 把scanID和雷达头信息存入curvature以便显示
    for(int i=0; i<cloud->points.size(); ++i)
    {
        if(isVLP32)
            cloud->points[i].curvature = cloud->points[i].data_n[2];
        else
            cloud->points[i].curvature = cloud->points[i].data_n[2]+150;
    }
}

// 指定一个保存的文件夹，将会以帧号命名，存下压缩的二进制PCD
inline bool savePCD(PointCloud::Ptr &cloud, std::string save_dir, int frame_id)
{
    // 建立文件夹
    char DirName[256];
    strcpy(DirName, save_dir.c_str());
    int i, len = strlen(DirName);
    if (DirName[len - 1] != '/')
        strcat(DirName, "/");
    len = strlen(DirName);
    for (i = 1; i < len; i++)
    {
        if (DirName[i] == '/' || DirName[i] == '\\')
        {
            DirName[i] = 0;
            if (access(DirName, 0) != 0) //存在则返回0
            {
                if (mkdir(DirName, 0755) == -1)
                {
                    perror("mkdir   error");
                    return false;
                }
            }
            DirName[i] = '/';
        }
    }
    
    return (pcl::io::savePCDFileBinaryCompressed<PointType>(
        save_dir + "/" + std::to_string(frame_id) + ".pcd", *cloud) != -1);
}

class PCDReader
{
public:
    PCDReader(std::string pcd_dir, int _data_columns = 8)
    {
        data_columns = _data_columns;
        openPCDDir(pcd_dir);
    }

    PCDReader(std::string pcd_dir, bool _is_binary, int _data_columns = 8)
    {
        is_binary = _is_binary;
        data_columns = _data_columns; 
        openPCDDir(pcd_dir);
    }

    // 设置是否为二进制文件，默认false
    void setBinary(bool value){ is_binary = value; }

    bool readPointCloud(PointCloud::Ptr &cloud, int frame_id)
    {
        if (pcd_files.count(frame_id) > 0)
            return readPCD(pcd_files[frame_id], cloud);
        else
            return false;
    };

private:
    template <typename T>
    T val(const std::string &s)
    {
        std::stringstream ss;
        ss << s;
        T v;
        ss >> v;
        return v;
    }

    bool is_binary = false;
    int data_columns;
    std::map<int, std::string> pcd_files;
    inline float norm(PointType &p){ return std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);}
    
    void openPCDDir(std::string path)
    {
        assert((access(path.c_str(), 0)) == 0);
        
        std::cout << path << std::endl;

        DIR *dir = opendir(path.c_str());
        dirent *ptr;
        while ((ptr = readdir(dir)) != NULL)
        {
            if (ptr->d_name[0] == '.') continue;
            
            std::string filepath = path + "/" + std::string(ptr->d_name); 
            int frame_id = frameid(filepath);

            // 文件名添加到列表
            pcd_files[frame_id] = filepath;
        }
        closedir(dir);

        std::cout << "read pcd names finished." << std::endl;
    };

    bool readPCD(std::string file, PointCloud::Ptr &cloud)
    {
        cloud->clear();
        std::ifstream fs(file.c_str());
        if (!fs.is_open() || fs.fail())
        {
            PCL_ERROR("Could not open: %s", file.c_str());
            return false;
        }

        if(is_binary)
            return (pcl::io::loadPCDFile(file, *cloud) != -1);   

        std::string line;
        int line_cnts = 0;
        std::vector<std::string> st;
        cloud->clear();
        while (!fs.eof())
        {
            getline(fs, line);

            // Ignore lines
            if (line.empty())
                continue;

            // Tokenize the line
            boost::trim(line);
            boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);

            // x y z intensity
            if (st.size() != data_columns) continue;

            PointType p;
            p.x = val<float>(st[0]);
            p.y = val<float>(st[1]);
            p.z = val<float>(st[2]);
            
            // 25m
            if (norm(p) > 25.0) continue;
            
            p.intensity = val<float>(st[3]);;
            cloud->points.push_back(p);
        }
        fs.close();
        cloud->width = uint32_t(cloud->points.size());
        cloud->height = 1;
        cloud->is_dense = true;

        return true;
    }
};
