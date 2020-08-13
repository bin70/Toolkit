#pragma once
#include <io/FileOperator.hpp>
#include <point_cloud/common.hpp>

class TXTReader
{
    using string = std::string;
private:
    int data_columns;
    void openDir(string path);
    float norm(PointType &p){ return std::sqrt(p.x*p.x + p.y*p.y + p.z*p.z);}
    template <typename T>
    
    T val(const std::string &s)
    {
        std::stringstream ss;
        ss << s;
        T v;
        ss >> v;
        return v;
    }
    
    std::map<string, string> files;
    FileOperator fop;

public:
    TXTReader(string txt_dir, int _data_columns = 4);
    ~TXTReader();
    bool readPointCloud(PointCloud::Ptr &cloud, string filename);
};

TXTReader::TXTReader(string txt_dir, int _data_columns)
{
    data_columns = _data_columns;
    openDir(txt_dir);
}

TXTReader::~TXTReader(){}

void TXTReader::openDir(std::string path)
{
    assert((access(path.c_str(), 0)) == 0);

    DIR *dir = opendir(path.c_str());
    dirent *ptr;
    while ((ptr = readdir(dir)) != NULL)
    {
        if (ptr->d_name[0] == '.') continue;
        
        std::string filepath = path + "/" + std::string(ptr->d_name); 
        string filename = fop.getFileName(filepath);

        // 文件名添加到列表
        files[filename] = filepath;
    }
    closedir(dir);

    std::cout << "read filenames finished." << std::endl;
}

bool TXTReader::readPointCloud(PointCloud::Ptr &cloud, std::string filename)
{
    cloud->clear();
    string file = files[filename];
    if(!fop.isExist(file)) return false;

    std::string line;
    int line_cnts = 0;
    std::vector<std::string> st;
    std::ifstream fs(file.c_str());
    while (!fs.eof())
    {
        getline(fs, line);

        // Ignore lines
        if (line.empty()) continue;

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
