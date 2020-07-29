#pragma once
// linux lib
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <stdio.h>
#include <fstream>
#include <eigen3/Eigen/Dense>

inline void consoleProgress(int progress)
{
    static char bar[102] = {0};
    static std::string lable = "|/-\\";
    static int inprogress = -1;
    if (inprogress == progress)
        return;

    int st = progress / 2;
    for (int i = 0; i < st; i++)
        bar[i] = '#';

    bar[st] = 0;

    printf("[%-50s][%d%%][%c]\r", bar, progress, lable[progress % 4]);
    fflush(stdout);

    inprogress = progress;

    if (progress == 100)
        printf("\n");
}

inline void consoleProgress(int progress, int begin, int end)
{
    consoleProgress(100. * (progress - begin) / (end - begin));
}

class FileOperator{
    typedef std::string string;
    typedef Eigen::Matrix4d Matrix;

public:
    ///************************************************************************/
    ///* 根据文件数打印处理进度，没有打开文件则打印 0%                               */
    ///************************************************************************/
    void printProgress(int file_index)
    {
        consoleProgress(file_index, 0, file_num);
    }

    ///************************************************************************/
    ///* 递归创建文件夹，存在则不创建                                               */
    ///************************************************************************/
    bool makeDir(string path)
    {
        return CreateDir(path.c_str());
    }

    bool delDir(string path)
    {
        return DeleteDir(path.c_str());
    }
    
    ///************************************************************************/
    ///* 检查文件是否存在
    ///************************************************************************/
    bool isExist(string path)
    {
        return CheckFileExist(path.c_str());
    }

    std::vector<string> openDir(string path)
    {
        assert( isExist(path) );

        std::vector<string> filelist;

        DIR *dir = opendir(path.c_str());
        dirent *ptr;
        while ((ptr = readdir(dir)) != NULL)
        {
            if (ptr->d_name[0] == '.') continue;
            
            string filename = string(ptr->d_name);
            // 文件名添加到列表
            filelist.push_back(path + "/" + filename);
        }
        closedir(dir);

        opened_dir = true;
        file_num = filelist.size();

        return filelist;
    }

    std::string getFileName(std::string path)
    {
        std::string fullName = path.substr(path.rfind('/') + 1);
        return fullName.substr(0, fullName.find('.'));
    }

    ///***************************************
    ///* 返回一个路径的父文件夹所在的路径
    ///***************************************
    std::string getParentDir(std::string path)
    {
        int path_end = path.rfind('/');
        return path.substr(0, path_end);
    }

    Matrix loadMatrix(std::string path)
    {
        if(access(path.c_str(), 0) != 0) perror("load matrix error: ");
        std::ifstream fmatrix(path.c_str());
        Matrix m;
        for(int i=0; i<4; ++i)
            for(int j=0; j<4; ++j)
                fmatrix >> m(i, j);
        return m;
    }

private:
    bool opened_dir = false;
    int file_num = 0;

    bool CheckFileExist(const char *p)
    {
        if ((access(p, 0)) == 0)
        {
            return true;
        }
        return false;
    }

    bool CreateDir(const char *sPathName)
    {
        char DirName[256];
        strcpy(DirName, sPathName);
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

        return true;
    }

    ///************************************************************************/
    ///* 删除文件夹，不存在则不删除                                               */
    ///************************************************************************/
    int DeleteDir(const char *path)
    {
        std::string dirPath = path;
        if (access(path, 0) == 0)
        {
            std::string shell_delete_command = "rm -rf " + dirPath;
            return system(shell_delete_command.c_str());
        }
        return 0;
    }
};