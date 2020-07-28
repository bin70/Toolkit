#pragma once
// linux lib
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <stdio.h>
#include <eigen3/Eigen/Dense>

class FileOperator{
    typedef std::string string;
    typedef Eigen::Matrix4d Matrix;

public:
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

    std::string getFileName(std::string path)
    {
        std::string fullName = path.substr(path.rfind('/') + 1);
        return fullName.substr(0, fullName.find('.'));
    }

    Matrix loadMatrix(string path)
    {
        if(access(path.c_str(), 0) != 0) perror("load matrix error: ");
        std::ifstream f(path);
        Matrix m;
        for(int i=0; i<4; ++i)
            for(int j=0; j<4; ++j)
                f >> m(i, j);
        return m;
    }

private:
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