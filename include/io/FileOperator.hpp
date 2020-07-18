#pragma once
// linux lib
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <stdio.h>
#include <common.hpp>

class FileOperator{
    typedef std::string string;
    typedef Eigen::Matrix4d Matrix;
public:
    bool makeDir(string path)
    {
        return CreateDir(path.c_str());
    }

    inline int CreateDir(const char *sPathName)
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
};