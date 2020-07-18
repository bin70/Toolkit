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
        if(access(output_dir.c_str(), 0) != 0)
            if(mkdir(output_dir.c_str(), 0755) == -1)
            {
                perror("makeDir error: ");
                return false;
            }
        return true;
    }

    Matrix loadMatrix(string path)
    {
        if(access(path.c_str(), 0) != 0) perror("load matrix error: ");
        std::oftream f(path);
        Matrix m;
        for(int i=0; i<4; ++i)
            for(int j=0; j<4; ++j)
                f >> m(i, j);
        return m;
    }
};