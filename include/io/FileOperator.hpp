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
};