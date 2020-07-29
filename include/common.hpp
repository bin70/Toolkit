#pragma once

#ifndef COMMON_H
#define COMMON_H
//----------system operation----------
#define linux 1
#ifdef linux
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#endif
#ifdef WIN32
#include <io.h>
#include <direct.h>
#endif

//-----------std headers-------------------------
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <cmath>
#include <stdlib.h>
#include <string.h>
// eigen
#include <Eigen/Dense>

#define DEBUG 1

// 画一条固定长度的分割线(强迫症福利)
inline void pLine()
{
    std::cout << "============================================================================" << std::endl;
}

// 打印参数列表，方便查错
inline void printParam(const std::string &header, 
    std::vector<std::pair<std::string, std::string>> &param)
{
    pLine();
    std::cout << "\t\t\t" << header << std::endl;
    pLine();

    for(int i=0; i<param.size(); ++i)
        std::cout << "\t" << param[i].first << " = " << param[i].second << std::endl;
    
    pLine();
}

#endif