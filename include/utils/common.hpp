#pragma once
#include <iostream>
#include <vector>

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
