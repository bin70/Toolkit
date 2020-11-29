#pragma once
#include <ctime>

// 和原来的计时器一样的接口
class Timer
{
public:
    Timer(std::string format = "ms") : _format(format) 
    { 
        begin(); 
    }
    
    void begin()
    {
        gettimeofday(&t1, NULL);
    }

    std::string end()
    {
        gettimeofday(&t2, NULL);
        double t = TimeDiff();
        if(_format == "ms") return std::to_string(t)+_format;
        else if(_format == "s") return std::to_string(t/1000.0)+_format;
        else
        {
            std::cout << "format of time output is only s/ms!" << std::endl;
            exit(-1);
        }   
    }

private:
    timeval t1, t2;
    std::string _format;
    double TimeDiff()
    {
        double t = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
        t += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
        return t;
    }
};