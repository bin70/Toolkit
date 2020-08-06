#pragma once
#include <ctime>

// 和原来的计时器一样的接口
class Timer
{
public:
    Timer() { begin(); }
    void begin()
    {
        gettimeofday(&t1, NULL);
    }

    double end()
    {
        gettimeofday(&t2, NULL);
        return TimeDiff();
    }

private:
    timeval t1, t2;

    double TimeDiff()
    {
        double t = (t2.tv_sec - t1.tv_sec) * 1000.0;      // sec to ms
        t += (t2.tv_usec - t1.tv_usec) / 1000.0;   // us to ms
        return t;
    }
};