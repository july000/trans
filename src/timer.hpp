#ifndef _TIMER_H_
#define _TIMER_H_

#include <functional>
#include <thread>
#include <atomic>
#include <memory>
#include <mutex>
#include <condition_variable>

#ifdef WIN32
#include <windows.h>
int gettimeofday(struct timeval *tp, void *tzp);
#else
#include <sys/time.h>
#endif

class Timer
{
public:
        Timer();
        Timer(const Timer &timer);
        ~Timer();

        int64_t get_cur_stamp();

        void start(int interval, std::function<void()> task);
        void startOnce(int delay, std::function<void()> task);
        void stop();

private:
        std::atomic<bool> mStopped; // timer stopped status
        std::atomic<bool> mToStop;  // timer is in stop process
        std::mutex mMutex;
        std::condition_variable mStopCondition;
};

#endif // !_TIMER_H_
