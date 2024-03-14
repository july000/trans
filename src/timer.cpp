#include "timer.hpp"

#ifdef WIN32
int gettimeofday(struct timeval *tp, void *tzp)
{
	time_t clock;
	struct tm tm;
	SYSTEMTIME wtm;
	GetLocalTime(&wtm);
	tm.tm_year = wtm.wYear - 1900;
	tm.tm_mon = wtm.wMonth - 1;
	tm.tm_mday = wtm.wDay;
	tm.tm_hour = wtm.wHour;
	tm.tm_min = wtm.wMinute;
	tm.tm_sec = wtm.wSecond;
	tm.tm_isdst = -1;
	clock = mktime(&tm);
	tp->tv_sec = clock;
	tp->tv_usec = wtm.wMilliseconds * 1000;
	return (0);
}
#endif

Timer::Timer() : mStopped(true), mToStop(false) {}

Timer::Timer(const Timer &timer)
{
	mStopped = timer.mStopped.load();
	mToStop = timer.mToStop.load();
}

Timer::~Timer()
{
	stop();
}

void Timer::start(int interval, std::function<void()> task)
{
	// do not start again
	if (!mStopped)
		return;

	// start async timer, launch thread and wait in that thread
	mStopped = false;
	std::thread([this, interval, task]()
			{
				while (!mToStop)
				{
					int64_t pre = get_cur_stamp();
					task();
					int64_t cur = get_cur_stamp();
					int64_t gap = cur - pre;
					// sleep every interval and do the task again and again until times up
					int rel_interval = interval - gap;
					if (rel_interval > 0)
						std::this_thread::sleep_for(std::chrono::milliseconds(rel_interval));
					// task();
			}

			{
			// timer be stopped, update the condition variable and wake main thread
			std::lock_guard<std::mutex> locker(mMutex);
			mStopped = true;
			mStopCondition.notify_one();
			} })
	.detach();
}

void Timer::startOnce(int delay, std::function<void()> task)
{
	std::thread([delay, task]()
			{
			std::this_thread::sleep_for(std::chrono::milliseconds(delay));
			task(); })
		.detach();
}

void Timer::stop()
{
	// do not stop again
	if (mStopped)
		return;
	if (mToStop)
		return;

	// change this bool value to make timer while loop stop
	mToStop = true;
	{
		std::unique_lock<std::mutex> locker(mMutex);
		mStopCondition.wait(locker, [this]
				{ return mStopped == true; });

		// reset the timer
		if (mStopped == true)
			mToStop = false;
	}
}

int64_t Timer::get_cur_stamp()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return tv.tv_sec * 1000 + tv.tv_usec / 1000; // milliseconds
	// return tv.tv_sec * 1e6 + tv.tv_usec; // microseconds
}
