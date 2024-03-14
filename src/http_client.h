#ifndef HTTPCLIENT_H_
#define HTTPCLIENT_H_

#include "httplib.h"
#include "mqio.h"
#include <sstream>
#include <queue>

#include <sys/timeb.h>
#include <sys/types.h>

#define _USE_MATH_DEFINES
#include <math.h>

template <typename T>
std::string ntoString(T number)
{
	std::string result;
	std::ostringstream ss;
	ss << number;
	std::istringstream is(ss.str());
	is >> result;
	return result;
}

class httpclient
{
public:
	httpclient(SendAndConsumerArgs &args);
	httpclient(std::string content_type, std::string ip, int port = 80);
	~httpclient();

	void do_post(const std::string& data, const std::string& url);
	void do_put(const std::string& data, const std::string& url);
	void do_delete(const std::string& url);
	void do_get(const std::string &url, std::string& steering_angle, std::string& steering_speed);

	long long getSystemTime();

private:
	// 	std::ofstream log_payload;
	httplib::Client mCli;
	httplib::Headers mHDS;
	std::string mContentType;

public:
	static std::queue<ASC_t> steering_ini_0x200;
	static std::queue<ASC_t> steering_ini_0x210;
	static std::queue<ASC_t> steering_ini_0x53e;
	static std::queue<ASC_t> steering_ini_0x542;
	static std::queue<ASC_t> steering_ini_0x543;
	static std::queue<ASC_t> steering_ini_0x544;
	static std::queue<ASC_t> steering_ini_0x545;
};

#endif