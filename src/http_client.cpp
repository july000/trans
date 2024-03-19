#include "http_client.h"

// http
long long httpclient::getSystemTime()
{
    struct timeb t;
    ftime(&t);
    return 1000 * t.time + t.millitm;
}

httpclient::httpclient(SendAndConsumerArgs &args) : mCli(args.http_access_ip.c_str(), args.http_access_port)
{
    // #ifdef CPPHTTPLIB_OPENSSL_SUPPORT
    // 	httplib::SSLClient cli("localhost", 8080);
    // 	cli.set_ca_cert_path(CA_CERT_FILE);
    // 	cli.enable_server_certificate_verification(true);
    // #else
    // std::string ip = "10.66.9.244";
    // int port = 15672;
    // std::string user = "admin";
    // std::string password = "admin";
    // httplib::Client cli(ip.c_str(), port);
    // mCli.set_basic_auth(args.http_user.c_str(), args.http_password.c_str());
    // #endif
    mContentType = args.http_content_type;
    mHDS.insert({"Content-Type", "application/json"});
    mHDS.insert({"User-Agent", "Mozilla/5.0 (Windows NT 10.0; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/51.0.2704.103 Safari/537.36"});
    // log_payload.open("log_payload.txt", std::ios::trunc); // std::ios::app
}
httpclient::httpclient(std::string content_type, std::string ip, int port) : mCli(ip.c_str(), port)
{
    mContentType = content_type;
    mHDS.insert({"Content-Type", "application/json"});
    mHDS.insert({"User-Agent", "Mozilla/5.0 (Windows NT 10.0; WOW64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/51.0.2704.103 Safari/537.36"});
}
httpclient::~httpclient()
{
    // log_payload.close();
}

void httpclient::do_post(const std::string &data, const std::string &url)
{
    auto res = mCli.Post(url.c_str(), mHDS, data.c_str(), mContentType.c_str()); // "application/www-form-urlencoded" / "text/plain"  / "application/json"

    if (res->status == 200)
    {
        // std::cout << "POST请求成功，响应内容为：" << res->body << std::endl;
    }
    else
    {
        std::cout << "POST请求失败，状态码：" << res->status << std::endl;
    }

    // log_payload << "payload sent: " << wrp_data.c_str() << "\n";
    // auto res = cli.Post("/Post", data.c_str(), "text/plain"); // end_t = getSystemTime(); // log_payload << "------------ after sent: " << ntoString(end_t).c_str() << "\n";
    // log_payload << "============ time_counter: " << ntoString(getSystemTime()).c_str() << "\n";
    // log_payload.flush();
}

void httpclient::do_put(const std::string &data, const std::string &url)
{
    auto res = mCli.Put(url.c_str(), mHDS, data.c_str(), mContentType.c_str()); // "application/www-form-urlencoded" / "text/plain" / "application/json"

    if (res->status == 200)
    {
        // std::cout << "PUT请求成功，响应内容为：" << res->body << std::endl;
    }
    else
    {
        std::cout << "PUT请求失败，状态码：" << res->status << std::endl;
    }
}
void httpclient::do_get(const std::string &url, std::string& steering_angle, std::string& steering_speed)
{
    auto res = mCli.Get(url.c_str(), mHDS); // "application/www-form-urlencoded" / "text/plain" / "application/json"

    if (res->status == 200)
    {
        // std::cout << "GET请求成功，响应内容为：" << res->body << std::endl;
        std::string str = res->body;
        str.erase(std::remove(str.begin(), str.end(), '\"' ), str.end());

        // Parse the string
        std::vector<string> result;
        std::stringstream ss(str);
        std::string token;
        while (std::getline(ss, token, ',')) {
            // Remove whitespaces and brackets from the token
            token.erase(std::remove_if(token.begin(), token.end(), [](char c){ return c == ' ' || c == '[' || c == ']'; }), token.end());
            result.push_back(token); // Convert string to integer and add to vector
        }
        steering_angle = result[0];
        steering_speed = result[1];
        // std::sscanf(res->body.c_str(), "[ %d, %d, %d ]", &num1, &num2, &num3);
        // std::cout << "angle : " << steering_angle << ", steering_speed : " << steering_speed << std::endl;
    }
    else
    {
        std::cout << "GET请求失败，状态码：" << res->status << std::endl;
    }
}
void httpclient::do_delete(const std::string& url)
{
    auto res = mCli.Delete(url.c_str(), mHDS); // "application/www-form-urlencoded" / "text/plain" / "application/json"
    if (res->status == 200)
    {
        // std::cout << "Delete请求成功，响应内容为：" << res->body << std::endl;
    }
    else
    {
        std::cout << "Delete请求失败，状态码：" << res->status << std::endl;
    }
}
