#ifndef WEBSOCKET_CLIENT_H
#define WEBSOCKET_CLIENT_H

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <iostream>
#include "nlohmann/json.hpp"
#include <queue>
#include <mutex>
#include "zd/ZDAPI.h"
#include "zd/MsgStruct.h"
#include "simapi.h"
#include "mqio.h"
#include "http_client.h"

typedef websocketpp::client<websocketpp::config::asio_client> client;
typedef websocketpp::config::asio_client::message_type::ptr message_ptr;

using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

struct CompareControl {
    bool operator()(const Control_t& s1, const Control_t& s2);
};
struct CompareSignalLight
{
    bool operator()(const SignalLight_t& s1, const SignalLight_t& s2);
};

// struct CompareDrivingLever
// {
//     bool operator()(const DrivingLever_t& s1, const DrivingLever_t& s2);
// };

class websocket_client
{
public:
    websocket_client();
    ~websocket_client();

    void on_open(client *c, websocketpp::connection_hdl hdl);
    void on_message(client *c, websocketpp::connection_hdl hdl, message_ptr msg);

    void parsed_websocket_data(nlohmann::json j);
    void parse_pedal_data(Control_t& ptl_data, long long& timestamp, const nlohmann::json& signals_data);
    void parse_steering_data(Control_t& ptl_data, long long& timestamp, const nlohmann::json& signals_data);
    void parse_signal_light(long long& timestamp, const nlohmann::json& signals_data);
    void parse_wiper_data(long long& timestamp, const nlohmann::json& signals_data);
    void set_steering_scope(const char* steering_max);
    
    // void Timeout(client *c, websocketpp::connection_hdl &hdl, boost::asio::deadline_timer *pt, const boost::system::error_code &ec);
    void Timeout(boost::asio::deadline_timer *pt, const boost::system::error_code &ec);
    void ConsumeQueueData(simapi &api, SendAndConsumerArgs &args);
    void ReverseControl(httpclient& httpCli, simapi *&p_api, SendAndConsumerArgs &args);
    void ReverseControlByPCAN(simapi *&p_api, SendAndConsumerArgs &args);

    void run_websocket_client(SendAndConsumerArgs &args);

    float split_uint32(const string& phys_value_str, uint8_t shift_bits, uint32_t mask_bits);
    // float split_uint32(const string& phys_value_str);
    // void sendASC(const string &filename, string& init_steering_url, httpclient& http_cli);
    void sendASC(SendAndConsumerArgs &args);
    void sendHeartBeatASC(string& init_steering_url, httpclient& http_cli);

    static int phys_gear;
    static DrivingLever_t mDrivingLever;
    static bool mCruise_valid;
    static float mCruise_speed;
private:
    static client c;
    static httpclient  httpCli;
    static websocketpp::connection_hdl hdl;
    // static client::connection_ptr con;
    static std::mutex mMutex_hdl;
    static std::priority_queue<Control_t, std::vector<Control_t>, CompareControl> myControlQueue;
    static std::priority_queue<SignalLight_t, std::vector<SignalLight_t>, CompareSignalLight> mySignalLightQueue;
    // static std::priority_queue<DrivingLever_t, std::vector<DrivingLever_t>, CompareDrivingLever> myDrivingLeverQueue;
    // const string& left_steering_max, const string& right_steering_max
    float m_SteeringMax;
};

#endif