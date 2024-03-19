#include <queue>
#include "http_client.h"
#include "client.h"
#include "PID.h"

// websocket
client websocket_client::c;
websocketpp::connection_hdl websocket_client::hdl;
std::mutex websocket_client::mMutex_hdl;

DrivingLever_t websocket_client::mDrivingLever;
bool websocket_client::mCruise_valid;

std::mutex mMutex_control;
std::mutex mMutex_signal_light;
// std::mutex mMutex_mDrivingLever;
ZDAPI::WebsocketAPI zdConsumer;

std::priority_queue<Control_t, std::vector<Control_t>, CompareControl> websocket_client::myControlQueue;
std::priority_queue<SignalLight_t, std::vector<SignalLight_t>, CompareSignalLight> websocket_client::mySignalLightQueue;
// std::priority_queue<DrivingLever_t, std::vector<DrivingLever_t>, CompareDrivingLever> websocket_client::myDrivingLeverQueue;
int websocket_client::phys_gear = 0x0;
float websocket_client::mCruise_speed;

std::queue<ASC_t> httpclient::steering_ini_0x200;
std::queue<ASC_t> httpclient::steering_ini_0x210;
std::queue<ASC_t> httpclient::steering_ini_0x53e;
std::queue<ASC_t> httpclient::steering_ini_0x542;
std::queue<ASC_t> httpclient::steering_ini_0x543;
std::queue<ASC_t> httpclient::steering_ini_0x544;
std::queue<ASC_t> httpclient::steering_ini_0x545;

float roundToNearest5(float speed, bool accelerate) {
    float speed_km_h = speed * 3.6;
    int integral = (int)speed_km_h;
    int remainder = integral % 5;
    if (accelerate)
    {
        if (remainder < 3)
        {
            integral += (5 - remainder);
        }
        else
        {
            integral += (10 - remainder); 
        }
    }
    else
    {
        if (remainder <= 2)
        {
            integral -= (5 - remainder);
        }
        else
        {
            integral -= remainder;
        }
    }
    return (float)integral; 
}

double SteeringFollow(float currSteerSpd, float currSteerAngle, float targetAngle, float targetSpd, PID &posPID, PID &spdPID, float kvff)
{
    bool testSpdFollow = false;
    // float kvff = 0.8;        // 速度前馈系数

    double posPIDCalRst = 0, spdPIDCalRst = 0;
    if (!testSpdFollow)
    {
        posPIDCalRst = posPID.PID_Cal(targetAngle, currSteerAngle);
        spdPIDCalRst = spdPID.PID_Cal(posPIDCalRst + kvff * targetSpd, currSteerSpd);
        // std::cout << "--------------- SteeringFollow : " << spdPIDCalRst << std::endl; 
        std::cout << currSteerSpd << ", " << targetSpd << ", " << currSteerAngle << ", " << targetAngle << 
        ", " << posPIDCalRst << ", " << spdPIDCalRst << std::endl; 
    }
    else
    {        // Test Speed follow
        spdPIDCalRst = spdPID.PID_Cal(targetAngle, currSteerSpd);
    }
    // std::cout << "--------------- SteeringFollow2 : " << spdPIDCalRst << std::endl; 
    return spdPIDCalRst; 
    // spdPIDCalRst 最终计算出来的力
}


bool CompareControl::operator()(const Control_t &s1, const Control_t &s2)
{
    return s1.timestamp > s2.timestamp;
}

bool CompareSignalLight::operator()(const SignalLight_t &s1, const SignalLight_t &s2)
{
    return s1.timestamp > s2.timestamp;
}

// bool CompareDrivingLever::operator()(const DrivingLever_t &s1, const DrivingLever_t &s2)
// {
//     return s1.timestamp > s2.timestamp;
// }
websocket_client::websocket_client()
{
    mDrivingLever.isCanceled = false;
    mDrivingLever.isSpeedSet = false;
    mDrivingLever.isSpeedUp = false;
    mDrivingLever.isSpeedDown = false;
    mDrivingLever.isCanceled = false;
    mDrivingLever.isResumed = false;
    mCruise_valid = false;
    mCruise_speed = 0.0;
}
websocket_client::~websocket_client()
{

}

float websocket_client::split_uint32(const string &phys_value_str, uint8_t shift_bits, uint32_t mask_bits)
{
    uint32_t phys_value = std::stoul(phys_value_str.c_str());
    // std::cout << "phys_value : " << phys_value  << std::endl;
    uint32_t steering = 0.0;
    uint16_t turn = 0.0;
    float ret_steering = 0.0;
    turn = static_cast<uint16_t>((phys_value >> shift_bits) & mask_bits);
    steering = static_cast<uint32_t>(phys_value & mask_bits);
    if (turn != 0) // 0: right 1: left
    {
        // Set the sign bit
        steering -= 1;
        steering ^= mask_bits;
        ret_steering = steering * (-1.0);
        // ret_steering = phys_value - mask_bits;
    }
    else
    {
        ret_steering = steering;
    }
    return ret_steering;
}


// float websocket_client::split_uint32(const string &phys_value_str)
// {
//     uint32_t phys_value = std::stoul(phys_value_str.c_str());
//     // std::cout << "phys_value : " << phys_value  << std::endl;
//     uint16_t steering = 0.0;
//     uint16_t turn = 0.0;
//     float ret_steering = 0.0;
//     turn = static_cast<uint16_t>((phys_value >> 16) & 0xFFFF);
//     steering = static_cast<uint16_t>(phys_value & 0xFFFF);
//     if (turn != 0) // 0: right 1: left
//     {
//         // Set the sign bit
//         steering -= 1;
//         steering ^= 0xFFFF;
//         ret_steering = steering * (-1);
//     }
//     else
//     {
//         ret_steering = steering;
//     }
//     return ret_steering;
// }

void websocket_client::on_open(client *c, websocketpp::connection_hdl hdl)
{
    // std::string msg = "hello";
    // c->send(hdl, msg, websocketpp::frame::opcode::text);
    // c->get_alog().write(websocketpp::log::alevel::app, "Tx: " + msg);

    // std::lock_guard<std::mutex> locker(mMutex_hdl);
    zdConsumer.heartbeat(c, hdl);
    zdConsumer.addChannel(c, hdl);
    zdConsumer.getChannel(c, hdl);
    zdConsumer.setRule(c, hdl);
    zdConsumer.getRule(c, hdl);
}

void websocket_client::parse_pedal_data(Control_t &ptl_data, long long &timestamp, const nlohmann::json &signals_data)
{
    // std::cout << "-- start parse_pedal_data --- " << signals_data << std::endl;
    ptl_data.timestamp = timestamp;
    bool has_brake = false;
    bool has_throttle = false;

    for (const auto &signal : signals_data)
    {
        std::string signal_name = signal["name"];
        if (signal_name.compare("hwBrake") == 0)
        {
            has_brake = true;
            // std::string raw_value_str = signal["raw"];
            std::string phys_value_str = signal["phys"];
            // float raw_value = std::stof(raw_value_str.c_str());
            float phys_value = std::stof(phys_value_str.c_str());
            ptl_data.brake = phys_value >= 2.0 ? phys_value - 2.0 : phys_value - 1.0;
            // std::cout << "\t---brake--- " << phys_value << std::endl;
            // std::cout << "\t---brake--- " << ptl_data.brake << std::endl;
        }
        else if (signal_name.compare("hwThrottle") == 0)
        {
            has_throttle = true;
            // std::string raw_value_str = signal["raw"];
            std::string phys_value_str = signal["phys"];
            // float raw_value = std::stof(raw_value_str.c_str());
            float phys_value = std::stof(phys_value_str.c_str());
            ptl_data.throttle = phys_value - 8.0;
            // std::cout << "\t---throttle--- " << phys_value << std::endl;
            // std::cout << "\t---throttle--- " << ptl_data.throttle << std::endl;
        }
    }
    if (has_brake && has_throttle)
    {
        ptl_data.isInitialized = true;
    }
    // std::lock_guard<std::mutex> locker(mMutex_control);
    // myControlQueue.push(ptl_data);
}

void websocket_client::parse_steering_data(Control_t &ptl_data, long long &timestamp, const nlohmann::json &signals_data)
{
    // std::coyyput << "-- start parse_steering_data --- " << signals_data << std::endl;
    ptl_data.timestamp = timestamp;

    for (const auto &signal : signals_data)
    {
        std::string signal_name = signal["name"];
        if (signal_name.compare("hwSteeringAngle") == 0)
        {
            ptl_data.isInitialized = true;
            std::string raw_value_str = signal["raw"];
            std::string phys_value_str = signal["phys"];
            if (phys_value_str.empty())
            {
                // std::cout << "steering value is empty" << std::endl;
                continue;
            }

            float unnormalized_steering = split_uint32(phys_value_str, 31, 0xFFFFFFFF);
            ptl_data.unnormalized_steering = unnormalized_steering;
            // log_web << "[" << timestamp << "] " << ", unnormalized_steering : " << unnormalized_steering << ", phys_value_str : " << phys_value_str << std::endl;
            std::cout << "[" << timestamp << "] " << ", unnormalized_steering : " << unnormalized_steering << ", phys_value_str : " << phys_value_str << std::endl;
            if (unnormalized_steering < 0)
            {
                // 4294966460-->FCBC(64700)
                float normalized_steering = unnormalized_steering / m_SteeringMax;
                // ptl_data.steering = normalized_steering < -1.0 ? -1.0 : normalized_steering;
                ptl_data.steering = normalized_steering;

                std::cout << " left normalized_steering : " << normalized_steering << ", m_SteeringMax : " << m_SteeringMax << std::endl;
            }
            else
            {
                // (9D5F)40287
                float normalized_steering = unnormalized_steering / m_SteeringMax;
                // ptl_data.steering = normalized_steering > 1.0 ? 1.0 : normalized_steering;
                ptl_data.steering = normalized_steering;

                std::cout << " right normalized_steering : " << normalized_steering << ", m_SteeringMax : " << m_SteeringMax << std::endl;
            }
            std::cout << "-- ptl_data.steering---#  " << ptl_data.steering << std::endl;
        }
        // else if (signal_name.compare("MU_Fahrpedal") == 0){
        //     std::string raw_value_str = signal["raw"];
        //     std::string phys_value_str = signal["phys"];
        //     float raw_value = std::stof(raw_value_str.c_str());
        //     float phys_value = std::stof(phys_value_str.c_str());
        //     ptl_data.throttle = phys_value;
        //     std::cout << "-- \t throttle--- " << phys_value << std::endl;

        // }
        // else if (signal_name.compare("hwSteeringSpeed") == 0){
        //     std::string raw_value_str = signal["raw"];
        //     std::string phys_value_str = signal["phys"];
        //     // std::cout << "-- \t raw_value_str --- " << raw_value_str << std::endl;
        //     // std::cout << "-- \t phys_value_str --- " << phys_value_str<< std::endl;
        //     float unnormalized_steering_speed = split_uint32(phys_value_str, 8, 0xFF);
        //     // std::cout << "-- \t unnormalized_steering_speed--- " << unnormalized_steering_speed << std::endl;
        //     ptl_data.steering_speed = unnormalized_steering_speed;
        //     // if (unnormalized_steering_speed < 0)
        //     // {
        //     //     // 4294966460-->FCBC(64700)
        //     //     float normalized_steering = unnormalized_steering_speed / m_SteeringMax;
        //     //     ptl_data.steering = normalized_steering < -1.0 ? -1.0 : normalized_steering;

        //     //     // std::cout << " left normalized_steering : " << normalized_steering << ", m_SteeringMax : " << m_SteeringMax << std::endl;
        //     // }
        //     // else
        //     // {
        //     //     // (9D5F)40287
        //     //     float normalized_steering = unnormalized_steering_speed / m_SteeringMax;
        //     //     ptl_data.steering_speed = unnormalized_steering_speed;
        //     //     // ptl_data.steering = normalized_steering > 1.0 ? 1.0 : normalized_steering;

        //     //     // std::cout << " right normalized_steering : " << normalized_steering << ", m_SteeringMax : " << m_SteeringMax << std::endl;
        //     // }
        //     // float raw_value = std::stof(raw_value_str.c_str());
        //     // float phys_value = std::stof(phys_value_str.c_str());
        //     // ptl_data.steering_speed = phys_value;
        //     // std::cout << "-- \t hwSteeringSpeed--- " << phys_value << std::endl;

        // }
    }
}

void websocket_client::parse_signal_light(long long &timestamp, const nlohmann::json &signals_data)
{
    // std::cout << "-- start parse_signal_light --- " << signals_data << std::endl;
    SignalLight_t light_data;
    light_data.HighBeam = 0;
    light_data.RightBlinker = 0;
    light_data.LeftBlinker = 0;
    light_data.timestamp = timestamp;
    light_data.gear = 0;

    // mDrivingLever.on = false;
    // mDrivingLever.speed_ctrl = SPEED_UNKNOWN;
    mDrivingLever.timestamp = timestamp;

    for (const auto &signal : signals_data)
    {
        // std::cout << "-- start parse_signal_light  signal--- " << signal << std::endl;
        std::string signal_name = signal["name"];

        // Signal Light
        // if (signal_name.compare("BH_Fernlicht") == 0  || signal_name.compare("BH_Lichthupe") == 0){
        if (signal_name.compare("BH_Lichthupe") == 0 || signal_name.compare("BH_Fernlicht") == 0)
        {
            // std::string raw_value_str = signal["raw"];
            std::string phys_value_str = signal["phys"];
            // int raw_value = std::atoi(raw_value_str.c_str());
            int phys_value = std::atoi(phys_value_str.c_str());
            if (phys_value)
            {
                light_data.HighBeam = phys_value;
                // std::cout << "-- \t HighBeam--- " << phys_value << std::endl;
            }
        }
        else if (signal_name.compare("BH_Blinker_re") == 0)
        {
            // std::string raw_value_str = signal["raw"];
            std::string phys_value_str = signal["phys"];
            // int raw_value = std::atoi(raw_value_str.c_str());
            int phys_value = std::atoi(phys_value_str.c_str());
            light_data.RightBlinker = phys_value;
            // std::cout << "\t---RightBlinker--- " << phys_value << std::endl;
        }
        else if (signal_name.compare("BH_Blinker_li") == 0)
        {
            // std::string raw_value_str = signal["raw"];
            std::string phys_value_str = signal["phys"];
            // int raw_value = std::atoi(raw_value_str.c_str());
            int phys_value = std::atoi(phys_value_str.c_str());
            light_data.LeftBlinker = phys_value;
            // std::cout << "\t---LeftBlinker--- " << phys_value << std::endl;
        }
        else if (signal_name.compare("GE_Waehlhebel") == 0)
        {
            // std::string raw_value_str = signal["raw"];
            std::string phys_value_str = signal["phys"];
            // int raw_value = std::atoi(raw_value_str.c_str());
            int phys_value = std::atoi(phys_value_str.c_str());
            light_data.gear = phys_value;
            phys_gear = phys_value;
            // std::cout << "\t---gear--- " << phys_value << std::endl;
        }

        // Signal Lever
        else if (signal_name.compare("LS_Hauptschalter") == 0)
        {
            std::string phys_value_str = signal["phys"];
            int phys_value = std::atoi(phys_value_str.c_str());
            mDrivingLever.isCruiseOn = phys_value;
            // std::cout << "\t---isCruiseOn--- " << phys_value << std::endl;
        }
        else if (signal_name.compare("LS_Tip_Setzen") == 0)
        {
            // std::cout << "!!!!!!!!!!!!!!!!!!!!!!LS_Tip_Setzen" << std::endl;
            static int pre_phys_value = 0;
            std::string phys_value_str = signal["phys"];
            int phys_value = std::atoi(phys_value_str.c_str());
            // std::cout << "!!!!!!!!!!!!!!!!!!!!!!LS_Tip_Setzen   " << phys_value_str << "   " << phys_value << std::endl;

            if (mDrivingLever.isCruiseOn && phys_value)
            {
                if (pre_phys_value != phys_value)
                {
                    mDrivingLever.isSpeedSet = true;
                    // std::cout << "\t---isSpeedSet--- " << phys_value << ", timestamp : "<< timestamp << std::endl;
                }
                pre_phys_value = phys_value;
            }
            else
            {
                pre_phys_value = 0;
            }
            // bool isSpeedSet = false;
        }
        else if (signal_name.compare("LS_Tip_Hoch") == 0)
        {
            static int pre_phys_value = 0;
            std::string phys_value_str = signal["phys"];
            int phys_value = std::atoi(phys_value_str.c_str());

            // std::cout << "!!!!!!!!!!!!!!!!!!!!!!LS_Tip_Hoch   " << phys_value_str << "   " << phys_value << std::endl;

            if (mDrivingLever.isCruiseOn && phys_value)
            {
                if (pre_phys_value != phys_value)
                {
                    mDrivingLever.isSpeedUp = true;
                    // std::cout << "\t---isSpeedUp--- " << phys_value << ", timestamp : "<< timestamp << std::endl;
                }
                pre_phys_value = phys_value;
            }
            else
            {
                pre_phys_value = 0;
            }
            // std::cout << "\t---isSpeedUp--- " << phys_value << std::endl;
        }
        else if (signal_name.compare("LS_Tip_Runter") == 0)
        {
            static int pre_phys_value = 0;
            std::string phys_value_str = signal["phys"];
            int phys_value = std::atoi(phys_value_str.c_str());
            // std::cout << "!!!!!!!!!!!!!!!!!!!!!!LS_Tip_Runter" << phys_value_str << "   " << phys_value << std::endl;
            if (mDrivingLever.isCruiseOn && phys_value)
            {
                if (pre_phys_value != phys_value)
                {
                    mDrivingLever.isSpeedDown = true;
                    // std::cout << "\t---isSpeedDown--- " << phys_value << ", timestamp : "<< timestamp << std::endl;
                }
                pre_phys_value = phys_value;
            }
            else
            {
                pre_phys_value = 0;
            }
        }
        else if (signal_name.compare("LS_Abbrechen") == 0)
        {
            static int pre_phys_value = 0;
            std::string phys_value_str = signal["phys"];
            int phys_value = std::atoi(phys_value_str.c_str());
            // std::cout << "!!!!!!!!!!!!!!!!!!!!!!LS_Abbrechen" << phys_value_str << "   " << phys_value << std::endl;
            // isCanceled = phys_value;
            if (mDrivingLever.isCruiseOn && phys_value)
            {
                if (pre_phys_value != phys_value)
                {
                    mDrivingLever.isCanceled = true;
                    // std::cout << "\t---isCanceled--- " << phys_value << ", timestamp : "<< timestamp << std::endl;
                }
                pre_phys_value = phys_value;
            }
            else
            {
                pre_phys_value = 0;
            }
        }
        else if (signal_name.compare("LS_Tip_Wiederaufnahme") == 0)
        {
            static int pre_phys_value = 0;
            std::string phys_value_str = signal["phys"];
            int phys_value = std::atoi(phys_value_str.c_str());
            // std::cout << "!!!!!!!!!!!!!!!!!!!!!!LS_Tip_Wiederaufnahme" << phys_value_str << "   " << phys_value << std::endl;
            // isResumed = phys_value;
            if (mDrivingLever.isCruiseOn && phys_value)
            {
                if (pre_phys_value != phys_value)
                {
                    mDrivingLever.isResumed = true;
                    // std::cout << "\t---isResumed--- " << phys_value << ", timestamp : "<< timestamp << std::endl;
                }
                pre_phys_value = phys_value;
            }
            else
            {
                pre_phys_value = 0;
            }
        }
    }

    {
        std::lock_guard<std::mutex> locker(mMutex_signal_light);
        mySignalLightQueue.push(light_data);
    }

    // {
    //     std::lock_guard<std::mutex> locker(mMutex_mDrivingLever);
    //     myDrivingLeverQueue.push(mDrivingLever);
    // }
    // std::cout << "==================================== lever" << std::endl;
    // std::cout << "lever.timestamp : " << mDrivingLever.timestamp << std::endl;
}

void websocket_client::parse_wiper_data(long long &timestamp, const nlohmann::json &signals_data)
{
    // std::cout << "-- start parse_signal_light --- " << signals_data << std::endl;
    // SignalLight_t light_data;
    // light_data.timestamp = timestamp;

    for (const auto &signal : signals_data)
    {
        // std::cout << "-- start parse_signal_light  signal--- " << signal << std::endl;
        std::string signal_name = signal["name"];
        // if (signal_name.compare("BH_Fernlicht") == 0  || signal_name.compare("BH_Lichthupe") == 0){
        if (signal_name.compare("WH_Intervallstufen") == 0)
        {
            // std::string raw_value_str = signal["raw"];
            std::string phys_value_str = signal["phys"];
            // int raw_value = std::atoi(raw_value_str.c_str());
            int phys_value = std::atoi(phys_value_str.c_str());
        }
        else if (signal_name.compare("WH_Heckwaschen") == 0)
        {
            // std::string raw_value_str = signal["raw"];
            std::string phys_value_str = signal["phys"];
            // int raw_value = std::atoi(raw_value_str.c_str());
            int phys_value = std::atoi(phys_value_str.c_str());
            // light_data.RightBlinker = phys_value;
            // std::cout << "\t---RightBlinker--- " << phys_value << std::endl;
        }
        else if (signal_name.compare("WH_Heckintervall") == 0)
        {
            // std::string raw_value_str = signal["raw"];
            std::string phys_value_str = signal["phys"];
            // int raw_value = std::atoi(raw_value_str.c_str());
            int phys_value = std::atoi(phys_value_str.c_str());
            // light_data.LeftBlinker = phys_value;
            // std::cout << "\t---LeftBlinker--- " << phys_value << std::endl;
        }
        else if (signal_name.compare("WH_Frontwaschen") == 0)
        {
            // std::string raw_value_str = signal["raw"];
            std::string phys_value_str = signal["phys"];
            // int raw_value = std::atoi(raw_value_str.c_str());
            int phys_value = std::atoi(phys_value_str.c_str());
            // light_data.gear = phys_value;
            // std::cout << "\t---gear--- " << phys_value << std::endl;
        }
        else if (signal_name.compare("WH_WischerStufe1") == 0)
        {
            // std::string raw_value_str = signal["raw"];
            std::string phys_value_str = signal["phys"];
            // int raw_value = std::atoi(raw_value_str.c_str());
            int phys_value = std::atoi(phys_value_str.c_str());
            // light_data.gear = phys_value;
            // std::cout << "\t---gear--- " << phys_value << std::endl;
        }
        else if (signal_name.compare("WH_Intervall") == 0)
        {
            // std::string raw_value_str = signal["raw"];
            std::string phys_value_str = signal["phys"];
            // int raw_value = std::atoi(raw_value_str.c_str());
            int phys_value = std::atoi(phys_value_str.c_str());
            // light_data.gear = phys_value;
            // std::cout << "\t---gear--- " << phys_value << std::endl;
        }
        else if (signal_name.compare("WH_Tipwischen") == 0)
        {
            // std::string raw_value_str = signal["raw"];
            std::string phys_value_str = signal["phys"];
            // int raw_value = std::atoi(raw_value_str.c_str());
            int phys_value = std::atoi(phys_value_str.c_str());
            // light_data.gear = phys_value;
            // std::cout << "\t---gear--- " << phys_value << std::endl;
        }
    }
    // std::lock_guard<std::mutex> locker(mMutex_signal_light);
    // mySignalLightQueue.push(light_data);
}

void websocket_client::parsed_websocket_data(nlohmann::json j)
{
    // std::cout << "-- start parse websocket data --- " << j <<std::endl;
    std::string opcode = j["opcode"];
    if (opcode.compare("subscribe") != 0)
    {
        return;
    }
    Control_t ptl_data;
    ptl_data.gear = 0;
    ptl_data.throttle = 0.0;
    ptl_data.brake = 0.0;
    ptl_data.steering = 0.0;
    ptl_data.timestamp = 0;
    ptl_data.isInitialized = false;

    for (const auto &data : j["data"])
    {
        long long timestamp = data["timestamp"];
        // std::cout << " timestamp : "<< timestamp << std::endl;

        // check if 'id' is 256 or 257
        int id = data["id"];
        // std::cout << " ID : "<< id << std::endl;

        if (id != 529 && id != 257 && id != 1543 && id != 260)
        {
            // 529:steering, 257:signal light, 1543 pedal, 260:WischerStufe;
            continue;
        }

        // check if 'parsed' field exists
        if (data.find("parsed") == data.end())
        {
            std::cout << "--- Missing 'parsed' field --- " << std::endl;
            continue;
        }
        // check if 'signals' field exists
        if (data["parsed"].find("signals") == data["parsed"].end())
        {
            std::cout << "--- Missing 'signals' field --- " << std::endl;
            continue;
        }

        switch (id)
        {
        case 529:
            parse_steering_data(ptl_data, timestamp, data["parsed"]["signals"]);
            break;
        case 257:
            parse_signal_light(timestamp, data["parsed"]["signals"]);
            break;
        case 1543:
            // std::cout << "------ begin parse 1543 data " <<std::endl;
            parse_pedal_data(ptl_data, timestamp, data["parsed"]["signals"]);
            break;
        case 260:
            parse_wiper_data(timestamp, data["parsed"]["signals"]);
            break;
        default:;
        }
    }
    if (ptl_data.isInitialized)
    {
        std::lock_guard<std::mutex> locker(mMutex_control);
        myControlQueue.push(ptl_data);
    }
}

void websocket_client::set_steering_scope(const char *steering_max)
{
    m_SteeringMax = std::stof(steering_max);
}

void websocket_client::on_message(client *c, websocketpp::connection_hdl hdl, message_ptr msg)
{
    // std::cout << "on_message called with hdl: " << hdl.lock().get() << std::endl
    //     << "message: " << msg->get_payload() << std::endl;
    // return;
    // std::cout << "typeid(msg->get_payload())" <<typeid(msg->get_payload()).name()<<std::endl;
    nlohmann::json j = nlohmann::json::parse(msg->get_payload().c_str());
    // std::cout << "typeid(msg->get_payload())" <<typeid(j).name()<<std::endl;

    // printf("==================================start parse websocket data\n");
    parsed_websocket_data(j);
    // printf("==================================finish parse websocket data\n");

    // websocketpp::lib::error_code ec;
    // c->send(hdl, msg->get_payload(), msg->get_opcode(), ec);
    // if (ec)
    // {
    //     std::cout << "Echo failed because " << ec.message() << std::endl;
    // }
}

// 定时器回调函数
//  void websocket_client::Timeout(client *c, websocketpp::connection_hdl &hdl, boost::asio::deadline_timer *pt, const boost::system::error_code &ec)
void websocket_client::Timeout(boost::asio::deadline_timer *pt, const boost::system::error_code &ec)
{
    {
        std::lock_guard<std::mutex> locker(mMutex_hdl);
        zdConsumer.heartbeat(&c, hdl);
    }

    // std::string test = "{\"opcode\":\"heartbeat\",\"requestId\":" + std::to_string(ZDBOX::WebsocketAPI.) + "}";
    // c->send(hdl, test, websocketpp::frame::opcode::text);
    // reqID++;
    // std::cout << "Send" << std::endl;

    // if(ec)
    // {
    //     std::cout << "timer is cancel " << std::endl;
    //     return;
    // }
    // static int count = 0;
    // c->send(hdl, "hello", websocketpp::frame::opcode::text);
    // count++;
    // if(count > 5)//定时器触发五次后关闭连接
    // {
    //     c->close(hdl, websocketpp::close::status::normal, "");
    //     return;
    // }
    pt->expires_at(pt->expires_at() + boost::posix_time::seconds(5));
    // pt->async_wait(std::bind(&websocket_client::Timeout, this, c, hdl, pt, ::_1));
    pt->async_wait(std::bind(&websocket_client::Timeout, this, pt, ::_1));
}

long long get_milliseconds()
{
    auto now = std::chrono::system_clock::now();
    // auto now_time_t = std::chrono::system_clock::to_time_t(now);
    // std::time_t now_time = now_time_t;
    // std::tm *local_time = std::localtime(&now_time);
    long long milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
    // std::cout << "Milliseconds since epoch: " << milliseconds << std::endl;  
    return milliseconds;
} 

bool runable = false;

void sendASC_cb(std::queue<ASC_t> &steering_ini_q, string& init_steering_url, httpclient& http_cli, std::ofstream& log)
{
    // long long pre_timestamp = 0;
    // long long pre_systime = 0;
    long long base_systime = 0;

    while (!steering_ini_q.empty())
    {
        if (!runable)
        {
            printf("-------------------------------------------------\n");
            continue;
        }
        printf("--------------------------start send-------------------------\n");

        ASC_t asc = steering_ini_q.front();
        steering_ini_q.pop();

        nlohmann::json data1 = {
            {"id", ""},
            {"payload", ""}, // 先将 value 设置为空字符串
            {"isExtended", false},
            {"isCANFD", false}
        };

        // 将 record.data 转换为字符串
        std::ostringstream oss;
        for (const auto& value : asc.data) {
            oss << std::hex << std::setw(2) << std::setfill('0') << value << " ";
        }

        // 将字符串值设置为 JSON 对象的 payload 键的值
        std::string dataString = oss.str();
        dataString = dataString.substr(0, dataString.size() - 1);  // 去掉最后的空格
        data1["payload"] = dataString; 

        std::ostringstream id_str;
        id_str << "0x" << std::setw(2) << std::setfill('0') << std::hex << asc.id;
        std::string idString = id_str.str();
        data1["id"] = idString;

        // 将 JSON 对象转换为字符串
        std::string jsonData = data1.dump();

        // long long interval = asc.timestamp - pre_timestamp;
        long long cur = get_milliseconds();
        if (!base_systime)
        {
            
            base_systime = cur;
            std::cout << "------ base_systime : " << base_systime << std::endl;
        }

        while ((cur - base_systime) < asc.timestamp)
        {
            cur = get_milliseconds();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        // std::cout << "[" << asc.timestamp << "/" << cur - base_systime << "]" << std::endl;// jsonData : " << jsonData << std::endl;
        log << "[" << asc.timestamp << "/" << cur - base_systime << "]" << std::endl;// jsonData : " << jsonData << std::endl;
        http_cli.do_post(jsonData, init_steering_url);
    }
}

// void websocket_client::sendASC(const string& filename, string& init_steering_url, httpclient& http_cli)
void websocket_client::sendASC(SendAndConsumerArgs &args)
{
    httpclient http_cli_0x200(args.http_content_type, args.http_access_ip);
    httpclient http_cli_0x210(args.http_content_type, args.http_access_ip);
    httpclient http_cli_0x53e(args.http_content_type, args.http_access_ip);
    httpclient http_cli_0x542(args.http_content_type, args.http_access_ip);
    httpclient http_cli_0x543(args.http_content_type, args.http_access_ip);
    httpclient http_cli_0x544(args.http_content_type, args.http_access_ip);
    httpclient http_cli_0x545(args.http_content_type, args.http_access_ip);

    std::ifstream file(args.asc_filename);

    if (!file.is_open()) {
        std::cout << "Failed to open file\n";
        return;
    }
   
    std::string line;
    int lineCount = 0;
    while (std::getline(file, line) && lineCount < 2)
    {
        lineCount++;
    }

    while (std::getline(file, line)) 
    {
        ASC_t asc_record;
        std::istringstream iss(line);
        iss >> asc_record.timestamp;
        asc_record.timestamp *= 1000;
        iss >> asc_record.network;
        iss >> std::hex >> asc_record.id;
        iss >> asc_record.direction;
        iss >> asc_record.state;
        iss >> asc_record.dlc;
        
        // 根据dlc大小读取数据
        for (int i = 0; i < asc_record.dlc; i++) {
            int val;
            iss >> std::hex >> val;
            asc_record.data.push_back(val);
        }
        // records.push_back(record);
        switch(asc_record.id)
        {
            case 0x200:
                // printf("================== 0x200\n");
                httpclient::steering_ini_0x200.push(asc_record);
                break;
            case 0x210:
                // printf("================== 0x210\n");
                httpclient::steering_ini_0x210.push(asc_record);
                break;
            case 0x53e:
                // printf("================== 0x53e\n");
                httpclient::steering_ini_0x53e.push(asc_record);
                break;
            case 0x542:
                // printf("================== 0x542\n");
                httpclient::steering_ini_0x542.push(asc_record);
                break;
            case 0x543:
                // printf("================== 0x543\n");
                httpclient::steering_ini_0x543.push(asc_record);
                break;
            case 0x544:
                // printf("================== 0x544\n");
                httpclient::steering_ini_0x544.push(asc_record);
                break;
            case 0x545:
                // printf("================== 0x545\n");
                httpclient::steering_ini_0x545.push(asc_record);
                break;
            default:;
        }
    }

    printf("!!!!!!!!!!!!!!!!!!!!!!!!!! read file finished\n");

    std::ofstream log_0x200, log_0x210, log_0x53e, log_0x542, log_0x543, log_0x544, log_0x545;
    log_0x200.open("log_0x200.txt", std::ios::out | std::ios::trunc);
    log_0x210.open("log_0x210.txt", std::ios::out | std::ios::trunc);
    log_0x53e.open("log_0x53e.txt", std::ios::out | std::ios::trunc);
    log_0x542.open("log_0x542.txt", std::ios::out | std::ios::trunc);
    log_0x543.open("log_0x543.txt", std::ios::out | std::ios::trunc);
    log_0x544.open("log_0x544.txt", std::ios::out | std::ios::trunc);
    log_0x545.open("log_0x545.txt", std::ios::out | std::ios::trunc);

    std::thread thread_0x200 = std::thread(std::bind(&sendASC_cb, std::ref(httpclient::steering_ini_0x200), std::ref(args.http_init_steering_url), std::ref(http_cli_0x200), std::ref(log_0x200)));
    std::thread thread_0x210 = std::thread(std::bind(&sendASC_cb, std::ref(httpclient::steering_ini_0x210), std::ref(args.http_init_steering_url), std::ref(http_cli_0x210), std::ref(log_0x210)));
    std::thread thread_0x53e = std::thread(std::bind(&sendASC_cb, std::ref(httpclient::steering_ini_0x53e), std::ref(args.http_init_steering_url), std::ref(http_cli_0x53e), std::ref(log_0x53e)));
    std::thread thread_0x542 = std::thread(std::bind(&sendASC_cb, std::ref(httpclient::steering_ini_0x542), std::ref(args.http_init_steering_url), std::ref(http_cli_0x542), std::ref(log_0x542)));
    std::thread thread_0x543 = std::thread(std::bind(&sendASC_cb, std::ref(httpclient::steering_ini_0x543), std::ref(args.http_init_steering_url), std::ref(http_cli_0x543), std::ref(log_0x543)));
    std::thread thread_0x544 = std::thread(std::bind(&sendASC_cb, std::ref(httpclient::steering_ini_0x544), std::ref(args.http_init_steering_url), std::ref(http_cli_0x544), std::ref(log_0x544)));
    std::thread thread_0x545 = std::thread(std::bind(&sendASC_cb, std::ref(httpclient::steering_ini_0x545), std::ref(args.http_init_steering_url), std::ref(http_cli_0x545), std::ref(log_0x545)));

    std::this_thread::sleep_for(std::chrono::seconds(2));
    runable = true;
    thread_0x200.join();
    thread_0x210.join();
    thread_0x53e.join();
    thread_0x542.join();
    thread_0x543.join();
    thread_0x544.join();
    thread_0x545.join();
    file.close();
    log_0x200.close();
    log_0x210.close();
    log_0x53e.close();
    log_0x542.close();
    log_0x543.close();
    log_0x544.close();
    log_0x545.close();
}

void websocket_client::sendHeartBeatASC(string& init_steering_url, httpclient& http_cli)
{
    std::vector<int> init_IDs = {0x200, 0x210, 0x53E, 0x543, 0x544, 0x542, 0x545};
    while (true)
    {
        nlohmann::json data;
        for (const int& id : init_IDs)
        {
            switch (id)
            {
                case 0x200:
                    data["id"] = id;
                    data["payload"] = "14 00 1C 02 DD 00 14 28";// 44 00 00 00 00 00 64 64
                    std::this_thread::sleep_for(std::chrono::seconds(9));
                    break;
                case 0x210:
                    data["id"] = id;
                    data["payload"] = "14 6C 00 00 1C 02 DD 00";// 44 68 00 00 1C 02 DD 00 //18 4C 20 00 1C 02 DD 00
                    std::this_thread::sleep_for(std::chrono::seconds(9));
                    break;
                case 0x53E:
                    data["id"] = id;
                    data["payload"] = "00 00";
                    std::this_thread::sleep_for(std::chrono::seconds(8));
                    break;
                case 0x543:
                    data["id"] = id;
                    data["payload"] = "20 00 00 00 00 00 00 00";
                    std::this_thread::sleep_for(std::chrono::seconds(10));
                    break;
                case 0x544:
                    data["id"] = id;
                    data["payload"] = "20 00 00 00 00 00 00 00";
                    std::this_thread::sleep_for(std::chrono::seconds(10));
                    break;
                case 0x542:
                    data["id"] = id;
                    data["payload"] = "20 00 00 00 00 00 00 00";
                    std::this_thread::sleep_for(std::chrono::seconds(10));
                    break;
                case 0x545:
                    data["id"] = id;
                    data["payload"] = "00 00 00 00 00 00 00 00";
                    std::this_thread::sleep_for(std::chrono::seconds(10));
                    break;
                default:;
            }
            data["isExtended"] = false;
            data["isCANFD"] = false;
        }
        std::string hb = data.dump();
        http_cli.do_post(hb, init_steering_url);
    }
}




void websocket_client::ConsumeQueueData(simapi &api, SendAndConsumerArgs &args)
{
// bool once_cruise_on = true;
bool cruise_on_initial = true;
bool cruise_canceled = false;

while (true)
{
    if (api.SimAPI_GetControlMode())
    {
        Control_t ctrl;
        SignalLight_t light;
        // static DrivingLever_t lever = {false, SPEED_UNKNOWN, 0};

        bool has_ctrl = false;
        bool has_light = false;
        // bool has_lever = false;
        if (!myControlQueue.empty())
        {
            // std::cout << "consume control data" << std::endl;
            std::lock_guard<std::mutex> locker(mMutex_control);
            ctrl = myControlQueue.top();
            myControlQueue.pop();
            has_ctrl = true;
        }
        if (!mySignalLightQueue.empty())
        {
            std::lock_guard<std::mutex> locker(mMutex_signal_light);
            light = mySignalLightQueue.top();
            mySignalLightQueue.pop();
            has_light = true;
        }
        if (has_ctrl)
        {
            if (mDrivingLever.isCruiseOn)
            {
                if (ctrl.brake || ctrl.throttle || mDrivingLever.isCanceled)
                {
                    cruise_canceled = true;
                    mDrivingLever.isCanceled = false;
                }
                if (mDrivingLever.isSpeedSet)
                {
                    cruise_on_initial = false;
                    cruise_canceled = false;
                    mDrivingLever.isSpeedSet = false;
                    std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
                    if (api.SimAPI_GPS(pGps.get()))
                    {
                        std::cout << "pGps->velX: " << pGps->velX << std::endl;
                        std::cout << "pGps->velY: " << pGps->velY << std::endl;
                        std::cout << "pGps->velZ: " << pGps->velZ << std::endl;
                        mCruise_speed = sqrt(pGps->velX * pGps->velX + pGps->velY * pGps->velY + pGps->velZ * pGps->velZ);
                    }
                }

                if (mDrivingLever.isResumed)
                {
                    cruise_canceled = false;
                    mDrivingLever.isResumed = false;
                }

                if (cruise_on_initial || cruise_canceled)
                {
                    mCruise_valid = false;
                    ctrl.gear = light.gear;
                    // std::cout << "--------------------------------gear: " << ctrl.gear << std::endl;
                    // std::cout << "--------------------------------throttle: " << ctrl.throttle << std::endl;
                    // std::cout << "--------------------------------brake: " << ctrl.brake << std::endl;
                    api.SimAPI_SetDrive(ctrl, simapi::Control_Mode_Percent);
                    continue;
                }

                if (mDrivingLever.isSpeedUp)
                {
                    mDrivingLever.isSpeedUp = false;
                    mCruise_speed = roundToNearest5(mCruise_speed, true)/3.6;
                }
                if (mDrivingLever.isSpeedDown)
                {
                    mDrivingLever.isSpeedDown = false;
                    mCruise_speed = roundToNearest5(mCruise_speed, false)/3.6; //std::round(mCruise_speed - 5.0 / 3.6);
                }

                mCruise_valid = true;
                ctrl.timestamp = mDrivingLever.timestamp;
                ctrl.throttle = mCruise_speed;
                std::cout << "lever.timestamp : " << mDrivingLever.timestamp << std::endl;
                std::cout << "cruise_speed: " << mCruise_speed << std::endl;
                std::cout << "ctrl.throttle: " << ctrl.throttle << std::endl;
                api.SimAPI_SetDrive(ctrl, simapi::Control_Mode_Speed);
            }
            else
            {
                if (!mDrivingLever.isCruiseOn)
                {
                    mCruise_speed = 0.0;
                }
                cruise_on_initial = true;
                cruise_canceled = false;
                mCruise_valid = false;

                // std::cout << "==================================== has_ctrl" << std::endl;
                // std::cout << "ctrl.timestamp : " << ctrl.timestamp << std::endl;
                // std::cout << "ctrl.steering : " << ctrl.steering << std::endl;
                // std::cout << "ctrl.throttle : " << ctrl.throttle << std::endl;
                // std::cout << "ctrl.brake : " << ctrl.brake << std::endl;
                // std::cout << "ctrl.gear : " << ctrl.gear << std::endl;

                ctrl.gear = light.gear;
                api.SimAPI_SetDrive(ctrl, simapi::Control_Mode_Percent);
            }
        }
        if (has_light)
        {
            api.SimAPI_SetSignalLights(light);
        }
    }
    std::this_thread::yield();
}
}

// ------------ debug
void sendMessage(websocketpp::client<websocketpp::config::asio_client> &c, websocketpp::connection_hdl &hdl, const std::string &message)
{
    while (true)
    {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        c.send(hdl, message, websocketpp::frame::opcode::text);
    }
}

void websocket_client::run_websocket_client(SendAndConsumerArgs &args)
{
    // c.set_access_channels(websocketpp::log::alevel::all);
    // c.clear_access_channels(websocketpp::log::alevel::frame_payload);
    // c.clear_access_channels(websocketpp::log::alevel::frame_header);

    // 初始化 ASIO
    c.init_asio();
    // printf("==================================init asio\n");

    // 注册消息回调
    c.set_message_handler(std::bind(&websocket_client::on_message, this, &c, ::_1, ::_2));
    c.set_open_handler(std::bind(&websocket_client::on_open, this, &c, _1));
    // printf("==================================on open\n");

    websocketpp::lib::error_code ec;
    client::connection_ptr con = c.get_connection(args.websocket_client_uri, ec);
    con->add_subprotocol("janus-protocol");
    if (ec)
    {
        std::cout << "could not create connection because: " << ec.message() << std::endl;
        return;
    }
    // websocketpp::connection_hdl hdl = con->get_handle();

    c.connect(con);
    hdl = con->get_handle();

    // ------------ debug
    // std::thread websocket_send = std::thread(std::bind(&sendMessage, std::ref(c), std::ref(hdl), "Hello, WebSocket!"));
    // websocket_send.detach();

    boost::asio::deadline_timer t(c.get_io_service(), boost::posix_time::seconds(5)); // 设置一个5s超时的定时器
    // t.async_wait(std::bind(&websocket_client::Timeout, this, &c, hdl, &t, ::_1));
    t.async_wait(std::bind(&websocket_client::Timeout, this, &t, ::_1));

    c.run();
}

void websocket_client::ReverseControl(httpclient& httpCli, simapi *&p_api, SendAndConsumerArgs &args)
{
    static PID posPID, spdPID;
    posPID.SetPID(100, 0, 0);
    spdPID.SetPID(1, 0.0, 0);

    // if (Producer::m_case_type != 2)
    std::cout << "case type : " << args.simone_case_type << std::endl;
    if (args.simone_case_type != 2)
    {
        return;
    }

    ZDAPI::BoxAPI zdBoxAPI = ZDAPI::BoxAPI(args);
    zdBoxAPI.deleteSimulation(httpCli);
    zdBoxAPI.createSimulation(httpCli);
    zdBoxAPI.activateMsgID(httpCli);



    zdBoxAPI.steeringQuitError(httpCli);
    
    httpCli.do_post("", args.http_box_start_url);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    zdBoxAPI.steeringReady(httpCli);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    zdBoxAPI.steeringOn(httpCli);
    std::this_thread::sleep_for(std::chrono::seconds(8));



    nlohmann::json signal_data_512_ready_on = {
        {"ControlWord", "4"},
        {"ModeOfOperation", "1"},
        {"AuxiliaryFunctions", "0"},
        {"EndStopPosition", "540"},
        {"PositionOffset", "220"},
        {"TorqueLimitation", "80"},
        {"PeakTorqueLimitation", "80"}
    };
    nlohmann::json j_ready_on = signal_data_512_ready_on;
    std::string modify_data_512_url = zdBoxAPI.modify_url(args.http_modify_data_url, 512);
    httpCli.do_put(j_ready_on.dump(), modify_data_512_url);
    zdBoxAPI.steeringIniParam(httpCli);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    float target_angle = 0.0;
    float target_angle_pre = 0.0;
    long long timestamp_pre = 0;
    long long timestamp = 0;
    int frame_pre = 0;
    int frame_cur = 0;
    int i_tick=0;
    bool b_tick=false;
    double first=0;

    std::ofstream log_sim, log_web;
    log_sim.open("log_sim.txt", std::ios::out | std::ios::trunc);
    log_sim << std::fixed << std::setprecision(8);
    log_web.open("log_web.txt", std::ios::out | std::ios::trunc);
    log_web << std::fixed << std::setprecision(10);

    while (true)
    {
        if (!myControlQueue.empty())
        {
            Control_t ctrl;
            std::lock_guard<std::mutex> locker(mMutex_control);
            ctrl = myControlQueue.top();
            // std::cout<< ctrl.steering<<std::endl;
            myControlQueue.pop();
            // std::cout<<ctrl.timestamp << ", " << ctrl.unnormalized_steering << std::endl;
            log_web << ctrl.timestamp << ", " << ctrl.unnormalized_steering << std::endl;
        }

        if (!mySignalLightQueue.empty())
        {
            std::lock_guard<std::mutex> locker(mMutex_signal_light);
            mySignalLightQueue = std::priority_queue<SignalLight_t, std::vector<SignalLight_t>, CompareSignalLight>();
        }
        if (!p_api)
        {
            std::cout << "---------------------------------- error " << std::endl;
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
        if (p_api->SimAPI_GPS(pGps.get()))
        {   
            frame_cur = pGps->frame;
            timestamp = pGps->timestamp;
            target_angle = pGps->steering;
        }
        if (timestamp_pre == 0)
        {
            frame_pre = frame_cur;
            timestamp_pre = timestamp;
            //target_angle_pre = target_angle;
            std::cout << "----------- timestape_pre == 0" << std::endl;
            continue;
        }
        if (timestamp_pre == timestamp)
        {
            continue;
        }
        timestamp_pre = timestamp;
        //target_angle_pre = target_angle;
        
        // int spdPIDRet;
        //target_angle=target_angle/3;
        // i_tick++;
        // double angle=std::sin(i_tick*2*M_PI/400)*100;
        double spdPIDRet=target_angle/3.0*30;
        std::vector<SIGNAL_t> signal_data_513_steering_data = {
                //{"DesiredTorque", "400", "0.4"},
                {"DesiredTorque", std::to_string(int(spdPIDRet))},
                {"Friction", "0"},
                {"Damping", "5"},
                {"SpringStiffness", "30"}
        };
        log_sim << timestamp << ", " << target_angle << std::endl;
        // std::cout << "timestamp : " << timestamp << ", angle steering : " << target_angle << std::endl;
        nlohmann::json j_steering_data = signal_data_513_steering_data;
        // std::cout << "-------------- signal_data_513_steering_data : " << j_steering_data.dump() << std::endl;
        httpCli.do_put(j_steering_data.dump(), args.http_modify_data_url);
        std::cout << "================================ steering data" << std::endl;
        
        std::this_thread::yield();
    }

}