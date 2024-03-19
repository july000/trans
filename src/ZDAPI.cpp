#include "zd/ZDAPI.h"
// #include "zd/MsgStruct.h"
// #include "websocketpp/config/asio_no_tls_client.hpp"
// #include "websocketpp/client.hpp"


namespace ZDAPI
{
    void WebsocketAPI::heartbeat(client *c, websocketpp::connection_hdl hdl)
    {
        std::string test;
        {
            std::lock_guard<std::mutex> locker(mMutex_Id);
            test = "{\"opcode\":\"heartbeat\",\"requestId\":" + std::to_string(reqID) + "}";
            reqID++;
        }
        c->send(hdl, test, websocketpp::frame::opcode::text);
        // std::cout << "heartbeat req sent!" << std::endl;

        // std::cout << "Receiving..." << std::endl;
        // pt->expires_at(pt->expires_at() + boost::posix_time::seconds(5));
        // pt->async_wait(bind(heartbeat, c, hdl, pt, ::_1));
        // websocketpp::lib::error_code ec;
        // client::message_ptr msg = c.receive(hdl, ec);
        // if (ec) {
        //     std::cout << "Received Error: " << ec.message() << std::endl;
        // } else {
        //     std::cout << "Received: " << msg->get_payload() << std::endl;
        // }
    }

    void WebsocketAPI::addChannel(client *c, websocketpp::connection_hdl hdl)
    {
        std::string test;
        {
            std::lock_guard<std::mutex> locker(mMutex_Id);
            test = "{\"opcode\":\"addChannel\",\"requestId\":" + std::to_string(reqID) + ",\"data\":{\"channel\":\"ipdu.can.*\",\"codec\":[\"can_test\"]}}";
            reqID++;
        }
        c->send(hdl, test, websocketpp::frame::opcode::text);
        // std::cout << "addChannel req sent!" << std::endl;

        // std::cout << "Receiving..." << std::endl;
        // websocketpp::lib::error_code ec;
        // client::message_ptr msg = c.receive(hdl, ec);
        // if (ec) {
        //     std::cout << "Received Error: " << ec.message() << std::endl;
        // } else {
        //     std::cout << "Received: " << msg->get_payload() << std::endl;
        // }
    }

    void WebsocketAPI::getChannel(client *c, websocketpp::connection_hdl hdl)
    {
        std::string test;
        {
            std::lock_guard<std::mutex> locker(mMutex_Id);
            test = "{\"opcode\":\"getChannel\",\"requestId\":" + std::to_string(reqID) + "}";
            reqID++;
        }
        c->send(hdl, test, websocketpp::frame::opcode::text);
        // std::cout << "getChannel req sent!" << std::endl;

        // std::cout << "Receiving..." << std::endl;
        // websocketpp::lib::error_code ec;
        // client::message_ptr msg = c.receive(hdl, ec);
        // if (ec) {
        //     std::cout << "Received Error: " << ec.message() << std::endl;
        // } else {
        //     std::cout << "Received: " << msg->get_payload() << std::endl;
        // }
    }

    void WebsocketAPI::setRule(client *c, websocketpp::connection_hdl hdl)
    {
        std::string test;
        {
            std::lock_guard<std::mutex> locker(mMutex_Id);
            test = "{\"opcode\":\"setRule\",\"requestId\":" + std::to_string(reqID) + ",\"data\":{\"rule\":\"()=>true\"}}";
            reqID++;
        }
        c->send(hdl, test, websocketpp::frame::opcode::text);
        // std::cout << "setRule req sent!" << std::endl;

        // std::cout << "Receiving..." << std::endl;
        // websocketpp::lib::error_code ec;
        // client::message_ptr msg = c.receive(hdl, ec);
        // if (ec) {
        //     std::cout << "Received Error: " << ec.message() << std::endl;
        // } else {
        //     std::cout << "Received: " << msg->get_payload() << std::endl;
        // }
    }

    void WebsocketAPI::getRule(client *c, websocketpp::connection_hdl hdl)
    {
        std::string test;
        {
            std::lock_guard<std::mutex> locker(mMutex_Id);
            test = "{\"opcode\":\"getRule\",\"requestId\":" + std::to_string(reqID) + "}";
            reqID++;
        }
        c->send(hdl, test, websocketpp::frame::opcode::text);
        // std::cout << "getRule req sent!" << std::endl;

        // std::cout << "Receiving..." << std::endl;
        // websocketpp::lib::error_code ec;
        // client::message_ptr msg;
        // if (ec) {
        //     std::cout << "Received Error: " << ec.message() << std::endl;
        // } else {
        //     std::cout << "Received: " << msg->get_message() << std::endl;
        // }
    }

    void WebsocketAPI::setControl(client *c, websocketpp::connection_hdl hdl, std::string& ctl)
    {
        {
            std::lock_guard<std::mutex> locker(mMutex_Id);
            // std::string test = "{\"opcode\":\"getRule\",\"requestId\":" + std::to_string(reqID) + "}";
            reqID++;
        }
        c->send(hdl, ctl, websocketpp::frame::opcode::text);
        // std::cout << "getRule req sent!" << std::endl;
    }

    BoxAPI::BoxAPI(SendAndConsumerArgs& args)
    {
        box_start_url = args.http_box_start_url;
        box_stop_url = args.http_box_stop_url;
        box_activate_url = args.http_box_activate_url;
        modify_data_url = args.http_modify_data_url;
        init_steering_url = args.http_init_steering_url;
        clear_simulator_url = args.http_clear_simulator_url;
        set_count_url = args.http_set_count_url;
        set_cycle_url = args.http_set_cycle_url;
        dbc_json = args.dbc_json_filename;
    }

    BoxAPI::~BoxAPI()
    {
    }

    void BoxAPI::deleteSimulation(httpclient& httpCli)
    {
        httpCli.do_delete(clear_simulator_url);
    }

    void BoxAPI::stopAllSImulation(httpclient& httpCli)
    {
        
    }

    void BoxAPI::startSimulation(httpclient& httpCli)
    {
        
    }

    void BoxAPI::createSimulation(httpclient& httpCli)
    {
        std::string dst_phy_id = "can1";
        nlohmann::json jStart;
        jStart["uuid"] = "abc1"; // args.http_box_start_url.substr(sStart, eStart-sStart).c_str(); // "abc1"
        jStart["type"] = "CAN-SIM";
        jStart["format"] = "DBC";
        jStart["dst_phy_id"] = dst_phy_id.c_str();
        jStart["file_path"] = dbc_json.c_str();  //   "Fahrsimulator_PCAN_72.json";
        jStart["file_src"] = "DATABASE";
        std::string dataCreatesim(jStart.dump());
        std::string boxURL = "/"; // "http://"+args.http_access_ip+":"+std::to_string(args.http_access_port);
        httpCli.do_post(dataCreatesim, boxURL);

    }

    void BoxAPI::activateMsgID(httpclient& httpCli)
    {
        httpCli.do_post("", box_activate_url+"512");
        httpCli.do_post("", box_activate_url+"513");
    } 
    
    void BoxAPI::steeringQuitError(httpclient& httpCli)   
    {
        std::vector<SIGNAL_t> signal_data_512_quit_error = {
            {"ControlWord", "15"},
            {"ModeOfOperation", "1"},
            {"AuxiliaryFunctions", "0"},
            {"EndStopPosition", "360"},
            {"PositionOffset", "220"},
            {"TorqueLimitation", "20"},
            {"PeakTorqueLimitation", "40"}
        };

    nlohmann::json j_quit_error = signal_data_512_quit_error;
    std::string modify_data_512_url = modify_url(modify_data_url, 512);
    httpCli.do_put(j_quit_error.dump(), modify_data_512_url);
    }

    void BoxAPI::steeringReady(httpclient& httpCli)   
    {
        std::vector<SIGNAL_t> signal_data_512_off_ready = {
            {"ControlWord", "2"},
            {"ModeOfOperation", "1"},
            {"AuxiliaryFunctions", "0"},
            {"EndStopPosition", "540"},
            {"PositionOffset", "220"},
            {"TorqueLimitation", "20"},
            {"PeakTorqueLimitation", "40"}
        };
        nlohmann::json j_off_ready = signal_data_512_off_ready;
        std::string modify_data_512_url = modify_url(modify_data_url, 512);
        httpCli.do_put(j_off_ready.dump(), modify_data_512_url);
    }

    void BoxAPI::steeringOn(httpclient& httpCli)   
    {
        // 6.init:on
        std::vector<SIGNAL_t> signal_data_512_ready_on = {
            {"ControlWord", "4"},
            {"ModeOfOperation", "1"},
            {"AuxiliaryFunctions", "0"},
            {"EndStopPosition", "360"},
            {"PositionOffset", "220"},
            {"TorqueLimitation", "20"},
            {"PeakTorqueLimitation", "40"}
        };
        nlohmann::json j_ready_on = signal_data_512_ready_on;
        std::string modify_data_512_url = modify_url(modify_data_url, 512);
        httpCli.do_put(j_ready_on.dump(), modify_data_512_url);
    }

    void BoxAPI::steeringIniParam(httpclient& httpCli)
    {
        std::vector<SIGNAL_t> signal_data_513_steering_param = {
            {"DesiredTorque", "0"},
            {"Friction", "0"},
            {"Damping", "50"},
            {"SpringStiffness", "30"}
        };
        nlohmann::json j_steering_param = signal_data_513_steering_param;
        httpCli.do_put(j_steering_param.dump(), modify_data_url);
    }

    std::string BoxAPI::modify_url(std::string ori_url, int number) {
        std::string numberStr = std::to_string(number);
        size_t startPos = ori_url.find("513");

        if(startPos != std::string::npos)
            ori_url.replace(startPos, 3, numberStr);

        // std::cout << "New URL: " << ori_url << std::endl;
        return ori_url;
    }
}



