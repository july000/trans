#include "zd/ZDAPI.h"
#include "websocketpp/config/asio_no_tls_client.hpp"
#include "websocketpp/client.hpp"

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
    }

    BoxAPI::~BoxAPI()
    {
    }

    void BoxAPI::DeleteSimulation()
    {

    }

    void DeleteSimulation()
    {
        
    }

    void StopAllSImulation()
    {
        
    }

    void StartSimulation()
    {
        
    }

    void StopSImulation()
    {
        
    }

    void ActivateMsgID()
    {
        
    } 
    
    void ModifyMsgData()   
    {
        
    }
}
