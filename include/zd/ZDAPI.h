#ifndef ZDBOX_H_
#define ZDBOX_H_
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include "http_client.h"
#include "mqio.h"

typedef websocketpp::client<websocketpp::config::asio_client> client;

namespace ZDAPI
{
  class WebsocketAPI
  {
    public:
      WebsocketAPI(){}
      ~WebsocketAPI(){}

      void heartbeat(client *c, websocketpp::connection_hdl hdl);
      void addChannel(client *c, websocketpp::connection_hdl hdl);
      void getChannel(client *c, websocketpp::connection_hdl hdl);
      void setRule(client *c, websocketpp::connection_hdl hdl);
      void getRule(client *c, websocketpp::connection_hdl hdl);

      void setControl(client *c, websocketpp::connection_hdl hdl, std::string &ctl);
      
    private:
      std::mutex mMutex_Id;
      int reqID = 1;
  };

  class BoxAPI
  {
    public:
      BoxAPI(SendAndConsumerArgs& args);
      ~BoxAPI();

      void DeleteSimulation();
      void StopAllSImulation();
      void StartSimulation();
      void StopSImulation();
      void ActivateMsgID();
      void ModifyMsgData();

    private:
      std::string content_type;
      std::string box_start_url;
      std::string box_stop_url;
      std::string box_activate_url;
      std::string modify_data_url;
      std::string init_steering_url;
      std::string clear_simulator_url;
      std::string set_count_url;
      std::string set_cycle_url;

  };
}

#endif