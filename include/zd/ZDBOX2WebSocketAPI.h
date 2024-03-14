#ifndef ZDBOX_H_
#define ZDBOX_H_
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
typedef websocketpp::client<websocketpp::config::asio_client> client;

namespace ZDBOX
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
}

#endif