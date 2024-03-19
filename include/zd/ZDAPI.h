#ifndef ZDBOX_H_
#define ZDBOX_H_
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include "http_client.h"
#include "mqio.h"

typedef websocketpp::client<websocketpp::config::asio_client> client;


namespace nlohmann {
template <>
struct adl_serializer<SIGNAL_t> {
    static void to_json(json& j, const SIGNAL_t& item) {
        // This tell the json library how to convert Person to json
        j = json{{"name", item.name}, {"raw", item.raw}};
    }
};
}// namespace nlohmann

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

      void deleteSimulation(httpclient& httpCli);
      void startSimulation(httpclient& httpCli);
      void stopAllSImulation(httpclient& httpCli);
      void stopSimulation(httpclient& httpCli);
      void createSimulation(httpclient& httpCli);
      void activateMsgID(httpclient& httpCli);
      void modifyMsgData(httpclient& httpCli);
      void steeringQuitError(httpclient& httpCli);
      void steeringReady(httpclient& httpCli); 
      void steeringOn(httpclient& httpCli);
      void steeringIniParam(httpclient& httpCli);
      std::string modify_url(std::string ori_url, int number);

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
      std::string dbc_json;

  };
}

#endif