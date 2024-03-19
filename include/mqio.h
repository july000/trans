
#ifndef MQIO_H_
#define MQIO_H_

#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#ifndef WIN32
#include "unistd.h"
#endif
#include "Arg_helper.h"
#include "DefaultMQProducer.h"
#include "DefaultMQPushConsumer.h"
#include "Config_.h"
#include "simapi.h"
#include "nlohmann/json.hpp"
using namespace rocketmq;

struct SendAndConsumerArgs {
 public:
  SendAndConsumerArgs();

 public:
  std::string bridgeio_ip;
  std::string namesrv;
  std::string namesrv_domain;
  std::string groupname;
  // std::string body;
  int thread_count;
  bool broadcasting;
  bool syncpush;
  bool SelectUnactiveBroker;  // default select active broker
  bool IsAutoDeleteSendCallback;
  int retrytimes;  // default retry 5 times;
  bool PrintMoreInfo;
  rocketmq::elogLevel log_level; // FATAL=1,ERROR=2,WARN=3,INFO=4,DEBUG=5,TRACE=6,LEVEL_NUM=7

  std::string topic_air;
  std::string topic_vehicle_moving;
  std::string topic_cargate;
  std::string topic_nio;
  std::string topic_laneinfo;
  std::string topic_acc;
  std::string topic_car;
  std::string topic_aio;
  std::string topic_lab;
  std::string topic_driveselect;
  std::string topic_collision;
  std::string topic_case_type;
  int minDistThre;
  // int ttlThre;

  int simone_case_type;

	// std::string websocket_client_ip;
	// std::string websocket_client_port;
  std::string websocket_client_uri;
  std::string steering_max;

  std::string http_access_ip;
	int http_access_port;
	// std::string http_user;
	// std::string http_password;
	std::string http_content_type;
	std::string http_box_start_url;
	std::string http_box_stop_url;
	std::string http_box_activate_url;
	std::string http_modify_data_url;
	std::string http_init_steering_url;
  std::string http_clear_simulator_url;
  std::string http_set_count_url;
  std::string http_set_cycle_url;
  std::string http_get_data_529_url;

  std::string asc_filename;
  std::string dbc_json_filename;
};

simapi RunSimOneAPI(SendAndConsumerArgs &args);

class TpsReportService {
 public:
  TpsReportService();
  ~TpsReportService();

  void start();
  void Increment();
  void TpsReport();

 private:
  std::chrono::seconds tps_interval_;
  std::shared_ptr<std::thread> tps_thread_;
  std::atomic<bool> quit_flag_;
  std::atomic<long> tps_count_;
};

struct ACC_ObjVehicle
{
  int frame;
  int id;
  int type;
  float pos_x;
  float pos_y;
  float pos_z;
  float ori_x;
  float ori_y;
  float ori_z;
  float zd_x;
  float zd_y;
  float zd_z;
};

struct VehicleMoving_SimData
{
  int frame;
	float posX;
	float posY;
	float posZ;
	float oriX;
	float oriY;
	float oriZ;
};

class Producer
{
  public:
    Producer(SendAndConsumerArgs &args);
    ~Producer();

    void SendAIR(simapi &api, std::ofstream &log);
    void SendVehicleMovingACC(simapi &api, std::ofstream &log);
    void SendVehicleMoving(simapi &api, std::ofstream &log);
    void SendCarGate(simapi &api, std::ofstream &log);
    void SendNIO(simapi &api, std::ofstream &log);
    void SendLaneInfo(simapi &api, std::ofstream &log);
    void SendACC(simapi &api, std::ofstream &log);
    void SendCAR(simapi &api, std::ofstream &log);
    void SetAIOData(nlohmann::json& j, SimOne_Data_SensorDetections_Entry* pObj, SimOne_Data_TrafficLight* pTrafficLight);
    void SendAIO(simapi &api, std::ofstream &log);
    void SendLAB(simapi &api, std::ofstream &log);
    void SendDriveSelect(simapi &api, std::ofstream &log);
    static void JudgeEventCB(const char* mainVehicleId, SimOne_Data_JudgeEvent *judgeEventDetailInfo);
    static void ScenarioEventCB(const char* source, const char* target, const char* type, const char* content);
    static void GpsUpdateCB(const char* mainVehicleId, SimOne_Data_Gps *pGps);
    static void SensorDetectionsUpdateCB(const char* mainVehicleId, const char *sensorId, SimOne_Data_SensorDetections *pDetections);

    void SendCollision(simapi &api, int minDistThre, std::ofstream &log);
    void SendCaseType(simapi &api, std::ofstream &log);

    static int m_case_type;

  private:
    SendAndConsumerArgs &m_args;
    DefaultMQProducer m_producer;
    static bool m_collided;
    static int m_route_type;
    static int m_trafficlight_type;
    static int m_sim_driv_turn_light;
    static int m_sim_waiting_area_sig;

    static std::mutex mtx;
    static std::condition_variable cv;
    static std::atomic<bool> m_sync_vm_acc;
    // int m_TTL;
    // static std::mutex mtx_vehiclemoving;
    // static std::condition_variable cv_vehiclemoving;
    static std::atomic<bool> m_sync_vehiclemoving;
    static std::atomic<bool> m_sync_gps;
    static std::atomic<bool> m_sync_vehiclemoving_frame;
    // static std::atomic<bool> m_gps_sync_ok;
    // static std::atomic<bool> m_acc_sync_ok;

    static VehicleMoving_SimData m_vehiclemoving;
    static std::vector<ACC_ObjVehicle> m_objVehicles;


    static std::mutex mtx_gps;
    static std::map<int, VehicleMoving_SimData> m_vehicle_moving_map;
    static std::mutex mtx_detections;
    static std::map<int, std::vector<ACC_ObjVehicle>> m_obj_vehicles_map;




    static int m_vehiclemoving_frame;
};

class Listener : public MessageListenerConcurrently
{
  public:
    Listener(TpsReportService &tps);
    virtual ~Listener();
    virtual ConsumeStatus consumeMessage(const std::vector<MQMessageExt>& msgs);

  private:
    TpsReportService &m_tps;
};

class Consumer
{
  public:
    // Consumer(SendAndConsumerArgs &args, DefaultMQPushConsumer &consumer, TpsReportService &m_tps);
    Consumer(SendAndConsumerArgs &args);
    ~Consumer();

  private:
    SendAndConsumerArgs &m_args;
    static TpsReportService m_tps;
    Listener m_listener;
    DefaultMQPushConsumer m_consumer;

    // std::mutex g_mtx;
    // std::condition_variable g_finished;
    // std::atomic<int> g_msgCount(10240);
};

#endif  // UTILITY_H_