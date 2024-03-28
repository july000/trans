#include "mqio.h"
#include "client.h"

TpsReportService Consumer::m_tps;

bool Producer::m_collided;
int Producer::m_route_type;
int Producer::m_trafficlight_type;
int Producer::m_sim_driv_turn_light = 0;
int Producer::m_sim_waiting_area_sig = 0;
int Producer::m_case_type = 0;

std::mutex Producer::mtx;
std::condition_variable Producer::cv;
std::atomic<bool> Producer::m_sync_vm_acc; // true vehicle_moving; false acc

// std::mutex Producer::mtx_vehiclemoving;
// std::condition_variable Producer::cv_vehiclemoving;
std::atomic<bool> Producer::m_sync_vehiclemoving;
std::atomic<bool> Producer::m_sync_gps;
std::atomic<bool> Producer::m_sync_vehiclemoving_frame;

// std::atomic<bool> Producer::m_gps_sync_ok;
// std::atomic<bool> Producer::m_acc_sync_ok;

VehicleMoving_SimData Producer::m_vehiclemoving;
std::vector<ACC_ObjVehicle> Producer::m_objVehicles;


std::mutex Producer::mtx_gps;
std::map<int, VehicleMoving_SimData> Producer::m_vehicle_moving_map;
std::mutex Producer::mtx_detections;
std::map<int, std::vector<ACC_ObjVehicle>> Producer::m_obj_vehicles_map;


int Producer::m_vehiclemoving_frame;

std::string getCurrentChinaTime() {
    // 获取当前系统时间点
    auto currentTime = std::chrono::system_clock::now();

    // 转换为时间结构体
    std::time_t currentTime_t = std::chrono::system_clock::to_time_t(currentTime);

    // 格式化时间为本地时间字符串
    std::tm* localTime = std::localtime(&currentTime_t);
    localTime->tm_hour += 8;  // 将小时字段加上8
    std::mktime(localTime);  // 调整时间结构体

    // 获取毫秒数
    auto duration = currentTime.time_since_epoch();
    auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(duration) % 1000;

    // 构造中国地区的时间字符串
    char timeStr[100];
    std::strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", localTime);
    std::string chinaTime = timeStr + std::string(".") + std::to_string(milliseconds.count());

    return chinaTime;
}

SendAndConsumerArgs::SendAndConsumerArgs()
{
  rr::RrConfig config;
  config.ReadConfig("TransConfig.ini");

  // Config BridgeIO
  bridgeio_ip = config.ReadString("BridgeIO", "IP", "127.0.0.1");

  // Config RocketMQ
  namesrv = config.ReadString("MQ", "NameServer", "127.0.0.1:9876");
  namesrv_domain = config.ReadString("MQ", "NameServerDomain", "Space");
  groupname = config.ReadString("MQ", "GroupName", "Team");
  log_level = rocketmq::elogLevel(config.ReadInt("MQ", "LogLevel", 2)); // FATAL=1,ERROR=2,WARN=3,INFO=4,DEBUG=5,TRACE=6,LEVEL_NUM=7

  topic_air = config.ReadString("MQ", "TopicAIR", "AIR");
  topic_vehicle_moving = config.ReadString("MQ", "TopicVehicleMoving", "VEHICLE_MOVING");
  topic_cargate = config.ReadString("MQ", "TopicCarGate", "CarGate");
  topic_nio = config.ReadString("MQ", "TopicNIO", "NIO");
  topic_laneinfo = config.ReadString("MQ", "TopicLaneInfo", "LaneInfo");
  topic_acc = config.ReadString("MQ", "TopicACC", "ACC");
  topic_car = config.ReadString("MQ", "TopicCAR", "CAR");
  topic_aio = config.ReadString("MQ", "TopicAIO", "AIO");
  topic_lab = config.ReadString("MQ", "TopicLab", "LAB");
  topic_driveselect = config.ReadString("MQ", "TopicDriveSelect", "DRIVESELECT");
  topic_collision = config.ReadString("MQ", "TopicSCP", "SCP");
  topic_case_type = config.ReadString("MQ", "TopicProjectType", "PROJECT_TYPE");
  minDistThre = config.ReadInt("MQ", "SafeDist", 30);
  // ttlThre = config.ReadInt("MQ", "TTL", 30000);

  simone_case_type = config.ReadInt("CaseType", "TYPE", 2);

  // Config Websocket
	std::string websocket_client_ip = config.ReadString("WebsocketClient", "ServerIp", "127.0.0.1");
	std::string websocket_client_port = config.ReadString("WebsocketClient", "ServerPort", "9002");
  steering_max = config.ReadString("Steering", "SteeringMax", "40303");
  websocket_client_uri = "ws://" + websocket_client_ip + ":" + websocket_client_port;

  // Config HTTP
  http_access_ip = config.ReadString("HTTP", "HTTP_Client_IP", "");
	http_access_port = config.ReadInt("HTTP", "HTTP_Client_Port", 0);
	// http_user = config.ReadString("HTTP", "HTTP_Client_User", "");
	// http_password = config.ReadString("HTTP", "HTTP_Client_Password", "");
	http_content_type = config.ReadString("HTTP", "HTTP_Client_Content_Type", "");
	http_box_start_url = config.ReadString("HTTP", "HTTP_Client_BoxStartURL", "");
	http_box_stop_url = config.ReadString("HTTP", "HTTP_Client_BoxStopURL", "");
	http_box_activate_url = config.ReadString("HTTP", "HTTP_Client_BoxActivateURL", "");
	http_modify_data_url = config.ReadString("HTTP", "HTTP_Client_ModifyDataURL", "");
	http_init_steering_url = config.ReadString("HTTP", "HTTP_Client_INIT_STEERING", "");

	http_clear_simulator_url = config.ReadString("HTTP", "HTTP_Client_CLEAR", "");
	http_set_count_url = config.ReadString("HTTP", "HTTP_Client_SetCount", "");
	http_set_cycle_url = config.ReadString("HTTP", "HTTP_Client_SetCycle", "");
  http_get_data_529_url = config.ReadString("HTTP", "HTTP_Client_GetData", "");

  can_dev_t = config.ReadString("CAN", "CAN_T", "can0");
  can_dev_r = config.ReadString("CAN", "CAN_R", "can1");

  // Config ASC
  asc_filename = config.ReadString("ASC", "FILENAME", "");
  dbc_json_filename = config.ReadString("DBC", "FILENAME", "Fahrsimulator_PCAN_72_wheel.json");

  thread_count = std::thread::hardware_concurrency();
  broadcasting = false;
  syncpush = false;
  SelectUnactiveBroker = false; // default select active broker
  IsAutoDeleteSendCallback = false;
  retrytimes = 5; // default retry 5 times;
  PrintMoreInfo = false;
}

// ------ SimOneAPI ------
simapi RunSimOneAPI(SendAndConsumerArgs &args)
{
  simapi api("0");
  api.SimAPI_InitSimOneAPI(false, args.bridgeio_ip.c_str());
  std::vector<std::string> apiNames = {}; // {"GetTrafficSignList","GetTrafficLightList","GetCrossHatchList","GetLaneLink"};
  api.SimAPI_HDMap_ALL(apiNames);
  api.SimAPI_RegisterVehicleState();
  return api;
}

// ------ Producer ------
Producer::Producer(SendAndConsumerArgs &args): m_args(args), m_producer("unique_group_name")
{
  m_producer.setSessionCredentials("Sim-Access", "Sim-Secret", "Sim-Channel");
  m_producer.setTcpTransportTryLockTimeout(1000);
  m_producer.setTcpTransportConnectTimeout(400);
  m_producer.setNamesrvDomain(m_args.namesrv_domain);
  m_producer.setNamesrvAddr(m_args.namesrv);
  m_producer.setGroupName(m_args.groupname);
  m_producer.setLogLevel(m_args.log_level);
  m_producer.start();
  m_collided = false;
  m_route_type = 0;
  m_trafficlight_type = 0;
  m_sync_vm_acc.store(false);
  // m_TTL = m_args.ttlThre;
  m_sync_vehiclemoving.store(false);
  m_sync_gps.store(false);
  m_sync_vehiclemoving_frame.store(false);

  // m_gps_sync_ok.store(false);
  // m_acc_sync_ok.store(true);

  // m_sim_vehiclemoving.acc_frame = 0;
  m_vehiclemoving_frame = 0;
}
Producer::~Producer()
{
  m_producer.shutdown();
}

void Producer::SendAIR(simapi &api, std::ofstream &log)
{		
  bool isInLane = false;
  std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
	if (!SimOneAPI::GetGps(api.mainVehicleId.c_str(), pGps.get()))
	{
		std::cout << "Get GPS Failed" << std::endl;
    isInLane = false;
	}

  SSD::SimString laneName;
	double s, t, s_toCenterLine, t_toCenterLine;
	SSD::SimPoint3D pos = {pGps->posX, pGps->posY, pGps->posZ};
	if (SimOneAPI::GetNearMostLane(pos, laneName, s, t, s_toCenterLine, t_toCenterLine))
  {
    isInLane = true;
  }
  // std::unique_ptr<SimOne_Data_LaneInfo> pLaneInfo = std::make_unique<SimOne_Data_LaneInfo>();
  // api.SimAPI_SensorLaneInfo(pLaneInfo.get());
  // bool isInLane = pLaneInfo->c_Line.lineID == 0 ? false : true;

  nlohmann::json j;
  j["ResetFidOptions"] = "";
  j["RouteGuidanceStarted"] = isInLane;
  //j["DriverLane"] = isInLane;
  j["ResetCamera"] = false;

  MQMessage msg_air(m_args.topic_air, "*", j.dump());
  msg_air.setKeys(m_args.topic_air);
  // msg_air.setProperty("TTL", std::to_string(m_TTL));
  try
  {
    SendResult sendResult = m_producer.send(msg_air);
    // std::cout << "----- SendAIR : " << j.dump() << std::endl;
    log << "[" << getCurrentChinaTime() << "] " << j.dump() << std::endl;
    log.flush();
    }
    catch (MQException &e)
    {
      std::cout << e << endl;
    }
}

void Producer::SendVehicleMovingACC(simapi &api, std::ofstream &log)
{
  std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>(); 
  bool m_sync_ok = false; // std::atomic<bool> m_sync_ok; m_sync_ok.store(false);
  int frame = 0;

  // ------------ Send VehicleMoving ------------
  while(!m_sync_ok) // while(!m_sync_ok.load())
  {
    frame = m_vehiclemoving_frame;
    if (frame == 0)
    {
      m_objVehicles.clear();
      break;
    }

    {
      std::lock_guard<std::mutex> lock(mtx_detections);
      auto it = m_obj_vehicles_map.find(frame);
      if (it == m_obj_vehicles_map.end())
      {
        continue;
      }

      // while (m_sync_vehiclemoving.load())
      // {
      //   std::this_thread::yield();
      // }
      // m_sync_vehiclemoving.store(true);
      m_objVehicles.clear();
      m_objVehicles = m_obj_vehicles_map[frame];
      // log_detections_cb << "------------ frame: " << vehiclemoving_frame << std::endl; // log_detections_cb << "obj_vehicles size: " << m_objVehicles.size() << std::endl; // for (int i=0; i<m_objVehicles.size(); i++) // { //   log_detections_cb << "objVehicles[" << i << "].id: " << m_objVehicles[i].id << std::endl; //   log_detections_cb << "objVehicles[" << i << "].pos: [" << m_objVehicles[i].pos_x << ", " << m_objVehicles[i].pos_y << ", " << m_objVehicles[i].pos_z << "]" << std::endl; //   log_detections_cb << "objVehicles[" << i << "].ori: [" << m_objVehicles[i].ori_x << ", " << m_objVehicles[i].ori_y << ", " << m_objVehicles[i].ori_z << "]" << std::endl; //   log_detections_cb << "objVehicles[" << i << "].zd: [" << m_objVehicles[i].zd_x << ", " << m_objVehicles[i].zd_y << ", " << m_objVehicles[i].zd_z << "]" << std::endl; // } // log_detections_cb.flush();
      // m_sync_vehiclemoving.store(false);
    }

    {
      std::lock_guard<std::mutex> lock(mtx_gps);
      auto it = m_vehicle_moving_map.find(frame);
      if (it == m_vehicle_moving_map.end())
      {
        continue;
      }

      // while (m_sync_gps.load())
      // {
      //   std::this_thread::yield();
      // }
      // m_sync_gps.store(true);
      m_vehiclemoving = m_vehicle_moving_map[frame];
      // log_gps_cb << "------------ frame: " << gps_frame << std::endl; // log_gps_cb << "pos: [" << m_vehiclemoving.posX << ", " << m_vehiclemoving.posY << ", " << m_vehiclemoving.posZ << "]" << std::endl; // log_gps_cb << "ori: [" << m_vehiclemoving.oriX << ", " << m_vehiclemoving.oriY << ", " << m_vehiclemoving.oriZ << "]" << std::endl; // log_gps_cb.flush();
      // m_sync_gps.store(false);
    }

    m_sync_ok = true; // m_sync_ok.store(true);
  }
  // m_vehiclemoving_frame = frame;

  if (frame == 0)
  {
    if (api.SimAPI_GPS(pGps.get()))
    {
      m_vehiclemoving_frame = pGps->frame;

      nlohmann::json j;
      // j["frame"] = pGps->frame; // -------------------------- Debug
      j["Position"].emplace_back(pGps->posX);                         // Unity x = (od X)
      j["Position"].emplace_back(pGps->posZ);                         // Unity y = (od Z)
      j["Position"].emplace_back(pGps->posY);                         // Unity z = (od Y)
      j["Rotation"].emplace_back(-pGps->oriX);                        // Unity oriX = -(od oriX)
      j["Rotation"].emplace_back(90.0 - (pGps->oriZ * 180.0 / M_PI)); // Unity oriY = 90 - (od oriZ)
      j["Rotation"].emplace_back(-pGps->oriY);                        // Unity oriZ = -(od oriY)

      MQMessage msg_vehicle_moving(m_args.topic_vehicle_moving, "*", j.dump());
      msg_vehicle_moving.setKeys(m_args.topic_vehicle_moving);
      try
      {
        SendResult sendResult = m_producer.send(msg_vehicle_moving);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        log << "[" << getCurrentChinaTime() << "] frame: " << pGps->frame << j.dump() << std::endl;
        log.flush();
      }
      catch (MQException &e)
      {
        std::cout << e << endl;
      }
    }
  }
  else
  {
    int gps_frame = 0;
    // while (m_sync_gps.load())
    // {
    //   std::this_thread::yield();
    // }
    // m_sync_gps.store(true);
    nlohmann::json j;
    // j["frame"] = pGps->frame; // -------------------------- Debug
    j["Position"].emplace_back(m_vehiclemoving.posX);                         // Unity x = (od X)
    j["Position"].emplace_back(m_vehiclemoving.posZ);                         // Unity y = (od Z)
    j["Position"].emplace_back(m_vehiclemoving.posY);                         // Unity z = (od Y)
    j["Rotation"].emplace_back(-m_vehiclemoving.oriX);                        // Unity oriX = -(od oriX)
    j["Rotation"].emplace_back(90.0 - (m_vehiclemoving.oriZ * 180.0 / M_PI)); // Unity oriY = 90 - (od oriZ)
    j["Rotation"].emplace_back(-m_vehiclemoving.oriY);                        // Unity oriZ = -(od oriY)
    gps_frame = m_vehiclemoving.frame;
    m_sync_gps.store(false);

    MQMessage msg_vehicle_moving(m_args.topic_vehicle_moving, "*", j.dump());
    msg_vehicle_moving.setKeys(m_args.topic_vehicle_moving);
    try
    {
      SendResult sendResult = m_producer.send(msg_vehicle_moving);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      log << "[" << getCurrentChinaTime() << "] frame: " << gps_frame << j.dump() << std::endl;
      log.flush();
    }
    catch (MQException &e)
    {
      std::cout << e << endl;
    }
  }

  //  ------------ Send ACC ------------
  SSD::SimString mvLaneName;
  if (!api.SimAPI_GetLaneName({pGps->posX, pGps->posY, pGps->posZ}, mvLaneName))
  {
    return;
  }

  if (0 == m_objVehicles.size())
  {
    // m_sim_vehiclemoving.acc_frame = 0;
    nlohmann::json j;
    j["VehicleAheadPosition"] = nlohmann::json::array();
    j["DistanceAdjustment"] = 0;           // 距离调整  0，3
    j["DistanceAdjustmentActive"] = false; // websocket_client::mDrivingLever.isCruiseOn; // 距离调整激活 false
    j["PredictiveControlType"] = 0;        // 预测控制类型 0
    j["PredictiveControlSpeed"] = 0;       // 预测控制速度 0，327

    j["SpeedLimit"] = websocket_client::mCruise_speed ? std::round(websocket_client::mCruise_speed * 3.6) : 333.0; // 速度限制 333.0 --mk/h
    j["Status"] = websocket_client::mCruise_valid ? 3 : 1;                                                                       // 状态 Green (unity 3 On/5 ReadyToGo); Wite (unity 1 Wait/2 Standby/4 Violating); Disapear (unity 0 Off, skip this status)
    j["VehicleAheadCountdown"] = 0.0;                                                                                            // minDist; ------------
    j["VehicleAheadID"] = 0;                                                                                                     // aheadVId; ------------
    j["VehicleOnEgoLaneID"] = 0;                                                                                                 // 100
    j["VehicleType"] = 1;

    MQMessage msg_acc(m_args.topic_acc, "*", j.dump());
    msg_acc.setKeys(m_args.topic_acc);
    try
    {
      SendResult sendResult = m_producer.send(msg_acc);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      log << "[" << getCurrentChinaTime() << "] " << j.dump() << std::endl;
      log.flush();
    }
    catch (MQException &e)
    {
      std::cout << e << endl;
    }
  }
  else
  {
    int aheadVId = 0;
    double minDist = std::numeric_limits<double>::infinity();
    for (int i = 0; i < m_objVehicles.size(); i++)
    {
      double dist = 0.0;
      if (api.SimAPI_GetObjVehicleDist({pGps->posX, pGps->posY, pGps->posZ}, {m_objVehicles[i].pos_x, m_objVehicles[i].pos_y, m_objVehicles[i].pos_z}, mvLaneName, dist))
      {
        if (dist < minDist)
        {
          minDist = dist;
          aheadVId = m_objVehicles[i].id;
        }
      }
    }

    for (int i = 0; i < m_objVehicles.size(); i++)
    {
      nlohmann::json j;
      // j["frame"] = m_vehiclemoving_frame; // -------------------------- Debug
      j["VehicleAheadPosition"].emplace_back(m_objVehicles[i].zd_x);
      j["VehicleAheadPosition"].emplace_back(m_objVehicles[i].zd_y);
      j["VehicleAheadPosition"].emplace_back(m_objVehicles[i].zd_z);
      j["VehicleAheadPosition"].emplace_back(m_objVehicles[i].ori_x);
      j["VehicleAheadPosition"].emplace_back(m_objVehicles[i].ori_y);
      j["VehicleAheadPosition"].emplace_back(m_objVehicles[i].ori_z);

      j["DistanceAdjustment"] = 0;                                                                                                 // 距离调整  0，3
      j["DistanceAdjustmentActive"] = false;                                                                                       // websocket_client::mDrivingLever.isCruiseOn; // 距离调整激活 false
      j["PredictiveControlType"] = 0;                                                                                              // 预测控制类型 0
      j["PredictiveControlSpeed"] = 0;                                                                                             // 预测控制速度 0，327
      j["SpeedLimit"] = websocket_client::mCruise_speed ? std::round(websocket_client::mCruise_speed * 3.6) : 333.0; // 速度限制 333.0 --mk/h
      j["Status"] = websocket_client::mCruise_valid ? 3 : 1;                                                                       // 状态 Green (unity 3 On/5 ReadyToGo); Wite (unity 1 Wait/2 Standby/4 Violating); Disapear (unity 0 Off, skip this status)
      j["VehicleAheadCountdown"] = std::isinf(minDist) ? 0.0 : (float)(minDist);                                                   // minDist; ------------
      j["VehicleAheadID"] = aheadVId;                                                                                              // aheadVId; ------------
      j["VehicleOnEgoLaneID"] = m_objVehicles[i].id;
      j["VehicleType"] = m_objVehicles[i].type; // 100

      MQMessage msg_acc(m_args.topic_acc, "*", j.dump());
      msg_acc.setKeys(m_args.topic_acc);
      try
      {
        SendResult sendResult = m_producer.send(msg_acc);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        log << "[" << getCurrentChinaTime() << "] frame: " << m_objVehicles[i].frame << " ----- SendACC: " << j.dump() << std::endl;
        log.flush();
      }
      catch (MQException &e)
      {
        std::cout << e << endl;
      }
    }
  }

}

void Producer::SendVehicleMoving(simapi &api, std::ofstream &log)
{
  std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
  std::unique_lock<std::mutex> lock(mtx);
  cv.wait(lock, [&](){ return !m_sync_vm_acc.load(); });

  // if (!api.SimAPI_GPS(pGps.get()))
  // {
  //   return;
  // }
  // m_gps_sync_ok.store(false);
  // m_acc_sync_ok.store(false);
  std::atomic<bool> m_sync_ok;
  m_sync_ok.store(false);
  int frame = 0;

  while(!m_sync_ok.load())
  {
    frame = m_vehiclemoving_frame;
    if (frame == 0)
    {
      m_objVehicles.clear();
      break;
    }
    // std::cout << "00000000000000000000000000000000000000000000 frame: " << frame << std::endl;
    {
      std::lock_guard<std::mutex> lock(mtx_detections);
      auto it = m_obj_vehicles_map.find(frame);
      if (it == m_obj_vehicles_map.end())
      {
        continue;
      }

      // while (m_sync_vehiclemoving.load())
      // {
      //   std::this_thread::yield();
      // }
      // m_sync_vehiclemoving.store(true);
      m_objVehicles.clear();
      m_objVehicles = m_obj_vehicles_map[frame];
      // log_detections_cb << "------------ frame: " << vehiclemoving_frame << std::endl; // log_detections_cb << "obj_vehicles size: " << m_objVehicles.size() << std::endl; // for (int i=0; i<m_objVehicles.size(); i++) // { //   log_detections_cb << "objVehicles[" << i << "].id: " << m_objVehicles[i].id << std::endl; //   log_detections_cb << "objVehicles[" << i << "].pos: [" << m_objVehicles[i].pos_x << ", " << m_objVehicles[i].pos_y << ", " << m_objVehicles[i].pos_z << "]" << std::endl; //   log_detections_cb << "objVehicles[" << i << "].ori: [" << m_objVehicles[i].ori_x << ", " << m_objVehicles[i].ori_y << ", " << m_objVehicles[i].ori_z << "]" << std::endl; //   log_detections_cb << "objVehicles[" << i << "].zd: [" << m_objVehicles[i].zd_x << ", " << m_objVehicles[i].zd_y << ", " << m_objVehicles[i].zd_z << "]" << std::endl; // } // log_detections_cb.flush();
      // m_sync_vehiclemoving.store(false);
    }

    {
      std::lock_guard<std::mutex> lock(mtx_gps);
      auto it = m_vehicle_moving_map.find(frame);
      if (it == m_vehicle_moving_map.end())
      {
        continue;
      }

      // while (m_sync_gps.load())
      // {
      //   std::this_thread::yield();
      // }
      // m_sync_gps.store(true);
      m_vehiclemoving = m_vehicle_moving_map[frame];
      // log_gps_cb << "------------ frame: " << gps_frame << std::endl; // log_gps_cb << "pos: [" << m_vehiclemoving.posX << ", " << m_vehiclemoving.posY << ", " << m_vehiclemoving.posZ << "]" << std::endl; // log_gps_cb << "ori: [" << m_vehiclemoving.oriX << ", " << m_vehiclemoving.oriY << ", " << m_vehiclemoving.oriZ << "]" << std::endl; // log_gps_cb.flush();
      // m_sync_gps.store(false);
    }

    m_sync_ok.store(true);
  }
  // m_vehiclemoving_frame = frame;



  if (frame == 0)
  {
    if (api.SimAPI_GPS(pGps.get()))
    {
      m_vehiclemoving_frame = pGps->frame;

      nlohmann::json j;
      // j["frame"] = pGps->frame; // -------------------------- Debug
      j["Position"].emplace_back(pGps->posX);                         // Unity x = (od X)
      j["Position"].emplace_back(pGps->posZ);                         // Unity y = (od Z)
      j["Position"].emplace_back(pGps->posY);                         // Unity z = (od Y)
      j["Rotation"].emplace_back(-pGps->oriX);                        // Unity oriX = -(od oriX)
      j["Rotation"].emplace_back(90.0 - (pGps->oriZ * 180.0 / M_PI)); // Unity oriY = 90 - (od oriZ)
      j["Rotation"].emplace_back(-pGps->oriY);                        // Unity oriZ = -(od oriY)

      MQMessage msg_vehicle_moving(m_args.topic_vehicle_moving, "*", j.dump());
      msg_vehicle_moving.setKeys(m_args.topic_vehicle_moving);
      try
      {
        SendResult sendResult = m_producer.send(msg_vehicle_moving);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        log << "[" << getCurrentChinaTime() << "] frame: " << pGps->frame << j.dump() << std::endl;
        log.flush();
      }
      catch (MQException &e)
      {
        std::cout << e << endl;
      }
    }
  }
  else
  {
    int gps_frame = 0;
    // while (m_sync_gps.load())
    // {
    //   std::this_thread::yield();
    // }
    // m_sync_gps.store(true);
    nlohmann::json j;
    // j["frame"] = pGps->frame; // -------------------------- Debug
    j["Position"].emplace_back(m_vehiclemoving.posX);                         // Unity x = (od X)
    j["Position"].emplace_back(m_vehiclemoving.posZ);                         // Unity y = (od Z)
    j["Position"].emplace_back(m_vehiclemoving.posY);                         // Unity z = (od Y)
    j["Rotation"].emplace_back(-m_vehiclemoving.oriX);                        // Unity oriX = -(od oriX)
    j["Rotation"].emplace_back(90.0 - (m_vehiclemoving.oriZ * 180.0 / M_PI)); // Unity oriY = 90 - (od oriZ)
    j["Rotation"].emplace_back(-m_vehiclemoving.oriY);                        // Unity oriZ = -(od oriY)
    gps_frame = m_vehiclemoving.frame;
    m_sync_gps.store(false);

    MQMessage msg_vehicle_moving(m_args.topic_vehicle_moving, "*", j.dump());
    msg_vehicle_moving.setKeys(m_args.topic_vehicle_moving);
    try
    {
      SendResult sendResult = m_producer.send(msg_vehicle_moving);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      log << "[" << getCurrentChinaTime() << "] frame: " << gps_frame << j.dump() << std::endl;
      log.flush();
    }
    catch (MQException &e)
    {
      std::cout << e << endl;
    }
  }

  {
    std::lock_guard<std::mutex> lock(mtx_detections);
    auto it = m_obj_vehicles_map.begin();
    while (it != m_obj_vehicles_map.end())
    {
      if (it->first < frame)
      {
        auto nextIt = std::next(it);
        m_obj_vehicles_map.erase(it);
        it = nextIt;
      }
      else
      {
        break;
      }
    }
  }



  m_sync_vm_acc.store(true);
  cv.notify_all();
}

bool fliper(bool on)
{
  static int counter = 7;
  static bool lighter_state = true;

  if (on)
  {
    // if (counter<0)
    // {
    //   lighter_state = !lighter_state;
    //   counter = 7;
    // }

    // counter --;
    return lighter_state;
  }
  else
  {
    return false;
  }
}

void Producer::SendCarGate(simapi &api, std::ofstream &log)
{
  std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
  if (api.SimAPI_GPS(pGps.get()))
  {
	  float vel = sqrt(pGps->velX*pGps->velX + pGps->velY*pGps->velY + pGps->velZ*pGps->velZ) * 3.6;
    nlohmann::json j;
    j["DisplayedVehicleSpeed"] = vel;// 显示车辆速度
	  j["EngineSpeed"]=pGps->engineRpm; // 引擎速度
	  j["SteeringWheelAngle"]=pGps->steering; // 方向盘角度

    // switch (pGps->gear)
    // {
    // case 0: // ESimOne_Gear_Mode_Neutral
    //   j["GearSelection"] = 2;
    //   // std::cout << "------------- 2 ESimOne_Gear_Mode_Neutral" << std::endl;
    //   break;
    // case 1: // ESimOne_Gear_Mode_Parking
    //   j["GearSelection"] = 0;
    //   // std::cout << "------------- 0 ESimOne_Gear_Mode_Parking" << std::endl;
    //   break;
    // case -1: // ESimOne_Gear_Mode_Reverse
    //   j["GearSelection"] = 1;
    //   // std::cout << "------------- 1 ESimOne_Gear_Mode_Reverse" << std::endl;
    //   break;
    // // case ESimOne_Gear_Mode_Drive: // 2/3/...
    // default:
    //   // std::cout << "------------- 3 ESimOne_Gear_Mode_Drive" << std::endl;
    //   j["GearSelection"] = 3;
    // }

		// 0x5: Parking 0x6: Reverse 0x7: Neutral 0x8: Drive 0x9: Sport
    switch(websocket_client::phys_gear)
    {
      case 0x5:
        j["GearSelection"] = 0; //"P"
        break;
      case 0x6:
        j["GearSelection"] = 1; //"R"
        break;
      case 0x7:
        j["GearSelection"] = 2; //"N"
        break;
      case 0x8:
        j["GearSelection"] = 3; //"D"
        break;
      case 0x9:
        j["GearSelection"] = 4; //"S"
        break;
      default:
        j["GearSelection"] = 0; //"P"
        break;
    } 
    //j["GearSelection"] = websocket_client::phys_gear;
	  j["BrakeActive"]=pGps->brake == 0.0 ? false : true;

    if (api.SimAPI_GetControlMode())
    {
	    j["DashboardIndicators_OP_TurnSignalLightLeft"]= fliper(simapi::lights[0]); // 仪表板指示灯_OP_左转向信号灯 ---------------------------------
	    j["DashboardIndicators_OP_TurnSignalLightRight"]= fliper(simapi::lights[1]); // 仪表板指示灯_OP_右转向信号灯 --------------------------------
    }
    else
    {
      switch(m_sim_driv_turn_light)
      {
        case 1:
	        j["DashboardIndicators_OP_TurnSignalLightLeft"]= fliper(true);
	        j["DashboardIndicators_OP_TurnSignalLightRight"]= fliper(false);
          break;
        case 2:
	        j["DashboardIndicators_OP_TurnSignalLightLeft"]= fliper(false);
	        j["DashboardIndicators_OP_TurnSignalLightRight"]= fliper(true);
          break;
        case 0:
        default:
	        j["DashboardIndicators_OP_TurnSignalLightLeft"]= false;
	        j["DashboardIndicators_OP_TurnSignalLightRight"]= false;
      }
    }

    // std::cout << "=================================lights[0]: " << simapi::lights[0] << std::endl;
    // std::cout << "=================================lights[1]: " << simapi::lights[1] << std::endl;

    MQMessage msg_cargate(m_args.topic_cargate, "*", j.dump());
    msg_cargate.setKeys(m_args.topic_cargate);
    // msg_cargate.setProperty("TTL", std::to_string(m_TTL));
    try
    {
      SendResult sendResult = m_producer.send(msg_cargate);
      // std::cout << "----- SendCarGate: " << j.dump() << std::endl;
      // std::string messageId = sendResult.getMsgId();
      // std::cout << "msg_cargate sent with ID: " << messageId << std::endl;
      log << "[" << getCurrentChinaTime() << "] " << j.dump() << std::endl;
      log.flush();
    }
    catch (MQException &e)
    {
      std::cout << e << endl;
    }
  }
}

void Producer::SendNIO(simapi &api, std::ofstream &log)
{
  double restLength = 0.0;
  double routeLength = 0.0;
  int routeCountDown = 0;
	static std::vector<long> roadIdList;
	static std::map<std::string, bool> route_section_ori;
  bool retRoute = api.SimAPI_GetRouteInfo(routeLength, routeCountDown, roadIdList, route_section_ori);
  bool retLength = api.SimAPI_GetRestLength(restLength, roadIdList, route_section_ori);

  if (!retLength || !retRoute)
  {
    return;
  }

  nlohmann::json j;
  j["RouteType"] = m_route_type; // 路线类型 1直行 2左传 3右转 4环岛第一出口驶出 5环岛第二出口驶出 6环岛第三出口驶出 7环岛第四出口驶出 10高速第一匝道右转 12高速继续直行 14高速左传 255目的地
  j["RouteDistance"] = static_cast<int>(std::round(restLength)); // 线路距离
  j["RouteDistanceTotal"] = static_cast<int>(std::round(routeLength)); // 线路总长
  j["RouteCountdownTotal"] = routeCountDown;  // 路线倒计时汇总

  MQMessage msg_nio(m_args.topic_nio, "*", j.dump());
  msg_nio.setKeys(m_args.topic_nio);
  // msg_nio.setProperty("TTL", std::to_string(m_TTL));
  try
  {
    SendResult sendResult = m_producer.send(msg_nio);
    // std::cout << "----- SendNIO: " << j.dump() << std::endl;
    // std::string messageId = sendResult.getMsgId();
    // std::cout << "msg_nio sent with ID: " << messageId << std::endl;
    log << "[" << getCurrentChinaTime() << "] " << j.dump() << std::endl;
    log.flush();
  }
  catch (MQException &e)
  {
    std::cout << e << endl;
  }
}

void Producer::SendLaneInfo(simapi &api, std::ofstream &log)
{
	std::unique_ptr<SimOne_Data_LaneInfo> pLaneInfo = std::make_unique<SimOne_Data_LaneInfo>();
	if (api.SimAPI_SensorLaneInfo(pLaneInfo.get()))
  {
    nlohmann::json j; 
    j["LaneAvailability"].emplace_back(1); // 道路可用性1 0 0 0
    j["LaneAvailability"].emplace_back(0);
    j["LaneAvailability"].emplace_back(0);
    j["LaneAvailability"].emplace_back(0);
    j["DriverLane"] = pLaneInfo->id; // 驾驶道路

    MQMessage msg_laneinfo(m_args.topic_laneinfo, "*", j.dump());
    msg_laneinfo.setKeys(m_args.topic_laneinfo);
    try
    {
      SendResult sendResult = m_producer.send(msg_laneinfo);
      // std::cout << "----- SendLaneInfo: " << j.dump() << std::endl;
      // std::string messageId = sendResult.getMsgId();
      // std::cout << "msg_laneinfo sent with ID: " << messageId << std::endl;
      log << "[" << getCurrentChinaTime() << "] " << j.dump() << std::endl;
      log.flush();
    }
    catch (MQException &e)
    {
      std::cout << e << endl;
    }
  }
}
void Producer::SendACC(simapi &api, std::ofstream &log)
{
	// std::unique_ptr<SimOne_Data_SensorDetections> pDetections = std::make_unique<SimOne_Data_SensorDetections>();
  std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();

  if (!api.SimAPI_GPS(pGps.get()))
  {
    return;
  }

  SSD::SimString mvLaneName;
  if (!api.SimAPI_GetLaneName({pGps->posX, pGps->posY, pGps->posZ}, mvLaneName))
  {
    return;
  }

  std::unique_lock<std::mutex> lock(mtx);
  cv.wait(lock, [&]() { return m_sync_vm_acc.load(); });

  if (0 == m_objVehicles.size())
  {
    // m_sim_vehiclemoving.acc_frame = 0;
    nlohmann::json j;
    j["VehicleAheadPosition"] = nlohmann::json::array();
    j["DistanceAdjustment"] = 0;           // 距离调整  0，3
    j["DistanceAdjustmentActive"] = false; // websocket_client::mDrivingLever.isCruiseOn; // 距离调整激活 false
    j["PredictiveControlType"] = 0;        // 预测控制类型 0
    j["PredictiveControlSpeed"] = 0;       // 预测控制速度 0，327

    j["SpeedLimit"] = websocket_client::mCruise_speed ? std::round(websocket_client::mCruise_speed * 3.6) : 333.0; // 速度限制 333.0 --mk/h
    j["Status"] = websocket_client::mCruise_valid ? 3 : 1;                                                                       // 状态 Green (unity 3 On/5 ReadyToGo); Wite (unity 1 Wait/2 Standby/4 Violating); Disapear (unity 0 Off, skip this status)
    j["VehicleAheadCountdown"] = 0.0;                                                                                            // minDist; ------------
    j["VehicleAheadID"] = 0;                                                                                                     // aheadVId; ------------
    j["VehicleOnEgoLaneID"] = 0;                                                                                                 // 100
    j["VehicleType"] = 1;

    MQMessage msg_acc(m_args.topic_acc, "*", j.dump());
    msg_acc.setKeys(m_args.topic_acc);
    try
    {
      SendResult sendResult = m_producer.send(msg_acc);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      log << "[" << getCurrentChinaTime() << "] " << j.dump() << std::endl;
      log.flush();
    }
    catch (MQException &e)
    {
      std::cout << e << endl;
    }
  }
  else
  {
    // while (!m_sync_vehiclemoving.load())
    // {
    //   std::this_thread::yield();
    // }
    // m_sync_vehiclemoving.store(true);

    int aheadVId = 0;
    double minDist = std::numeric_limits<double>::infinity();
    for (int i = 0; i < m_objVehicles.size(); i++)
    {
      double dist = 0.0;
      if (api.SimAPI_GetObjVehicleDist({pGps->posX, pGps->posY, pGps->posZ}, {m_objVehicles[i].pos_x, m_objVehicles[i].pos_y, m_objVehicles[i].pos_z}, mvLaneName, dist))
      {
        if (dist < minDist)
        {
          minDist = dist;
          aheadVId = m_objVehicles[i].id;
        }
      }
    }

    for (int i = 0; i < m_objVehicles.size(); i++)
    {
      nlohmann::json j;
      // j["frame"] = m_vehiclemoving_frame; // -------------------------- Debug
      j["VehicleAheadPosition"].emplace_back(m_objVehicles[i].zd_x);
      j["VehicleAheadPosition"].emplace_back(m_objVehicles[i].zd_y);
      j["VehicleAheadPosition"].emplace_back(m_objVehicles[i].zd_z);
      j["VehicleAheadPosition"].emplace_back(m_objVehicles[i].ori_x);
      j["VehicleAheadPosition"].emplace_back(m_objVehicles[i].ori_y);
      j["VehicleAheadPosition"].emplace_back(m_objVehicles[i].ori_z);

      j["DistanceAdjustment"] = 0;                                                                                                 // 距离调整  0，3
      j["DistanceAdjustmentActive"] = false;                                                                                       // websocket_client::mDrivingLever.isCruiseOn; // 距离调整激活 false
      j["PredictiveControlType"] = 0;                                                                                              // 预测控制类型 0
      j["PredictiveControlSpeed"] = 0;                                                                                             // 预测控制速度 0，327
      j["SpeedLimit"] = websocket_client::mCruise_speed ? std::round(websocket_client::mCruise_speed * 3.6) : 333.0; // 速度限制 333.0 --mk/h
      j["Status"] = websocket_client::mCruise_valid ? 3 : 1;                                                                       // 状态 Green (unity 3 On/5 ReadyToGo); Wite (unity 1 Wait/2 Standby/4 Violating); Disapear (unity 0 Off, skip this status)
      j["VehicleAheadCountdown"] = std::isinf(minDist) ? 0.0 : (float)(minDist);                                                   // minDist; ------------
      j["VehicleAheadID"] = aheadVId;                                                                                              // aheadVId; ------------
      j["VehicleOnEgoLaneID"] = m_objVehicles[i].id;
                                                                                      // 100
      j["VehicleType"] = m_objVehicles[i].type;                                                                                                 // 100

      MQMessage msg_acc(m_args.topic_acc, "*", j.dump());
      msg_acc.setKeys(m_args.topic_acc);
      try
      {
        SendResult sendResult = m_producer.send(msg_acc);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        log << "[" << getCurrentChinaTime() << "] frame: " << m_objVehicles[i].frame << " ----- SendACC: " << j.dump() << std::endl;
        log.flush();
      }
      catch (MQException &e)
      {
        std::cout << e << endl;
      }
    }

    // m_sync_vehiclemoving.store(false);
  }

  m_sync_vm_acc.store(false);
  cv.notify_all();
}

void Producer::SendCAR(simapi &api, std::ofstream &log)
{
  std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
  if (api.SimAPI_GPS(pGps.get()))
  {
	  // float velAngle = sqrt(pGps->angVelX*pGps->angVelX + pGps->angVelY*pGps->angVelY + pGps->angVelZ*pGps->angVelZ);
	  float vel = sqrt(pGps->velX*pGps->velX + pGps->velY*pGps->velY + pGps->velZ*pGps->velZ) * 3.6;
    nlohmann::json j;
    j["Speed"] = vel;

    MQMessage msg_car(m_args.topic_car, "*", j.dump());
    msg_car.setKeys(m_args.topic_car);
    // msg_car.setProperty("TTL", std::to_string(m_TTL));
    try
    {
      SendResult sendResult = m_producer.send(msg_car);
      // std::cout << "----- SendCAR: " << j.dump() << std::endl;
      // std::string messageId = sendResult.getMsgId();
      // std::cout << "msg_car sent with ID: " << messageId << std::endl;
      log << "[" << getCurrentChinaTime() << "] " << j.dump() << std::endl;
      log.flush();
    }
    catch (MQException &e)
    {
      std::cout << e << endl;
    }
  }
}

void Producer::SetAIOData(nlohmann::json &j, SimOne_Data_SensorDetections_Entry *pObj, SimOne_Data_TrafficLight *pTrafficLight)
{
  j["TrafficLightLuLAvailability"] = 1; // 红绿灯可用性
  j["CurrentPhase"] = 0; // 当前阶段，不是3，推荐速度会关掉，
  j["RecommendedSpeed"] = int(websocket_client::mCruise_speed * 3.6); // 推荐速度 acc is on, 主车的速度
  j["Type"] = m_trafficlight_type; //重定义类型: 0无 1左转 2直行左转 3直行 4直行右转 5右转 7直行右转左转 // pre类型 0左转 1左掉头 2左后转弯 3左前转弯 4直行 5右转 6右掉头 7右后转弯 8右前转弯
  // switch (m_route_type) 
  // {
  // case 2: // "左转"
  //   j["Type"] = 0;
  //   break;
  // case 1: // "直行"
  //   j["Type"] = 4;
  //   break;
  // case 3: // "右转"
  //   j["Type"] = 5;
  //   break;
  // case 5: // "环岛第二出口驶出"
  //   j["Type"] = 8;
  //   break;
  // case 6: // "环岛第三出口驶出"
  //   j["Type"] = 8;
  //   break;
  // case 7: // "环岛第四出口驶出"
  //   j["Type"] = 8;
  //   break;
  // case 10: // "高速第一匝道右转"
  //   j["Type"] = 5;
  //   break;
  // case 12: // "高速继续直行"
  //   j["Type"] = 4;
  //   break;
  // case 14: // "高速左转"
  //   j["Type"] = 0;
  //   break;
  // default:
  //   j["Type"] = 4;
  // }

  if (pObj != nullptr)
  {
    // nlohmann::json j;
    float distance = sqrt(pObj->relativePosX * pObj->relativePosX + pObj->relativePosY * pObj->relativePosY + pObj->relativePosZ * pObj->relativePosZ);
    j["TrafficLightDistance"] = static_cast<int>(std::round(distance)); // 红绿灯距离
    j["TrafficLightLuLAvailability"] = static_cast<int>(std::round(distance)) + 1; // 红绿灯可用性
    j["TrafficLightPositions"].emplace_back(pObj->posX); // 67.94531 pDetections->objects[i].relativePosX
    j["TrafficLightPositions"].emplace_back(pObj->posZ); // 85.71182 pDetections->objects[i].relativePosY
    j["TrafficLightPositions"].emplace_back(pObj->posY); // 0.0 pDetections->objects[i].relativePosZ
  }

  if (pTrafficLight != nullptr)
  {
    switch (pTrafficLight->status)
    {
    case ESimOne_TrafficLight_Status_Red:
    case ESimOne_TrafficLight_Status_RedBlink:
      j["TimeToGreen"] = pTrafficLight->countDown; // > 4
      if (websocket_client::mDrivingLever.isCruiseOn)
      {
        j["CurrentPhase"] = 1;
      }
      break;
    case ESimOne_TrafficLight_Status_Green:
    case ESimOne_TrafficLight_Status_GreenBlink:
      j["TimeToGreen"] = 0;
      if (websocket_client::mDrivingLever.isCruiseOn)
      {
        j["CurrentPhase"] = 3;
      }
      break;
    case ESimOne_TrafficLight_Status_Yellow:
    case ESimOne_TrafficLight_Status_YellowBlink:
    case ESimOne_TrafficLight_Status_Black:
      j["TimeToGreen"] = 0; // pTrafficLight->countDown + 44; // >0 && 3
      if (websocket_client::mDrivingLever.isCruiseOn)
      {
        j["CurrentPhase"] = 3;
      }
      break;
    case ESimOne_TrafficLight_Status_Invalid:
    default:
      j["TimeToGreen"] = 0;
    }
  }
}

void Producer::SendAIO(simapi &api, std::ofstream &log)
{
  std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
	std::unique_ptr<SimOne_Data_SensorDetections> pDetections = std::make_unique<SimOne_Data_SensorDetections>();
	std::unique_ptr<SimOne_Data_TrafficLight> pTrafficLight = std::make_unique<SimOne_Data_TrafficLight>();
  bool gotTrafficLight = false;
  // bool inJunc = false;
  // static int preDist = 0;
  // int curDist = 0;

  // static nlohmann::json preJ;
  static nlohmann::json j;
  static int preLightId = 0;

  if (!api.SimAPI_GPS(pGps.get()))
  {
    return;
  }

  SSD::SimString mvLaneName;
  if (!api.SimAPI_GetLaneName({pGps->posX, pGps->posY, pGps->posZ}, mvLaneName))
  {
    return;
  }

  // std::vector<simapi::MainTrafficLight_t> lights;
  // api.SimAPI_GetTrafficLightList(mvLaneName, lights);
  // // std::cout << "-------------------------lights size: " << lights.size() << std::endl;
  // for (int i=0; i<lights.size(); i++)
  // {
  //   if (api.SimAPI_GetTrafficLight(lights[i].id, pTrafficLight.get()))
  //   {
  //     // std::cout << "--------------------------light[" << i << "].id: " << lights[i].id << std::endl;
  //     // std::cout << "--------------------------isMainVehicleNextTrafficLight " << pTrafficLight->isMainVehicleNextTrafficLight << std::endl;
  //     if (pTrafficLight->isMainVehicleNextTrafficLight)
  //     {
  //       gotTrafficLight = true;
  //       inJunc = true;
  //       float distance = sqrt((pGps->posX - lights[i].pt.x) * (pGps->posX - lights[i].pt.x) +
  //                             (pGps->posY - lights[i].pt.y) * (pGps->posY - lights[i].pt.y) +
  //                             (pGps->posZ - lights[i].pt.z) * (pGps->posZ - lights[i].pt.z));

  //       nlohmann::json j;
  //       j["TrafficLightLuLAvailability"] = 1;                               // 红绿灯可用性
  //       j["TrafficLightDistance"] = static_cast<int>(std::round(distance)); // 红绿灯距离
  //       curDist = j["TrafficLightDistance"];
  //       j["CurrentPhase"] = 0; // 当前阶段，不是3，推荐速度会关掉，

  //       switch (pTrafficLight->status)
  //       {
  //       case ESimOne_TrafficLight_Status_Red:
  //       case ESimOne_TrafficLight_Status_RedBlink:
  //         j["TimeToGreen"] = pTrafficLight->countDown;
  //         if (websocket_client::mDrivingLever.isCruiseOn)
  //         {
  //           j["CurrentPhase"] = 1;
  //         }
  //         break;
  //       case ESimOne_TrafficLight_Status_Green:
  //       case ESimOne_TrafficLight_Status_GreenBlink:
  //         j["TimeToGreen"] = 0;
  //         if (websocket_client::mDrivingLever.isCruiseOn)
  //         {
  //           j["CurrentPhase"] = 3;
  //         }
  //         break;
  //       case ESimOne_TrafficLight_Status_Yellow:
  //       case ESimOne_TrafficLight_Status_YellowBlink:
  //       case ESimOne_TrafficLight_Status_Black:
  //         j["TimeToGreen"] = pTrafficLight->countDown + 44;
  //         break;
  //       case ESimOne_TrafficLight_Status_Invalid:
  //       default:
  //         j["TimeToGreen"] = 0;
  //       }

  //       j["RecommendedSpeed"] = int(websocket_client::cruise_speed); // 推荐速度 acc is on, 主车的速度
  //       switch (m_route_type)                                   // 类型 0左传 1左掉头 2左后转弯 3左前转弯 4直行 5右转 6右掉头 7右后转弯 8右前转弯
  //       {
  //       case 2: // "左转"
  //         j["Type"] = 0;
  //         break;
  //       case 1: // "直行"
  //         j["Type"] = 4;
  //         break;
  //       case 5: // "环岛第二出口驶出"
  //         j["Type"] = 8;
  //         break;
  //       case 6: // "环岛第三出口驶出"
  //         j["Type"] = 8;
  //         break;
  //       case 7: // "环岛第四出口驶出"
  //         j["Type"] = 8;
  //         break;
  //       case 10: // "高速第一匝道右转"
  //         j["Type"] = 5;
  //         break;
  //       case 12: // "高速继续直行"
  //         j["Type"] = 4;
  //         break;
  //       case 14: // "高速左转"
  //         j["Type"] = 0;
  //         break;
  //       default:
  //         j["Type"] = 4;
  //       }
  //       j["TrafficLightPositions"].emplace_back(lights[i].pt.x - pGps->posX);
  //       j["TrafficLightPositions"].emplace_back(lights[i].pt.y - pGps->posY);
  //       j["TrafficLightPositions"].emplace_back(lights[i].pt.z - pGps->posZ);

  //       MQMessage msg_aio(m_args.topic_aio, "*", j.dump());
  //       msg_aio.setKeys(m_args.topic_aio);
  //       try
  //       {
  //         SendResult sendResult = m_producer.send(msg_aio);
  //         std::cout << "----- SendAIO: " << j.dump() << std::endl;
  //         // std::string messageId = sendResult.getMsgId();
  //         // std::cout << "msg_aio sent with ID: " << messageId << std::endl;
  //         log << "[" << pGps->timestamp << "]" << j.dump() << std::endl;
  //         log.flush();
  //       }
  //       catch (MQException &e)
  //       {
  //         std::cout << e << endl;
  //       }
  //     }
  //   }
  // }

  if (api.SimAPI_GetSensorDetections(pDetections.get()))
  {
    // std::cout << "pDetections->objectSize: " << pDetections->objectSize << std::endl;
	  for (int i=0; i<pDetections->objectSize; i++)
    {
      if (ESimOne_Obstacle_Type_TrafficLight == pDetections->objects[i].type)
      {
        // std::cout << "------------ ESimOne_Obstacle_Type_TrafficLight !" << std::endl;
        if (api.SimAPI_GetTrafficLight(pDetections->objects[i].id, pTrafficLight.get()))
        {
          // std::cout << "------------ objects[" << i << "].id: " << pDetections->objects[i].id << ", objects pos : [" << pDetections->objects[i].posX << ", "  << pDetections->objects[i].posY << ", "  << pDetections->objects[i].posZ << "]"  << std::endl;
          // std::cout << "--------------------------isMainVehicleNextTrafficLight " << pTrafficLight->isMainVehicleNextTrafficLight << std::endl;

	        if (pTrafficLight->isMainVehicleNextTrafficLight)
          {
            gotTrafficLight = true;
            preLightId = pDetections->objects[i].id;
            j = {};
            SetAIOData(j, &pDetections->objects[i], pTrafficLight.get());

            // inJunc = true;
            // MQMessage msg_aio(m_args.topic_aio, "*", j.dump());
            // msg_aio.setKeys(m_args.topic_aio);
            // try
            // {
            //   // preJ = j;
            //   SendResult sendResult = m_producer.send(msg_aio);
            //   std::cout << "----- isMainVehicleNextTrafficLight SendAIO: " << j.dump() << std::endl;
            //   // std::string messageId = sendResult.getMsgId();
            //   // std::cout << "msg_aio sent with ID: " << messageId << std::endl;

            //   log << "[" << getCurrentChinaTime() << "] " << j.dump() << std::endl;
            //   log.flush();
            // }
            // catch (MQException &e)
            // {
            //   std::cout << e << endl;
            // }
          }
        }
      }
    }

  }

    // bool byDescend = false;
    // if (gotTrafficLight && inJunc)
    // {
    //   if (curDist > preDist)
    //   {
    //     byDescend = true;
    //   }
    // }
    // preDist  = curDist;
    // if ((gotTrafficLight && !inJunc) || byDescend)


  // std::cout << "--------------------------m_sim_waiting_area_sig" << m_sim_waiting_area_sig<< std::endl;
  if (m_sim_waiting_area_sig)
  {
    gotTrafficLight = true;

    if (api.SimAPI_GetTrafficLight(preLightId, pTrafficLight.get()))
    {
      SetAIOData(j, nullptr, pTrafficLight.get());
    }
  }

  if (!gotTrafficLight)
  {
    // if (api.SimAPI_IsInJunction(mvLaneName))
    // {
    //   MQMessage msg_aio(m_args.topic_aio, "*", preJ.dump());
    //   msg_aio.setKeys(m_args.topic_aio);
    //   try
    //   {
    //     SendResult sendResult = m_producer.send(msg_aio);
    //     // std::cout << "----- In InJunction SendAIO: " << preJ.dump() << std::endl;
    //     // std::string messageId = sendResult.getMsgId();
    //     // std::cout << "msg_aio sent with ID: " << messageId << std::endl;

    //     log << "[" << getCurrentChinaTime() << "] " << preJ.dump() << std::endl;
    //     log.flush();
    //   }
    //   catch (MQException &e)
    //   {
    //     std::cout << e << endl;
    //   }
    // }
    // else
    // {

    // int cout = 1;
    // while (cout > 0)
    // {
    gotTrafficLight = false;
    // nlohmann::json j;
    j["TrafficLightLuLAvailability"] = 1;
    j["TrafficLightDistance"] = 0;
    j["TimeToGreen"] = 3; // 3、2、1 任意值unity交通灯置灰
    j["RecommendedSpeed"] = 0;
    j["Type"] = 4;
    j["CurrentPhase"] = 1;
    j["TrafficLightPositions"] = nlohmann::json::array();
    // j["TrafficLightPositions"].emplace_back(0.1);
    // j["TrafficLightPositions"].emplace_back(0.1);
    // j["TrafficLightPositions"].emplace_back(0.1);
  }

  MQMessage msg_aio(m_args.topic_aio, "*", j.dump());
  msg_aio.setKeys(m_args.topic_aio);
  // msg_aio.setProperty("TTL", std::to_string(m_TTL));
  try
  {
    // preJ = j;
    SendResult sendResult = m_producer.send(msg_aio);
    // std::cout << "----- SendAIO: " << j.dump() << std::endl;
    // std::string messageId = sendResult.getMsgId();
    // std::cout << "msg_aio sent with ID: " << messageId << std::endl;
    log << "[" << getCurrentChinaTime() << "] " << j.dump() << std::endl;
    log.flush();
  }
  catch (MQException &e)
  {
    std::cout << e << endl;
  }
  // cout -= 1;

  // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // }
  // }
}

float get_degree(float percent)
{
  if (percent >= 0.9)
  {
    return 5000;
  }
  else if (percent >= 0.6)
  {
    return 500;
  }
  else if (percent >= 0.3)
  {
    return 50;
  }

  return 0;
}
void Producer::SendLAB(simapi &api, std::ofstream &log)
{
  std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
  if (api.SimAPI_GPS(pGps.get()))
  {
    nlohmann::json j;
    j["AcceleratorPedalPosition"] = get_degree(pGps->throttle); // 加速器踏板位置
    j["BrakePedalPosition"] = -get_degree(pGps->brake); // 制动踏板位置

    MQMessage msg_lab(m_args.topic_lab, "*", j.dump());
    msg_lab.setKeys(m_args.topic_lab);
    // msg_lab.setProperty("TTL", std::to_string(m_TTL));
    try
    {
      SendResult sendResult = m_producer.send(msg_lab);
      // std::cout << "----- SendLAB: " << j.dump() << std::endl;
      log << "[" << getCurrentChinaTime() << "] " << j.dump() << std::endl;
      log.flush();
    }
    catch (MQException &e)
    {
      std::cout << e << endl;
    }
  }
}

void Producer::SendDriveSelect(simapi &api, std::ofstream &log)
{
  nlohmann::json j;
  j["SetSetupProfile"] = 0; // 设置配置文件
  j["SetupProfile"] = 0; // 配置文件 1 3 5 7 9
 
  MQMessage msg_driveselect(m_args.topic_driveselect, "*", j.dump());
  msg_driveselect.setKeys(m_args.topic_driveselect);
  // msg_driveselect.setProperty("TTL", std::to_string(m_TTL));
  try
  {
    SendResult sendResult = m_producer.send(msg_driveselect);
    // std::cout << "----- SendDriveSelect: " << j.dump() << std::endl;
    log << "[" << getCurrentChinaTime() << "] " << j.dump() << std::endl;
    log.flush();
  }
  catch (MQException &e)
  {
    std::cout << e << endl;
  }
}

void Producer::JudgeEventCB(const char* mainVehicleId, SimOne_Data_JudgeEvent *judgeEventDetailInfo)
{
	// std::cout << "hostVehicle: " << judgeEventDetailInfo->hostVehicle << std::endl;
	// std::cout << "time: " << judgeEventDetailInfo->time << std::endl;
	// std::cout << "actualValue: " << judgeEventDetailInfo->actualValue << std::endl;
	// std::cout << "expectOp: " << judgeEventDetailInfo->expectOp << std::endl;
	// std::cout << "expectValue1: " << judgeEventDetailInfo->expectValue1 << std::endl;
	// std::cout << "expectValue2: " << judgeEventDetailInfo->expectValue2 << std::endl;
	// std::cout << "judgeId: " << judgeEventDetailInfo->judgeId << std::endl;
	// std::cout << "judgeType: " << judgeEventDetailInfo->judgeType << std::endl;
	// std::cout << "requireProperty: " << judgeEventDetailInfo->requireProperty << std::endl;
	// std::cout << "taskId: " << judgeEventDetailInfo->taskId << std::endl;
	// std::cout << "valueType: " << judgeEventDetailInfo->valueType << std::endl;
	// std::cout << "version: " << judgeEventDetailInfo->version << std::endl;

  if (std::string(judgeEventDetailInfo->judgeType) == "collision")
  {
    m_collided = true;
  }
}

void Producer::ScenarioEventCB(const char* source, const char* target, const char* type, const char* content)
{
  std::cout << "source:" << std::string(source) << ", target:" << std::string(target) << ", type:" << std::string(type) << ", content:" << std::string(content) << std::endl;

  // 路线类型 1直行 2左传 3右转 4环岛第一出口驶出 5环岛第二出口驶出 6环岛第三出口驶出 7环岛第四出口驶出 10高速第一匝道右转 12高速继续直行 14高速左传 255目的地
  // 交通灯类型: 0无 1左转 2直行左转 3直行 4直行右转 5右转 7直行右转左转
  if (0 == std::string(content).compare("直行"))
  {
    m_route_type = 1;
    m_trafficlight_type = 3;
  }
  else if (0 == std::string(content).compare("左转"))
  {
    m_route_type = 2;
    m_trafficlight_type = 1;
  }
  else if (0 == std::string(content).compare("右转"))
  {
    m_route_type = 3;
    m_trafficlight_type = 5;
  }
  else if (0 == std::string(content).compare("环岛第一出口驶出"))
  {
    m_route_type = 4;
  }
  else if (0 == std::string(content).compare("环岛第二出口驶出"))
  {
    m_route_type = 5;
  }
  else if (0 == std::string(content).compare("环岛第三出口驶出"))
  {
    m_route_type = 6;
  }
  else if (0 == std::string(content).compare("环岛第四出口驶出"))
  {
    m_route_type = 7;
  }
  else if (0 == std::string(content).compare("高速第一匝道右转"))
  {
    m_route_type = 10;
  }
  else if (0 == std::string(content).compare("高速继续直行"))
  {
    m_route_type = 12;
  }
  else if (0 == std::string(content).compare("高速左转"))
  {
    m_route_type = 14;
  }
  else if (0 == std::string(content).compare("目的地"))
  {
    m_route_type = 255;
  }
  else if (0 == std::string(content).compare("直行左转"))
  {
    m_trafficlight_type = 2;
  }
  else if (0 == std::string(content).compare("直行右转"))
  {
    m_trafficlight_type = 4;
  }
  else if (0 == std::string(content).compare("直行左转右转"))
  {
    m_trafficlight_type = 6;
  }
  else if (0 == std::string(content).compare("左转灯"))
  {
    m_sim_driv_turn_light = 1;
  }
  else if (0 == std::string(content).compare("右转灯"))
  {
    m_sim_driv_turn_light = 2;
  }
  else if (0 == std::string(content).compare("无转向灯"))
  {
    m_sim_driv_turn_light = 0;
  }
  else if (0 == std::string(content).compare("驶入待行区"))
  {
    m_sim_waiting_area_sig = 1;
  }
  else if (0 == std::string(content).compare("驶出待行区"))
  {
    m_sim_waiting_area_sig = 0;
  }
  else
  {
    m_sim_driv_turn_light = 0;
    m_sim_waiting_area_sig = 0;
    m_route_type = 0;
    m_trafficlight_type = 0;
  }
}

void Producer::SendCollision(simapi &api, int minDistThre, std::ofstream &log)
{
  std::unique_ptr<SimOne_Data_SensorDetections> pDetections = std::make_unique<SimOne_Data_SensorDetections>();
  std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();

  if (!api.SimAPI_GPS(pGps.get()))
  {
    return;
  }

  if (api.SimAPI_GetSensorDetections(pDetections.get()))
  {
    // std::cout << "pDetections->objectSize : " <<  pDetections->objectSize << std::endl;
    nlohmann::json j;
    j["Left"] = false;
    j["Right"] = false;

    for (int i = 0; i < pDetections->objectSize; i++)
    {
      if ((pDetections->objects[i].relativeRotZ < -M_PI / 2) || (pDetections->objects[i].relativeRotZ > M_PI / 2))
      {
        continue;
      }

      if (j["Left"] && j["Right"])
      {
        break;
      }

      switch (pDetections->objects[i].type)
      {
      case ESimOne_Obstacle_Type_Unknown:
      case ESimOne_Obstacle_Type_Pedestrian:
      case ESimOne_Obstacle_Type_Pole:
      case ESimOne_Obstacle_Type_Static:
      case ESimOne_Obstacle_Type_Fence:
      case ESimOne_Obstacle_Type_RoadMark:
      case ESimOne_Obstacle_Type_TrafficSign:
      case ESimOne_Obstacle_Type_TrafficLight:
      case ESimOne_Obstacle_Type_GuardRail:
      case ESimOne_Obstacle_Type_SpeedLimitSign:
      case ESimOne_Obstacle_Type_BicycleStatic:
      case ESimOne_Obstacle_Type_RoadObstacle:
        continue;
      case ESimOne_Obstacle_Type_Car:
      case ESimOne_Obstacle_Type_Bicycle:
      case ESimOne_Obstacle_Type_Rider:
      case ESimOne_Obstacle_Type_Truck:
      case ESimOne_Obstacle_Type_Bus:
      case ESimOne_Obstacle_Type_SpecialVehicle:
      case ESimOne_Obstacle_Type_Motorcycle:
      case ESimOne_Obstacle_Type_Dynamic:
        double dist = std::sqrt(std::pow(pDetections->objects[i].relativePosX, 2) + std::pow(pDetections->objects[i].relativePosY, 2) + std::pow(pDetections->objects[i].relativePosZ, 2));
        // std::cout << "dist: " << dist << std::endl;
        // std::cout << "object : refX :" << pDetections->objects[i].relativePosX << ", refY : " << pDetections->objects[i].relativePosY<< std::endl;

        if (pDetections->objects[i].relativePosX < 6.0 && pDetections->objects[i].relativePosY < 6.0 && pDetections->objects[i].relativePosY > 0)
        {
          if (dist < minDistThre)
          {
            j["Left"] = true;
            continue;
          }
        }

        if (pDetections->objects[i].relativePosX < 6.0 && pDetections->objects[i].relativePosY > -6.0 && pDetections->objects[i].relativePosY < 0)
        {
          if (dist < minDistThre)
          {
            j["Right"] = true;
            continue;
          }
        }
        continue;
      }
    }

    MQMessage msg_scp(m_args.topic_collision, "*", j.dump());
    msg_scp.setKeys(m_args.topic_collision);
    // msg_scp.setProperty("TTL", std::to_string(m_TTL));
    try
    {
      SendResult sendResult = m_producer.send(msg_scp);
      // std::cout << "----- SendCollision: " << j.dump() << std::endl;
      log << "[" << getCurrentChinaTime() << "] " << j.dump() << std::endl;
      log.flush();
    }
    catch (MQException &e)
    {
      std::cerr << e << endl;
    }
  }
}

void Producer::GpsUpdateCB(const char* mainVehicleId, SimOne_Data_Gps *pGps)
{
  // static std::ofstream log_gps_cb;
  // static bool run_once = true;
  // if (run_once)
  // {
  //   run_once = false;
  //   log_gps_cb.open("log_gps_cb.txt", std::ios::out | std::ios::trunc);
  // }

  // static std::map<int, VehicleMoving_SimData> vehicle_moving_map;
  // static std::mutex mtx_gps;
  VehicleMoving_SimData data;
  data.frame = pGps->frame;
  data.posX = pGps->posX;
  data.posY = pGps->posY;
  data.posZ = pGps->posZ;
  data.oriX = pGps->oriX;
  data.oriY = pGps->oriY;
  data.oriZ = pGps->oriZ;
  m_vehicle_moving_map[pGps->frame] = data;

  // std::cout << "--------------------------------- m_vehicle_moving_map size: " << m_vehicle_moving_map.size() << std::endl;

/*
  if (!m_acc_sync_ok.load())
  {
    while (m_sync_vehiclemoving_frame.load())
    {
      std::this_thread::yield();
    }
    m_sync_vehiclemoving_frame.store(true);
    m_vehiclemoving_frame = pGps->frame;
    m_sync_vehiclemoving_frame.store(false);
  }

  if (!m_gps_sync_ok.load() && m_acc_sync_ok.load())
  {
    auto it = vehicle_moving_map.find(m_vehiclemoving_frame);
    if (it != vehicle_moving_map.end())
    {
      // gps_frame = m_vehiclemoving_frame;
      while (m_sync_gps.load())
      {
        std::this_thread::yield();
      }
      m_sync_gps.store(true);
      m_vehiclemoving = vehicle_moving_map[m_vehiclemoving_frame];
      // log_gps_cb << "------------ frame: " << gps_frame << std::endl;
      // log_gps_cb << "pos: [" << m_vehiclemoving.posX << ", " << m_vehiclemoving.posY << ", " << m_vehiclemoving.posZ << "]" << std::endl;
      // log_gps_cb << "ori: [" << m_vehiclemoving.oriX << ", " << m_vehiclemoving.oriY << ", " << m_vehiclemoving.oriZ << "]" << std::endl;
      // log_gps_cb.flush();
      m_sync_gps.store(false);

      m_gps_sync_ok.store(true);
    }
  }
*/



/*
  if (gps_frame != m_vehiclemoving_frame)
  {
    if (gps_frame == 0)
    {
      if (!m_acc_sync_ok.load())
      {
        return;
      }
      gps_frame = m_vehiclemoving_frame;
      return;
    }

    // while (true)
    // {
      if (!m_acc_sync_ok.load())
      {
        // std::this_thread::yield();
        return;
      }

      auto it = vehicle_moving_map.find(m_vehiclemoving_frame);
      if (it != vehicle_moving_map.end())
      {
        gps_frame = m_vehiclemoving_frame;
        while (m_sync_gps.load())
        {
          std::this_thread::yield();
        }
        m_sync_gps.store(true);
        m_vehiclemoving = vehicle_moving_map[gps_frame];
        // log_gps_cb << "------------ frame: " << gps_frame << std::endl;
        // log_gps_cb << "pos: [" << m_vehiclemoving.posX << ", " << m_vehiclemoving.posY << ", " << m_vehiclemoving.posZ << "]" << std::endl;
        // log_gps_cb << "ori: [" << m_vehiclemoving.oriX << ", " << m_vehiclemoving.oriY << ", " << m_vehiclemoving.oriZ << "]" << std::endl;
        // log_gps_cb.flush();
        m_sync_gps.store(false);
        // break;
      }
    else
    {
      return;
    //     // if (m_vehiclemoving_frame < pGps->frame)
    //     // {
    //     while (m_sync_vehiclemoving_frame.load())
    //     {
    //       std::this_thread::yield();
    //     }
    //     m_sync_vehiclemoving_frame.store(true);
    //     m_vehiclemoving_frame = pGps->frame;
    //     gps_frame = m_vehiclemoving_frame;
    //     m_sync_vehiclemoving_frame.store(false);
    //     m_acc_sync_ok.store(false);
    //     continue;

    //     // while (m_sync_gps.load())
    //     // {
    //     //   std::this_thread::yield();
    //     // }
    //     // m_sync_gps.store(true);
    //     // m_vehiclemoving = vehicle_moving_map[gps_frame];
    //     // m_sync_gps.store(false);
    //     // // }
    //     // m_acc_sync_ok.store(true);
    //   }
    }

    m_gps_sync_ok.store(true);
  }
*/

  {
    std::lock_guard<std::mutex> lock(mtx_gps);
    auto it = m_vehicle_moving_map.begin();
    while (it != m_vehicle_moving_map.end())
    {
      // log_detections_cb << "frame: " << it->first << std::endl;
      // log_detections_cb.flush();
      if (it->first < m_vehiclemoving_frame - 30)
      {
        auto nextIt = std::next(it);
        m_vehicle_moving_map.erase(it);
        it = nextIt;
      }
      else
      {
        break;
      }
    }
  }
}

void Producer::SendCaseType(simapi &api, std::ofstream &log)
{
  SimOne_Data_Control_Mode controlMode;
	if(!SimOneAPI::GetControlMode(api.mainVehicleId.c_str(), &controlMode))
	{
		std::cout << "GetControlMode Failed!" << std::endl;
		return;
	}

  nlohmann::json j;
	switch(controlMode.controlMode)
	{
		case ESimOne_Control_Mode_API:
		case ESimOne_Control_Mode_Unknown:
      m_case_type = 1;
      j["Type"] = 1;
			break;
			// std::cout << "ESimOne_Control_Mode_Unknown" << std::endl;
			// std::cout << "ESimOne_Control_Mode_Manual" << std::endl;
		case ESimOne_Control_Mode_Auto:
      m_case_type = 2;
      j["Type"] = 2;
			// std::cout << "ESimOne_Control_Mode_Auto" << std::endl;
			break;
		case ESimOne_Control_Mode_Manual:
		default:
      m_case_type = 0;
      j["Type"] = 0;
			break;
			// std::cout << "Invalid ControlMode!" << std::endl;
	}

  MQMessage msg_case_type(m_args.topic_case_type, "*", j.dump());
  msg_case_type.setKeys(m_args.topic_case_type);
  try
  {
    SendResult sendResult = m_producer.send(msg_case_type);
    // std::cout << "----- SendCaseType: " << j.dump() << std::endl;
    log << "[" << getCurrentChinaTime() << "] " << j.dump() << std::endl;
    log.flush();
  }
  catch (MQException &e)
  {
    std::cerr << e << endl;
  }
}

void Producer::SensorDetectionsUpdateCB(const char* mainVehicleId, const char *sensorId, SimOne_Data_SensorDetections *pDetections)
{
  // static std::ofstream log_detections_cb;
  // static bool run_once = true;
  // if (run_once)
  // {
  //   run_once = false;
  //   log_detections_cb.open("log_sensordetections_cb.txt", std::ios::out | std::ios::trunc);
  // }

  // static std::mutex mtx_detections;
  std::vector<ACC_ObjVehicle> obj_vehicles;
  // static std::map<int, std::vector<ACC_ObjVehicle>> obj_vehicles_map;
  
  // log_detections_cb << "------------ frame: " << pDetections->frame << std::endl;
  // log_detections_cb << "objectSize: " << pDetections->objectSize << std::endl;
  for (int i = 0; i < pDetections->objectSize; i++)
  {
    switch (pDetections->objects[i].type)
    {
     case ESimOne_Obstacle_Type_Unknown:
     case ESimOne_Obstacle_Type_Pole:
     case ESimOne_Obstacle_Type_Static:
     case ESimOne_Obstacle_Type_Fence:
     case ESimOne_Obstacle_Type_RoadMark:
     case ESimOne_Obstacle_Type_TrafficSign:
     case ESimOne_Obstacle_Type_TrafficLight:
     case ESimOne_Obstacle_Type_GuardRail:
     case ESimOne_Obstacle_Type_SpeedLimitSign:
     case ESimOne_Obstacle_Type_BicycleStatic:
     case ESimOne_Obstacle_Type_RoadObstacle:
     case ESimOne_Obstacle_Type_Dynamic:
     case ESimOne_Obstacle_Type_Pedestrian:
       continue;
     case ESimOne_Obstacle_Type_Car:
     case ESimOne_Obstacle_Type_Bicycle:
     case ESimOne_Obstacle_Type_Rider:
     case ESimOne_Obstacle_Type_Truck:
     case ESimOne_Obstacle_Type_Bus:
     case ESimOne_Obstacle_Type_SpecialVehicle:
     case ESimOne_Obstacle_Type_Motorcycle:
     {
        if (pDetections->objects[i].relativePosX > 0) // front vehicle
        {
          ACC_ObjVehicle objV;
          objV.frame = pDetections->frame;
          switch (pDetections->objects[i].type)
          {
            default:
            case ESimOne_Obstacle_Type_Car:
            case ESimOne_Obstacle_Type_Bus:
            case ESimOne_Obstacle_Type_SpecialVehicle:
              objV.type = 1;
              break;
            case ESimOne_Obstacle_Type_Bicycle:
            case ESimOne_Obstacle_Type_Rider:
              objV.type = 4;
              break;
            case ESimOne_Obstacle_Type_Truck:
              objV.type = 2;
              break;
            case ESimOne_Obstacle_Type_Motorcycle:
              objV.type = 13;
              break;
          }
          objV.pos_x = pDetections->objects[i].posX;
          objV.pos_y = pDetections->objects[i].posY;
          objV.pos_z = pDetections->objects[i].posZ;
          objV.id = pDetections->objects[i].id;
          objV.zd_x = pDetections->objects[i].posX;                          // Unity x = (od X)   as mentioned in mv ox
          objV.zd_y = 0.0;                                                   // pDetections->objects[i].posZ; // Unity y = (od Z)   as mentioned in mv ox
          objV.zd_z = pDetections->objects[i].posY;                          // Unity z = (od Y)   as mentioned in mv ox
          objV.ori_x = -pDetections->objects[i].oriX;                        // Unity oriX = -(od oriX)
          objV.ori_y = 90.0 - (pDetections->objects[i].oriZ * 180.0 / M_PI); // Unity oriY = 90 - (od oriZ)
          objV.ori_z = -pDetections->objects[i].oriY;                        // Unity oriZ = -(od oriY)

          // log_detections_cb << "[" << getCurrentChinaTime() << "] objId: " << objV.id << "  pos: [" << objV.pos_x << "," << objV.pos_y << "," << objV.pos_z << "] " << std::endl;
          obj_vehicles.emplace_back(objV);
        }
        // log_detections_cb.flush();
     }
     break;
     default:;
    }
  }

  {
    std::lock_guard<std::mutex> lock(mtx_detections);
    auto it = m_obj_vehicles_map.begin();
    while (it != m_obj_vehicles_map.end())
    {
      if (it->first < m_vehiclemoving_frame - 30)
      {
        auto nextIt = std::next(it);
        m_obj_vehicles_map.erase(it);
        it = nextIt;
      }
      else
      {
        break;
      }
    }

    m_obj_vehicles_map[pDetections->frame] = obj_vehicles;
  }

  while (m_sync_vehiclemoving_frame.load())
  {
    std::this_thread::yield();
  }
  m_sync_vehiclemoving_frame.store(true);
  m_vehiclemoving_frame = pDetections->frame;
  m_sync_vehiclemoving_frame.store(false);

  // std::cout << "--------------------------------- m_obj_vehicles_map size: " << m_obj_vehicles_map.size() << std::endl;

  // ------------ Debug ------------
  // if (obj_vehicles_map[pDetections->frame].size() > 0)
  // {
  //   log_detections_cb << "------------ frame: " << pDetections->frame << std::endl;
  //   log_detections_cb << "obj_vehicles size: " << obj_vehicles_map[pDetections->frame].size() << std::endl;
  //   for (int i=0; i<obj_vehicles_map[pDetections->frame].size(); i++)
  //   {
  //     log_detections_cb << "objVehicles[" << i << "].id: " << obj_vehicles_map[pDetections->frame][i].id << std::endl;
  //     log_detections_cb << "objVehicles[" << i << "].pos: [" <<
  //       obj_vehicles_map[pDetections->frame][i].pos_x << ", " <<
  //       obj_vehicles_map[pDetections->frame][i].pos_y << ", " <<
  //       obj_vehicles_map[pDetections->frame][i].pos_z << "]" << std::endl;
  //     log_detections_cb << "objVehicles[" << i << "].ori: [" <<
  //       obj_vehicles_map[pDetections->frame][i].ori_x << ", " <<
  //       obj_vehicles_map[pDetections->frame][i].ori_y << ", " <<
  //       obj_vehicles_map[pDetections->frame][i].ori_z << "]" << std::endl;
  //     log_detections_cb << "objVehicles[" << i << "].zd: [" <<
  //       obj_vehicles_map[pDetections->frame][i].zd_x << ", " <<
  //       obj_vehicles_map[pDetections->frame][i].zd_y << ", " <<
  //       obj_vehicles_map[pDetections->frame][i].zd_z << "]" << std::endl;
  //   }
  //   log_detections_cb.flush();
  // }
  
  // if (obj_vehicles_map.size() > 100)
  // {
  //   std::lock_guard<std::mutex> lock(mtx_detections);
  //   auto it = obj_vehicles_map.begin();
  //   for (int i=0; i<60 && it!=obj_vehicles_map.end(); i++)
  //   {
  //     log_detections_cb << "sensorId: " << sensorId << "  frame: " << it->first << std::endl;
  //     log_detections_cb.flush();
  //     auto nextIt = std::next(it);
  //     obj_vehicles_map.erase(it);
  //     it = nextIt;
  //   }
  // }
}

// ------ Consumer ------
TpsReportService::TpsReportService() : tps_interval_(1), quit_flag_(false), tps_count_(0){}
TpsReportService::~TpsReportService()
{
  quit_flag_.store(true);
  if (tps_thread_ == nullptr)
  {
    std::cout << "tps_thread_ is null" << std::endl;
    return;
  }
  if (tps_thread_->joinable())
  {
    tps_thread_->join();
  }
}
void TpsReportService::start()
{
  if (tps_thread_ != nullptr)
  {
    std::cout << "tps_thread_ is not null" << std::endl;
    return;
  }
  tps_thread_.reset(new std::thread(std::bind(&TpsReportService::TpsReport, this)));
}
void TpsReportService::Increment() { ++tps_count_; }
void TpsReportService::TpsReport()
{
  while (!quit_flag_.load())
  {
    std::this_thread::sleep_for(tps_interval_);
    std::cout << "tps: " << tps_count_.load() << std::endl;
    tps_count_.store(0);
  }
}

Listener::Listener(TpsReportService &tps): m_tps(tps) {}
Listener::~Listener() {}

ConsumeStatus Listener::consumeMessage(const std::vector<MQMessageExt>& msgs)
{
  // std::cout << "-=-=-=-=-=-= msg size: " << msgs.size() << std::endl;
  for (size_t i = 0; i < msgs.size(); ++i)
  {
    MQMessageExt mqext = msgs[i];
    // std::cout << "====== mq#:" << i << std::endl;
    // std::cout << "queue id: " << mqext.getQueueId() << std::endl;
    // std::cout << "msg id: " << mqext.getMsgId() << std::endl;
    // std::cout << "store size: " << mqext.getStoreSize() << std::endl;
    // std::cout << "mq message: " << mqext.MQMessage::toString() << std::endl;
    std::string str(mqext.MQMessage::toString());
    auto s = str.find_first_of('=', 0)+1;
    auto e = str.find_first_of(',', 0);
    if (s != std::string::npos || e != std::string::npos)
    {
      if (str.substr(s, e-s).compare("AIR") == 0)
      {
        nlohmann::json j = nlohmann::json::parse(mqext.MQMessage::getBody());
        // std::cout << j.dump() << std::endl;
      }
    }

    m_tps.Increment();
  }

  return CONSUME_SUCCESS;
}

Consumer::Consumer(SendAndConsumerArgs &args): m_args(args), m_consumer("unique_group_name"), m_listener(m_tps)
{
  m_consumer.setNamesrvAddr(m_args.namesrv);
  m_consumer.setGroupName(m_args.groupname);
  m_consumer.setNamesrvDomain(m_args.namesrv_domain);
  m_consumer.setConsumeFromWhere(CONSUME_FROM_LAST_OFFSET);
  m_consumer.setInstanceName(m_args.groupname);
  m_consumer.subscribe(m_args.topic_air, "*");
  m_consumer.subscribe(m_args.topic_vehicle_moving, "*");
  m_consumer.subscribe(m_args.topic_cargate, "*");
  m_consumer.subscribe(m_args.topic_nio, "*");
  m_consumer.subscribe(m_args.topic_laneinfo, "*");
  m_consumer.subscribe(m_args.topic_acc, "*");
  m_consumer.subscribe(m_args.topic_car, "*");
  m_consumer.subscribe(m_args.topic_aio, "*");
  m_consumer.subscribe(m_args.topic_lab, "*");
  m_consumer.subscribe(m_args.topic_driveselect, "*");
  m_consumer.setConsumeThreadCount(15);
  m_consumer.setTcpTransportTryLockTimeout(1000);
  m_consumer.setTcpTransportConnectTimeout(400);
  m_consumer.registerMessageListener(&m_listener);
  try {
    m_consumer.start();
  } catch (MQClientException& e) {
    std::cout << e << std::endl;
  }

  m_tps.start();
}

Consumer::~Consumer()
{
  m_consumer.shutdown();
}

// VehicleMoving Sync to ACC
// void Producer::SendACC(simapi &api, std::ofstream &log)
// {
// 	std::unique_ptr<SimOne_Data_SensorDetections> pDetections = std::make_unique<SimOne_Data_SensorDetections>();
//   std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
// 
//   if (!api.SimAPI_GPS(pGps.get()))
//   {
//     return;
//   }
// 
//   SSD::SimString mvLaneName;
//   if (!api.SimAPI_GetLaneName({pGps->posX, pGps->posY, pGps->posZ}, mvLaneName))
//   {
//     return;
//   }
// 
//   int aheadVId = 0;
//   double minDist = std::numeric_limits<double>::infinity();
//   if (api.SimAPI_GetSensorDetections(pDetections.get()))
//   {
//     std::vector<ACC_ObjVehicle> objVehicles;
// 
//     for (int i = 0; i < pDetections->objectSize; i++)
//     {
//       switch (pDetections->objects[i].type)
//       {
//         case ESimOne_Obstacle_Type_Unknown:
//         case ESimOne_Obstacle_Type_Pedestrian:
//         case ESimOne_Obstacle_Type_Pole:
//         case ESimOne_Obstacle_Type_Static:
//         case ESimOne_Obstacle_Type_Fence:
//         case ESimOne_Obstacle_Type_RoadMark:
//         case ESimOne_Obstacle_Type_TrafficSign:
//         case ESimOne_Obstacle_Type_TrafficLight:
//         case ESimOne_Obstacle_Type_GuardRail:
//         case ESimOne_Obstacle_Type_SpeedLimitSign:
//         case ESimOne_Obstacle_Type_BicycleStatic:
//         case ESimOne_Obstacle_Type_RoadObstacle:
//           continue;
//         case ESimOne_Obstacle_Type_Car:
//         case ESimOne_Obstacle_Type_Bicycle:
//         case ESimOne_Obstacle_Type_Rider:
//         case ESimOne_Obstacle_Type_Truck:
//         case ESimOne_Obstacle_Type_Bus:
//         case ESimOne_Obstacle_Type_SpecialVehicle:
//         case ESimOne_Obstacle_Type_Motorcycle:
//         case ESimOne_Obstacle_Type_Dynamic:
//         {
//           if (pDetections->objects[i].relativePosX > 0) // front vehicle
//           {
//             ACC_ObjVehicle objV;
//             objV.id = pDetections->objects[i].id;
//             objV.zd_x = pDetections->objects[i].posX;                          // Unity x = (od X)   as mentioned in mv ox
//             objV.zd_y = 0.0;                                                   // pDetections->objects[i].posZ; // Unity y = (od Z)   as mentioned in mv ox
//             objV.zd_z = pDetections->objects[i].posY;                          // Unity z = (od Y)   as mentioned in mv ox
//             objV.ori_x = -pDetections->objects[i].oriX;                        // Unity oriX = -(od oriX)
//             objV.ori_y = 90.0 - (pDetections->objects[i].oriZ * 180.0 / M_PI); // Unity oriY = 90 - (od oriZ)
//             objV.ori_z = -pDetections->objects[i].oriY;                        // Unity oriZ = -(od oriY)
// 
//             objVehicles.emplace_back(objV);
// 
//             double dist = 0.0;
//             if (api.SimAPI_GetObjVehicleDist({pGps->posX, pGps->posY, pGps->posZ},
//                                              {pDetections->objects[i].posX, pDetections->objects[i].posY, pDetections->objects[i].posZ},
//                                              mvLaneName, dist))
//             {
//               // std::cout << "------------------------dist: " << dist << std::endl;
//               if (dist < minDist)
//               {
//                 minDist = dist;
//                 aheadVId = pDetections->objects[i].id;
//               }
//             }
//           }
//         }
//         break;
//         default:;
//       }
//     }
// 
//     // m_sync_vm_acc.store(false);
//     std::unique_lock<std::mutex> lock(mtx);
//     cv.wait(lock, [&](){return !m_sync_vm_acc.load();});
//     // --------------------Normal start---------------------
//     if (0 == objVehicles.size())
//     {
//         m_sim_vehiclemoving.acc_frame=0;
//         nlohmann::json j;
//         j["VehicleAheadPosition"] = nlohmann::json::array();
//         j["DistanceAdjustment"] = 0;            // 距离调整  0，3
//         j["DistanceAdjustmentActive"] = false;  // websocket_client::mDrivingLever.isCruiseOn; // 距离调整激活 false
//         j["PredictiveControlType"] = 0;         // 预测控制类型 0
//         j["PredictiveControlSpeed"] = 0;        // 预测控制速度 0，327
// 
//         // j["SpeedLimit"] =  websocket_client::mCruise_valid ? websocket_client::mCruise_speed * 3.6 : 333.0; // 速度限制 333.0 --mk/h
//         j["SpeedLimit"] =  websocket_client::mCruise_speed ? std::round(websocket_client::mCruise_speed * 3.6 / 10.0)*10.0 : 333.0; // 速度限制 333.0 --mk/h
//         j["Status"] = websocket_client::mCruise_valid ? 3 : 1;  // 状态 Green (unity 3 On/5 ReadyToGo); Wite (unity 1 Wait/2 Standby/4 Violating); Disapear (unity 0 Off, skip this status)
//         j["VehicleAheadCountdown"] = 0.0; // minDist; ------------
//         j["VehicleAheadID"] = 0;          // aheadVId; ------------
//         j["VehicleOnEgoLaneID"] = 0;      // 100
// 
//         MQMessage msg_acc(m_args.topic_acc, "*", j.dump());
//         msg_acc.setKeys(m_args.topic_acc);
//         try
//         {
//           SendResult sendResult = m_producer.send(msg_acc);
//           std::this_thread::sleep_for(std::chrono::milliseconds(1));
//           // std::cout << "----- SendACC: " << j.dump() << std::endl;
//           // std::string messageId = sendResult.getMsgId();
//           // std::cout << "msg_acc sent with ID: " << messageId << std::endl;
//           log << "[" << getCurrentChinaTime() << "] " << j.dump() << std::endl;
//           log.flush();
//         }
//         catch (MQException &e)
//         {
//           std::cout << e << endl;
//         }
//     }
//     else
//     {
//       m_sim_vehiclemoving.acc_frame=pDetections->frame;
//       for (int i = 0; i < objVehicles.size(); i++)
//       {
//         nlohmann::json j;
//         // j["frame"] = m_sim_vehiclemoving.acc_frame; // -------------------------- Debug
//         j["VehicleAheadPosition"].emplace_back(objVehicles[i].zd_x);
//         j["VehicleAheadPosition"].emplace_back(objVehicles[i].zd_y);
//         j["VehicleAheadPosition"].emplace_back(objVehicles[i].zd_z);
//         j["VehicleAheadPosition"].emplace_back(objVehicles[i].ori_x);
//         j["VehicleAheadPosition"].emplace_back(objVehicles[i].ori_y);
//         j["VehicleAheadPosition"].emplace_back(objVehicles[i].ori_z);
// 
//         j["DistanceAdjustment"] = 0;                                                                                 // 距离调整  0，3
//         j["DistanceAdjustmentActive"] = false;                                                                       // websocket_client::mDrivingLever.isCruiseOn; // 距离调整激活 false
//         j["PredictiveControlType"] = 0;                                                                              // 预测控制类型 0
//         j["PredictiveControlSpeed"] = 0;                                                                             // 预测控制速度 0，327
//         j["SpeedLimit"] =  websocket_client::mCruise_speed ? std::round(websocket_client::mCruise_speed * 3.6 / 10.0) * 10.0 : 333.0; // 速度限制 333.0 --mk/h
//         j["Status"] = websocket_client::mCruise_valid ? 3 : 1;  // 状态 Green (unity 3 On/5 ReadyToGo); Wite (unity 1 Wait/2 Standby/4 Violating); Disapear (unity 0 Off, skip this status)
//         j["VehicleAheadCountdown"] = std::isinf(minDist) ? 0.0 : (float)(minDist); // minDist; ------------
//         j["VehicleAheadID"] = aheadVId;                                            // aheadVId; ------------
//         j["VehicleOnEgoLaneID"] = objVehicles[i].id;                               // 100
// 
//         MQMessage msg_acc(m_args.topic_acc, "*", j.dump());
//         msg_acc.setKeys(m_args.topic_acc);
//         try
//         {
//           SendResult sendResult = m_producer.send(msg_acc);
//           std::this_thread::sleep_for(std::chrono::milliseconds(1));
//           // std::cout << "----- SendACC: frame: " << m_sim_vehiclemoving.acc_frame << j.dump() << std::endl;
//           // std::string messageId = sendResult.getMsgId();
//           // std::cout << "msg_acc sent with ID: " << messageId << std::endl;
//           log << "[" << getCurrentChinaTime() << "] ----- SendACC: " << j.dump() << std::endl;
//           log.flush();
//         }
//         catch (MQException &e)
//         {
//           std::cout << e << endl;
//         }
//       }
//     }
//     // --------------------Normal End---------------------
// 
//     // ------------ Debug Begin ------------------------
//     /*
//     float deltPosX = pGps->posX + 10; // *cos(pGps->oriZ);
//     float deltPosY = pGps->posY + 0;  // *sin(pGps->oriZ);
//     double dist = 0.0;
//     api.SimAPI_GetObjVehicleDist({pGps->posX, pGps->posY, pGps->posZ}, {deltPosX, deltPosY, pGps->posZ}, mvLaneName, dist);
//     nlohmann::json j;
//     j["VehicleAheadPosition"].emplace_back(deltPosX);
//     j["VehicleAheadPosition"].emplace_back(0.0);
//     j["VehicleAheadPosition"].emplace_back(deltPosY);
//     j["VehicleAheadPosition"].emplace_back(-pGps->oriX);
//     j["VehicleAheadPosition"].emplace_back(90.0 - (pGps->oriZ * 180.0 / M_PI));
//     j["VehicleAheadPosition"].emplace_back(-pGps->oriY);
//     j["DistanceAdjustment"] = 0;                                                                       // 距离调整  0，3
//     j["DistanceAdjustmentActive"] = false;                                                             // websocket_client::mDrivingLever.isCruiseOn; // 距离调整激活 false
//     j["PredictiveControlType"] = 0;                                                                    // 预测控制类型 0
//     j["PredictiveControlSpeed"] = 0;                                                                   // 预测控制速度 0，327
//     j["SpeedLimit"] = websocket_client::mCruise_speed ? websocket_client::mCruise_speed * 3.6 : 333.0; // 速度限制 333.0 --mk/h
//     j["Status"] = websocket_client::mCruise_valid ? 3 : 1;                                             // 状态 Green (unity 3 On/5 ReadyToGo); Wite (unity 1 Wait/2 Standby/4 Violating); Disapear (unity 0 Off, skip this status)
//     j["VehicleAheadCountdown"] = dist;                                                                 // minDist; ------------
//     j["VehicleAheadID"] = 11646222;                                                                    // aheadVId; ------------
//     j["VehicleOnEgoLaneID"] = 11646222;                                                                // 100
// 
//     MQMessage msg_acc(m_args.topic_acc, "*", j.dump());
//     msg_acc.setKeys(m_args.topic_acc);
//     msg_acc.setProperty("TTL", std::to_string(m_TTL));
//     try
//     {
//       SendResult sendResult = m_producer.send(msg_acc);
//       // std::cout << "----- SendACC: " << j.dump() << std::endl;
//       // std::string messageId = sendResult.getMsgId();
//       // std::cout << "msg_acc sent with ID: " << messageId << std::endl;
//       // log << "[----- SendACC: " << getCurrentChinaTime() << "] " << j.dump() << std::endl;
//       // log.flush();
//     }
//     catch (MQException &e)
//     {
//       std::cout << e << endl;
//     }
//     */
//     // ------------ Debug End ------------------------
// 
//     m_sync_vm_acc.store(true);
//     cv.notify_all();
//   }
// }

// VehicleMoving Sync to ACC
// void Producer::SendVehicleMoving(simapi &api, std::ofstream &log)
// {
//   std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
//   // while (true)
//   // {
//   //   if (m_sync_vm_acc.load())
//   //   {
// 
//   std::unique_lock<std::mutex> lock(mtx);
//   cv.wait(lock, [&](){ return m_sync_vm_acc.load(); });
// 
//   if (0 == m_sim_vehiclemoving.acc_frame)
//   {
//     if (api.SimAPI_GPS(pGps.get()))
//     {
//       nlohmann::json j;
//       j["Position"].emplace_back(pGps->posX);                         // Unity x = (od X)
//       j["Position"].emplace_back(pGps->posZ);                         // Unity y = (od Z)
//       j["Position"].emplace_back(pGps->posY);                         // Unity z = (od Y)
//       j["Rotation"].emplace_back(-pGps->oriX);                        // Unity oriX = -(od oriX)
//       j["Rotation"].emplace_back(90.0 - (pGps->oriZ * 180.0 / M_PI)); // Unity oriY = 90 - (od oriZ)
//       j["Rotation"].emplace_back(-pGps->oriY);                        // Unity oriZ = -(od oriY)
// 
//       MQMessage msg_vehicle_moving(m_args.topic_vehicle_moving, "*", j.dump());
//       msg_vehicle_moving.setKeys(m_args.topic_vehicle_moving);
//       // msg_vehicle_moving.setProperty("TTL", std::to_string(m_TTL));
//       try
//       {
//         SendResult sendResult = m_producer.send(msg_vehicle_moving);
//         std::this_thread::sleep_for(std::chrono::milliseconds(1));
//         // std::cout << "----- SendVehicleMoving: " << j.dump() << std::endl;
//         log << "[" << getCurrentChinaTime() << "] " << j.dump() << std::endl;
//         log.flush();
//       }
//       catch (MQException &e)
//       {
//         std::cout << e << endl;
//       }
//     }
//   }
//   else
//   {
//       nlohmann::json j;
//       // std::unique_lock<std::mutex> lock(mtx_vehiclemoving);
//       // cv_vehiclemoving.wait(lock, [&](){ return m_sync_vehiclemoving.load(); });
//       while(m_sync_vehiclemoving.load())
//       {
//         std::this_thread::yield();
//       }
//       m_sync_vehiclemoving.store(true);
//       // j["frame"] = m_sim_vehiclemoving.acc_frame; // -------------------------- Debug
//       j["Position"].emplace_back(m_sim_vehiclemoving.posX);                         // Unity x = (od X)
//       j["Position"].emplace_back(m_sim_vehiclemoving.posZ);                         // Unity y = (od Z)
//       j["Position"].emplace_back(m_sim_vehiclemoving.posY);                         // Unity z = (od Y)
//       j["Rotation"].emplace_back(-m_sim_vehiclemoving.oriX);                        // Unity oriX = -(od oriX)
//       j["Rotation"].emplace_back(90.0 - (m_sim_vehiclemoving.oriZ * 180.0 / M_PI)); // Unity oriY = 90 - (od oriZ)
//       j["Rotation"].emplace_back(-m_sim_vehiclemoving.oriY);                        // Unity oriZ = -(od oriY)
//       m_sync_vehiclemoving.store(false);
//       // m_sync_vehiclemoving.store(false);
//       // cv_vehiclemoving.notify_all();
// 
//       MQMessage msg_vehicle_moving(m_args.topic_vehicle_moving, "*", j.dump());
//       msg_vehicle_moving.setKeys(m_args.topic_vehicle_moving);
//       // msg_vehicle_moving.setProperty("TTL", std::to_string(m_TTL));
//       try
//       {
//         SendResult sendResult = m_producer.send(msg_vehicle_moving);
//         std::this_thread::sleep_for(std::chrono::milliseconds(1));
//         // std::cout << "----- SendVehicleMoving: frame--- " << frame << " " << j.dump() << std::endl;
//         log << "[" << getCurrentChinaTime() << "] ----- SendVehicleMoving: " << j.dump() << std::endl;
//         log.flush();
//       }
//       catch (MQException &e)
//       {
//         std::cout << e << endl;
//       }
// 
//   }
// 
//   m_sync_vm_acc.store(false);
//   cv.notify_all();
// 
//   //     break;
//   //   }
//   //  
//   //   std::this_thread::yield();
//   // }
// }

// void Producer::GpsUpdateCB(const char* mainVehicleId, SimOne_Data_Gps *pGps)
// {
//   // static std::ofstream log_gps_cb;
//   // static bool run_once = true;
//   // if (run_once)
//   // {
//   //   run_once = false;
//   //   log_gps_cb.open("log_gps_cb.txt", std::ios::out | std::ios::trunc);
//   // }
//   static std::map<int, VehicleMoving_SimData> vehicle_moving_map;
//   static std::mutex mtx_gps;
//   VehicleMoving_SimData data;
//   data.posX = pGps->posX;
//   data.posY = pGps->posY;
//   data.posZ = pGps->posZ;
//   data.oriX = pGps->oriX;
//   data.oriY = pGps->oriY;
//   data.oriZ = pGps->oriZ;
//   vehicle_moving_map[pGps->frame] = data;
// 
//   if (0 != m_sim_vehiclemoving.acc_frame)
//   {
//     while (m_sync_vehiclemoving.load())
//     {
//       std::this_thread::yield();
//     }
//     m_sync_vehiclemoving.store(true);
//     m_sim_vehiclemoving = vehicle_moving_map[m_sim_vehiclemoving.acc_frame];
//     m_sync_vehiclemoving.store(false);
//     {
//       std::lock_guard<std::mutex> lock(mtx_gps);
//       auto it = vehicle_moving_map.begin();
//       while (it != vehicle_moving_map.end())
//       {
//         // log_gps_cb << "frame: " << it->first << std::endl;
//         // log_gps_cb.flush();
//         if (it->first <= m_sim_vehiclemoving.acc_frame)
//         {
//           // if (it->first == m_sim_vehiclemoving.acc_frame)
//           // {
//           //   while(m_sync_vehiclemoving.load())
//           //   {
//           //     std::this_thread::yield();
//           //   }
//           //   // std::unique_lock<std::mutex> lock(mtx_vehiclemoving);
//           //   // cv_vehiclemoving.wait(lock, [&](){ return !m_sync_vehiclemoving.load(); });
//           //   m_sync_vehiclemoving.store(true);
//           //   m_sim_vehiclemoving = it->second;
//           //   m_sync_vehiclemoving.store(false);
//           //   // m_sync_vehiclemoving.store(true);
//           //   // cv_vehiclemoving.notify_all();
//           // }
//           auto nextIt = std::next(it);
//           vehicle_moving_map.erase(it);
//           it = nextIt;
//         }
//         else
//         {
//           break;
//         }
//       }
//     }
//   }
//   else
//   {
//     if (vehicle_moving_map.size() > 100)
//     {
//       std::lock_guard<std::mutex> lock(mtx_gps);
//       auto it = vehicle_moving_map.begin();
//       for (int i=0; i<60 && it!=vehicle_moving_map.end(); i++)
//       {
//         // log_gps_cb << "frame: " << it->first << std::endl;
//         // log_gps_cb.flush();
//         auto nextIt = std::next(it);
//         vehicle_moving_map.erase(it);
//         it = nextIt;
//       }
//     }
//   }
// }
