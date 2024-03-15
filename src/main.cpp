#include <stdlib.h>
#include <string.h>
#include <chrono>
#include <condition_variable>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <vector>
#include <fstream>
#include "mqio.h"
#include "timer.hpp"
// #include "zd/ZDBOX2WebSocketAPI.h"
#include "client.h"

// extern void run_websocket_client(SendAndConsumerArgs &args);
// extern void ConsumeQueueData(simapi &api);

int main(int argc, char* argv[])
{
  SendAndConsumerArgs args;
  websocket_client wsC;
  httpclient httpCli(args);

  simapi *p_api = nullptr;
  std::thread websocket_client_reverse_control = std::thread(std::bind(&websocket_client::ReverseControl, &wsC, std::ref(p_api), std::ref(args)));
  simapi &&api = RunSimOneAPI(args);
  p_api = &api;

  Producer p(args);
  // Consumer c(args);

  if (!api.SimAPI_SetJudgeEventCB(p.JudgeEventCB))
  {
    std::cout << "SetJudgeEventCB Failed!" << std::endl;
  }

  if (!api.SimAPI_SetScenarioEventCB(p.ScenarioEventCB))
  {
    std::cout << "SetScenarioEventCB Failed!" << std::endl;
  }

  if (!api.SimAPI_SetGpsUpdateCB(p.GpsUpdateCB))
  {
    std::cout << "SetGpsUpdateCB Failed!" << std::endl;
  }

  if (!api.SimAPI_SetSensorDetectionsUpdateCB(p.SensorDetectionsUpdateCB))
  {
    std::cout << "SetSensorDetectionsUpdateCB Failed!" << std::endl;
  }

  std::ofstream log_vehiclemoving_acc, log_air, log_cargate, log_nio, log_laneinfo, log_car, log_aio, log_lab, log_driveselect, log_collision, log_case_type; // log_vehicle_moving, log_acc, 
  log_vehiclemoving_acc.open("log_vehiclemoving_acc.txt", std::ios::out | std::ios::trunc);
  log_air.open("log_air.txt", std::ios::out | std::ios::trunc);
  // log_vehicle_moving.open("log_vehicle_moving.txt", std::ios::out | std::ios::trunc);
  log_cargate.open("log_cargate.txt", std::ios::out | std::ios::trunc);
  log_nio.open("log_nio.txt", std::ios::out | std::ios::trunc);
  log_laneinfo.open("log_laneinfo.txt", std::ios::out | std::ios::trunc);
  // log_acc.open("log_acc.txt", std::ios::out | std::ios::trunc);
  log_car.open("log_car.txt", std::ios::out | std::ios::trunc);
  log_aio.open("log_aio.txt", std::ios::out | std::ios::trunc);
  log_lab.open("log_lab.txt", std::ios::out | std::ios::trunc);
  log_driveselect.open("log_driveselect.txt", std::ios::out | std::ios::trunc);
  log_collision.open("log_collision.txt", std::ios::out | std::ios::trunc);
  log_case_type.open("log_case_type.txt", std::ios::out | std::ios::trunc);

  Timer t_vehiclemoving_acc, t_air, t_cargate, t_nio, t_laneinfo, t_car, t_aio, t_lab, t_driveselect, t_collision, t_case_type; // t_vehicle_moving, t_acc,

  t_vehiclemoving_acc.start(16, std::bind(&Producer::SendVehicleMovingACC, &p, std::ref(api), std::ref(log_vehiclemoving_acc)));
  // t_vehicle_moving.start(100, std::bind(&Producer::SendVehicleMoving, &p, std::ref(api), std::ref(log_vehicle_moving)));
  t_air.start(100, std::bind(&Producer::SendAIR, &p, std::ref(api), std::ref(log_air)));
  // t_vehicle_moving.start(100, std::bind(&Producer::SendVehicleMoving, &p, std::ref(api), std::ref(log_vehicle_moving)));
  t_cargate.start(100, std::bind(&Producer::SendCarGate, &p, std::ref(api), std::ref(log_cargate)));
  t_nio.start(100, std::bind(&Producer::SendNIO, &p, std::ref(api), std::ref(log_nio)));
  t_laneinfo.start(100, std::bind(&Producer::SendLaneInfo, &p, std::ref(api), std::ref(log_laneinfo)));
  // t_acc.start(100, std::bind(&Producer::SendACC, &p, std::ref(api), std::ref(log_acc)));
  t_car.start(100, std::bind(&Producer::SendCAR, &p, std::ref(api), std::ref(log_car)));
  t_aio.start(100, std::bind(&Producer::SendAIO, &p, std::ref(api), std::ref(log_aio)));
  t_lab.start(100, std::bind(&Producer::SendLAB, &p, std::ref(api), std::ref(log_lab)));
  t_driveselect.start(100, std::bind(&Producer::SendDriveSelect, &p, std::ref(api), std::ref(log_driveselect)));
  t_collision.start(100, std::bind(&Producer::SendCollision, &p, std::ref(api), args.minDistThre, std::ref(log_collision)));
  t_case_type.start(100, std::bind(&Producer::SendCaseType, &p, std::ref(api), std::ref(log_case_type)));

  wsC.set_steering_scope(args.steering_max.c_str());
  std::thread websocket_client_get = std::thread(std::bind(&websocket_client::run_websocket_client, &wsC, std::ref(args)));
  std::thread websocket_client_set = std::thread(std::bind(&websocket_client::ConsumeQueueData, &wsC, std::ref(api), std::ref(args)));
  websocket_client_get.detach();
  websocket_client_set.detach();
  websocket_client_reverse_control.detach();

  while (true)
  {
    if (api.SimAPI_IsCaseStop())
    {
      t_air.stop();
      t_vehiclemoving_acc.stop();
      // t_vehicle_moving.stop();
      t_cargate.stop();
      t_nio.stop();
      t_laneinfo.stop();
      // t_acc.stop();
      t_car.stop();
      t_aio.stop();
      t_lab.stop();
      t_driveselect.stop();
      t_collision.stop();

      log_air.close();
      log_vehiclemoving_acc.close();
      // log_vehicle_moving.close();
      log_cargate.close();
      log_nio.close();
      log_laneinfo.close();
      // log_acc.close();
      log_car.close();
      log_aio.close();
      log_lab.close();
      log_driveselect.close();
      log_collision.close();

      break;
    }

    // try
    // {
    //   p.SendCarGate(api);
    //   p.SendAIR(api);
    //   p.SendVehicleMoving(api);
    //   p.SendNIO(api);
    //   p.SendLaneInfo(api);
    //   p.SendACC(api);
    //   p.SendCAR(api);
    //   p.SendAIO(api);
    //   p.SendLAB(api);
    //   p.SendDriveSelect(api);
    // }
    // catch (MQException &e)
    // {
    //   std::cout << e << endl;
    // }

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  api.SimAPI_TerminateSimOneAPI();
  httpCli.do_post("", args.http_box_stop_url);
  std::cout << "=============================== stop all simulator" << std::endl;
  return 0;
}
