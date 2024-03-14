#include "simapi.h"

dumper simapi::dbg_data;
char* simapi::prefix_h265 = nullptr;
char* simapi::prefix_rle = nullptr;
bool simapi::lights[2];

simapi::simapi(const char* mv_id):mainVehicleId(mv_id)
{
	lights[0] = false;
	lights[1] = false;
}
simapi::~simapi(){}

void simapi::SimAPI_InitSimOneAPI(bool isJoinTimeLoop, const char *serverIP)
{
	try{
	SimOneAPI::InitSimOneAPI(mainVehicleId.c_str(), isJoinTimeLoop, serverIP);
	}
	catch(const std::exception & e)
	{
		std::cout << e.what()<<std::endl;
	}
	catch(...)
	{
		std::cout << "ddddddddddddd" <<std::endl;
	}
}

bool simapi::SimAPI_IsCaseStop()
{
	return (SimOneAPI::GetCaseRunStatus() == ESimOne_Case_Status::ESimOne_Case_Status_Stop);
}

void simapi::SimAPI_TerminateSimOneAPI()
{
	SimOneAPI::TerminateSimOneAPI();
	std::this_thread::sleep_for(std::chrono::milliseconds(3000));
	std::cout << "SimOneAPI Closed!" << std::endl;
}

void simapi::SimAPI_GetCaseInfo()
{
	std::unique_ptr<SimOne_Data_CaseInfo> mpCaseInfo = std::make_unique<SimOne_Data_CaseInfo>();
	if (!SimOneAPI::GetCaseInfo(mpCaseInfo.get()))
	{
		std::cout << "GetCaseInfo Failed!" << std::endl;
		return;
	}
	std::cout << "caseName: " << mpCaseInfo->caseName;
	std::cout << "caseId: " << mpCaseInfo->caseId;
	std::cout << "taskId: " << mpCaseInfo->taskId;
}

void simapi::SimAPI_GetCaseRunStatus()
{
	switch (SimOneAPI::GetCaseRunStatus())
	{
	case ESimOne_Case_Status::ESimOne_Case_Status_Unknown:
		std::cout << "ESimOne_Case_Status_Unknown" << std::endl;
		break;
	case ESimOne_Case_Status::ESimOne_Case_Status_Stop:
		std::cout << "ESimOne_Case_Status_Stop" << std::endl;
		break;
	case ESimOne_Case_Status::ESimOne_Case_Status_Running:
		std::cout << "ESimOne_Case_Status_Running" << std::endl;
		break;
	case ESimOne_Case_Status::ESimOne_Case_Status_Pause:
		std::cout << "ESimOne_Case_Status_Pause" << std::endl;
		break;
	default:
		std::cout << "Invalid CaseRunStatus!" << std::endl;
	}
}

void simapi::SimAPI_GetMainVehicleList()
{
	std::unique_ptr<SimOne_Data_MainVehicle_Info> MainVehicleListTest = std::make_unique<SimOne_Data_MainVehicle_Info>();
	if(!SimOneAPI::GetMainVehicleList(MainVehicleListTest.get()))
	{
		std::cout << "GetMainVehicleList Failed!" << std::endl;
		return;
	}
	std::cout << "size:" << MainVehicleListTest->size << std::endl;
	for (int i = 0; i < MainVehicleListTest->size; i++)
	{
		std::cout << "id:" << MainVehicleListTest->id_list[i] << std::endl;
		std::cout << "type:" << MainVehicleListTest->type_list[i] << std::endl;
	}
}

bool simapi::SimAPI_GetVehicleState(SimOne_Data_Vehicle_Extra* pVehExtraState)
{
	if (!SimOneAPI::GetVehicleState(mainVehicleId.c_str(), pVehExtraState))
  {
    return false;
  }
	return true;
}

void simapi::SimAPI_GetMainVehicleStatus(bool IsCallBackMode)
{
	if (IsCallBackMode)
	{
		auto function = [](const char *mainVehicleId, SimOne_Data_MainVehicle_Status *pMainVehicleStatus) {
			std::cout << "mainVehicleId: " << pMainVehicleStatus->mainVehicleId << std::endl;
			std::cout << "mainVehicleStatus: " << pMainVehicleStatus->mainVehicleStatus << std::endl;
		};
		SimOneAPI::SetMainVehicleStatusUpdateCB(function);
	}
	else
	{
		std::unique_ptr<SimOne_Data_MainVehicle_Status> pMainVehicleStatus = std::make_unique<SimOne_Data_MainVehicle_Status>();
		if (!SimOneAPI::GetMainVehicleStatus(mainVehicleId.c_str(), pMainVehicleStatus.get()))
		{
			std::cout << "GetMainVehicleStatus Failed!" << std::endl;
			return;
		}
		std::cout << "mainVehicleId: " << pMainVehicleStatus->mainVehicleId << std::endl;
		std::cout << "mainVehicleStatus: " << pMainVehicleStatus->mainVehicleStatus << std::endl;
	}
}

void simapi::SimAPI_GetHDMapData()
{
	std::unique_ptr<SimOne_Data_Map> pHdmap = std::make_unique<SimOne_Data_Map>();
	if (!SimOneAPI::GetHDMapData(pHdmap.get()))
	{
		std::cout << "GetHDMapData Failed!" << std::endl;
		return;
	}
	std::cout << "openDrive: " << pHdmap->openDrive << std::endl;;
	std::cout << "openDriveUrl: " << pHdmap->openDriveUrl << std::endl;;
	std::cout << "opendriveMd5: " << pHdmap->opendriveMd5 << std::endl;
}

void simapi::SimAPI_GetSensorConfigurations()
{
	std::unique_ptr<SimOne_Data_SensorConfigurations> pConfigs = std::make_unique<SimOne_Data_SensorConfigurations>();
	while (true)
	{
		bool flag = SimOneAPI::GetSensorConfigurations(mainVehicleId.c_str(), pConfigs.get());
		if (flag)
		{
			dbg_data.dump_sensor_configurations(mainVehicleId.c_str(), pConfigs.get());
		}
		else
		{
			std::cout << "there is no sensor config in current vehicle!!!" << std::endl;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(30));
	}
}


void simapi::SimAPI_GetMessage()
{
	auto function = [](const char* source, const char* target, const char* type, const char* content) {
		std::cout << "source:" << std::string(source) << ", target:" << std::string(target) << ", type:" << std::string(type) << ", content:" << std::string(content) << std::endl;
	};
	SimOneAPI::SetScenarioEventCB(function);
}


void simapi::SimAPI_GetVersion()
{
	std::cout << "Version:" << SimOneAPI::GetVersion() << std::endl;;
}

void simapi::SimAPI_SetEnvironment()
{
	SimOne_Data_Environment Environment; // std::unique_ptr<SimOne_Data_Environment> pEnvironment = std::make_unique<SimOne_Data_Environment>();
	Environment.timeOfDay = 1000;
	// Environment.heightAngle = 90;
	// Environment.directionalLight = 0.5f;
	// Environment.ambientLight = 0.5f;
	// Environment.artificialLight = 0.5f;
	Environment.cloudDensity = 0.5f;
	Environment.fogDensity = 0.5f;
	Environment.rainDensity = 0.5f;
	Environment.snowDensity = 0.5f;
	// Environment.groundHumidityLevel = 0.5f;
	// Environment.groundDirtyLevel = 0.5f;
	if (!SimOneAPI::SetEnvironment(&Environment))
	{
		std::cout << "SetEnvironment Failed!" << std::endl;
	}
	else
	{
		dbg_data.dump_environment(&Environment);
	}
}

void simapi::SimAPI_GetEnvironment()
{
	std::unique_ptr<SimOne_Data_Environment> pEnvironment = std::make_unique<SimOne_Data_Environment>();
	bool flag = SimOneAPI::GetEnvironment(pEnvironment.get());
	if (flag)
	{
		dbg_data.dump_environment(pEnvironment.get());
	}
	else
	{
		std::cout << "GetEnvironment Failed!" << std::endl;
		return;
	}
}

void simapi::SimAPI_GetHdMapData() {
	std::unique_ptr<SimOne_Data_Map> hdMap = std::make_unique<SimOne_Data_Map>();
	while (true) {
		if (SimOneAPI::GetHDMapData(hdMap.get())) {
			std::cout << "hdMap.OpenDrive:" << hdMap->openDrive<< std::endl;
			std::this_thread::sleep_for(std::chrono::milliseconds(30));
		}
	}
}

void simapi::SimAPI_GetGroundTruth(bool IsCallBackMode)
{
	if (IsCallBackMode) {
		auto function = [](const char* mainVehicleId, SimOne_Data_Obstacle *pObstacle) {
			dbg_data.dump_ground_truth(mainVehicleId, pObstacle);
		};
		SimOneAPI::SetGroundTruthUpdateCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_Obstacle> pDetections = std::make_unique<SimOne_Data_Obstacle>();
		int lastFrame = 0;
		while (true) {
			bool flag = SimOneAPI::GetGroundTruth(mainVehicleId.c_str(),  pDetections.get());
			if (flag && pDetections->frame != lastFrame)
			{
				lastFrame  = pDetections->frame;
				dbg_data.dump_ground_truth(mainVehicleId.c_str(), pDetections.get());
			}
			if (!flag)
			{
				std::cout << "GetGroundTruth Failed!" << std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
	}
}

void simapi::SimAPI_RadarDetection(bool IsCallBackMode)
{
	if (IsCallBackMode) {
		auto function = [](const char* mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection *pDetections)
		{
			dbg_data.dump_radar_detection(mainVehicleId, sensorId, pDetections);
		};
		SimOneAPI::SetRadarDetectionsUpdateCB(function);
	}
	else
	{
		std::unique_ptr<SimOne_Data_RadarDetection> pDetections = std::make_unique<SimOne_Data_RadarDetection>();
		int lastFrame = 0;
		while (true)
		{
			bool flag = SimOneAPI::GetRadarDetections(mainVehicleId.c_str(), "radar1", pDetections.get()); // objectBasedRadar1
			if (flag && pDetections->frame != lastFrame)
			{
				lastFrame  = pDetections->frame;
				dbg_data.dump_radar_detection(mainVehicleId.c_str(), "radar1", pDetections.get()); // objectBasedRadar1
			}
			if (!flag)
			{
				std::cout << "GetRadarDetections Failed!" << std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(30));
		}
	}
}

bool simapi::SimAPI_GetSensorDetections(SimOne_Data_SensorDetections *pDetections)
{
	if (!SimOneAPI::GetSensorDetections(mainVehicleId.c_str(), "sensorFusion1", pDetections)) // "objectBasedCamera1"
	{
		std::cout << "GetSensorDetections Failed!" << std::endl;
		return false;
	}
	return true;
	
	// if (IsCallBackMode)
	// {
	// 	auto function = [](const char* MainVehicleID, const char* sensorId, SimOne_Data_SensorDetections* pGroundtruth)
	// 	{
	// 		dbg_data.dump_sensor_detections(MainVehicleID, sensorId, pGroundtruth);
	// 	};
	// 	SimOneAPI::SetSensorDetectionsUpdateCB(function);
	// }
	// else
	// {
	// 	std::unique_ptr<SimOne_Data_SensorDetections> pGroundtruth = std::make_unique<SimOne_Data_SensorDetections>();
	// 	int lastFrame = 0;
	// 	while (true)
	// 	{
	// 		// "sensorFusion1" "objectBasedCamera1" "objectBasedLidar1" "perfectPerception1"
	// 		bool flag = SimOneAPI::GetSensorDetections(mainVehicleId.c_str(), "objectBasedCamera1", pGroundtruth.get());
	// 		if (flag && pGroundtruth->frame != lastFrame)
	// 		{
	// 			lastFrame = pGroundtruth->frame;
	// 			dbg_data.dump_sensor_detections(mainVehicleId.c_str(), "objectBasedCamera1", pGroundtruth.get());
	// 		}
	// 		if (!flag)
	// 		{
	// 			std::cout << "GetSensorDetections Failed!" << std::endl;
	// 		}
	// 		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	// 	}
	// }
}

bool simapi::SimAPI_SetSensorDetectionsUpdateCB(void (*cb)(const char *mainVehicleId, const char *sensorId, SimOne_Data_SensorDetections *pGroundtruth))
{
	if (!SimOneAPI::SetSensorDetectionsUpdateCB(cb))
	{
		std::cout << "SetSensorDetectionsUpdateCB Failed!" << std::endl;
		return false;
	}

	return true;
}

bool simapi::SimAPI_GetTrafficLight(int lightId, SimOne_Data_TrafficLight *pTrafficLight)
{
	if(!SimOneAPI::GetTrafficLight(mainVehicleId.c_str(), lightId, pTrafficLight))
	{
		std::cout << "GetTrafficLight Failed!" << std::endl;
		return false;
	}
	return true;
}

void simapi::SimAPI_GetSensorRoadMark(bool IsCallBackMode)
{
	if (IsCallBackMode)
	{
		auto function = [](const char* MainVehicleID, const char* sensorId, SimOne_Data_RoadMarkInfo* pRoadMark)
		{
			std::cout << "pRoadMark's size = " << pRoadMark->detectNum << std::endl;

			//dbg_data.dump_sensor_detections(MainVehicleID, sensorId, pRoadMark);
		};
		SimOneAPI::SetSensorRoadMarkInfoCB(function);
	}
	else
	{
		std::unique_ptr<SimOne_Data_RoadMarkInfo> pRoadMark = std::make_unique<SimOne_Data_RoadMarkInfo>();
		int lastFrame = 0;
		while (true)
		{
			// "sensorFusion1" "objectBasedCamera1" "objectBasedLidar1" "perfectPerception1"
			bool flag = SimOneAPI::GetSensorRoadMarkInfo(mainVehicleId.c_str(), "sensorFusion1", pRoadMark.get());
			if (flag && pRoadMark->frame != lastFrame)
			{
				lastFrame = pRoadMark->frame;
				for (int index = 0; index < pRoadMark->detectNum; index++) {
					if(pRoadMark->roadMarks[index].id==22)
						std::cout <<"size ="<< pRoadMark->detectNum<< " id= "<< pRoadMark->roadMarks[index].id<<" type = "<<(int)pRoadMark->roadMarks[index].type << " subType = " << (int)pRoadMark->roadMarks[index].subtype << " pRoadMark.center(" << pRoadMark->roadMarks[index].center.x << ", "
							<< pRoadMark->roadMarks[index].center.y << ", " << pRoadMark->roadMarks[index].center.z << ")" <<" pRoadMark->roadMarks[index].pix3d[0] ("<< pRoadMark->roadMarks[index].bbox3d[0].x <<", " <<pRoadMark->roadMarks[index].bbox3d[0].y << ", " << pRoadMark->roadMarks[index].bbox3d[0].z <<")"<< std::endl;
				}

				std::cout << "pRoadMark's size = " << pRoadMark->detectNum << std::endl;
				//dbg_data.dump_sensor_detections(mainVehicleId.c_str(), "objectBasedCamera1", pRoadMark.get());
			}
			if (!flag)
			{
				std::cout << "GetSensorRoadMarkInfo Failed!" << std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

bool simapi::SimAPI_SensorLaneInfo(SimOne_Data_LaneInfo *pLaneInfo)
{
	if (!SimOneAPI::GetSensorLaneInfo(mainVehicleId.c_str(), "sensorFusion1", pLaneInfo)) // "objectBasedCamera1"
	{
		std::cout << "GetSensorLaneInfo Failed!" << std::endl;
		return false;
	}
	
	return true;

	// if (IsCallBackMode) {
	// 	auto function = [](const char* mainVehicleId, const char* sensorId, SimOne_Data_LaneInfo *pDetections) {
	// 		dbg_data.dump_sensor_laneInfo(mainVehicleId, sensorId, pDetections);
	// 	};
	// 	SimOneAPI::SetSensorLaneInfoCB(function);
	// }
	// else {
	// 	std::unique_ptr<SimOne_Data_LaneInfo> pDetections = std::make_unique<SimOne_Data_LaneInfo>();
	// 	int lastFrame = 0;
	// 	while (true) {
	// 		bool flag = SimOneAPI::GetSensorLaneInfo(mainVehicleId.c_str(), "sensorFusion1", pDetections.get());
	// 		if (flag && pDetections->frame != lastFrame)
	// 		{
	// 			lastFrame = pDetections->frame;
	// 			dbg_data.dump_sensor_laneInfo(mainVehicleId.c_str(), "sensorFusion1", pDetections.get());
	// 		}
	// 		if (!flag)
	// 		{
	// 			std::cout << "GetSensorLaneInfo Failed!" << std::endl;
	// 		}
	// 		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	// 	}
	// }
}

void simapi::SimAPI_UltrasonicRadars(bool IsCallBackMode)
{
	if (IsCallBackMode) {
		auto function = [](const char* mainVehicleId, SimOne_Data_UltrasonicRadars* pUltrasonics)
		{
			dbg_data.dump_ultrasonic_radars(mainVehicleId, pUltrasonics);
		};
		SimOneAPI::SetUltrasonicRadarsCB(function);
	}
	else
	{
		std::unique_ptr<SimOne_Data_UltrasonicRadars> pDetections = std::make_unique<SimOne_Data_UltrasonicRadars>();
		int lastFrame = 0;
		while (true)
		{
			bool flag = SimOneAPI::GetUltrasonicRadars(mainVehicleId.c_str(), pDetections.get());
			if (flag && pDetections->frame != lastFrame)
			{
				lastFrame  = pDetections->frame;
				dbg_data.dump_ultrasonic_radars(mainVehicleId.c_str(), pDetections.get());
			}
			if (!flag)
			{
				std::cout << "GetUltrasonicRadars Failed!" << std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

void simapi::SimAPI_UltrasonicRadar()
{
	std::unique_ptr<SimOne_Data_UltrasonicRadar> pDetections = std::make_unique<SimOne_Data_UltrasonicRadar>();
	int lastFrame = 0;
	while (true)
	{
		bool flag = SimOneAPI::GetUltrasonicRadar(mainVehicleId.c_str(), "ultrasonic1", pDetections.get());
		if (flag && pDetections->frame != lastFrame)
		{
			lastFrame  = pDetections->frame;
			dbg_data.dump_ultrasonic_radar(mainVehicleId.c_str(), "ultrasonic1",pDetections.get());
		}
		if (!flag)
		{
			std::cout << "GetUltrasonicRadar Failed!" << std::endl;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

bool simapi::SimAPI_GPS(SimOne_Data_Gps *pGps)
{
	if (!SimOneAPI::GetGps(mainVehicleId.c_str(), pGps))
	{
		std::cout<<"Get GPS Fail"<< std::endl;
		return false;
	}
	return true;
	// if (IsCallBackMode) {
	// 	auto function = []( const char* mainVehicleId, SimOne_Data_Gps *pGps){
	// 			dbg_data.dump_gps(mainVehicleId, pGps);
	// 	};
	// 	SimOneAPI::SetGpsUpdateCB(function);
	// }
	// else {
	// 	std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
	// 	int lastFrame = 0;
	// 	while(1)
	// 	{
	// 		bool flag = SimOneAPI::GetGps(mainVehicleId.c_str(), pGps.get());
	// 		if (flag && pGps->frame != lastFrame)
	// 		{
	// 			lastFrame  = pGps->frame;
	// 			dbg_data.dump_gps(mainVehicleId.c_str(), pGps.get());
	// 		}
	// 		if (!flag)
	// 		{
	// 			std::cout<<"Get GPS Fail"<< std::endl;
	// 		}
	// 		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	// 	}
	// }
}

bool simapi::SimAPI_SetGpsUpdateCB(void(*cb)(const char* mainVehicleId, SimOne_Data_Gps *pGps))
{
	if (!SimOneAPI::SetGpsUpdateCB(cb))
	{
		std::cout << "SetGpsUpdateCB Failed!" << std::endl;
		return false;
	}

	return true;
}

// void simapi::Get_GPS(Veh_Data_Para &vp)
// {
// 	std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
// 	int lastFrame = 0;
// 	while (1)
// 	{
// 		bool flag = SimOneAPI::GetGps(mainVehicleId.c_str(), pGps.get());
// 		if (flag && pGps->frame != lastFrame)
// 		{
// 			lastFrame = pGps->frame;
// 			dbg_data.dump_gps(mainVehicleId.c_str(), pGps.get());
// 
// 			// vp.SteeringAngleSpd;//方向盘转向角速度
// 			vp.SteeringAngle = pGps->steering;//方向盘转向角，左正右负
// 			// vp.SteeringTorsionToq;//驾驶员手力矩
// 			// vp.SteeringOverlayTorque;//转向管柱扭矩
// 			vp.WheelSpeed_AvyL1 = pGps->wheelSpeedFL;//左前轮Pitch rate(deg/s)
// 			vp.WheelSpeed_AvyR1 = pGps->wheelSpeedFR;//右前轮Pitch rate(deg/s)
// 			vp.WheelSpeed_AvyL2 = pGps->wheelSpeedRL;//左后轮Pitch rate(deg/s)
// 			vp.WheelSpeed_AvyR2 = pGps->wheelSpeedRR;//右后轮Pitch rate(deg/s)
// 			vp.Gear = pGps->gear;//挡位
// 			vp.Ax = pGps->accelX;//加速度
// 			vp.Ay = pGps->accelY;//加速度
// 			// vp.Avz;//车身Yaw rate(deg/s)
// 			vp.Xo = pGps->posX;//大地坐标系，主车X坐标
// 			vp.Yo = pGps->posY;//大地坐标系，主车Y坐标
// 			vp.Zo = pGps->posZ;//大地坐标系，主车Z坐标
// 			vp.Yaw = pGps->oriZ;//车身yaw（deg)
// 			vp.Roll = pGps->oriX;//车身Roll（deg)
// 			vp.Pitch = pGps->oriY;//车身Pitch（deg)
// 			// vp.Vx_veh;//车身坐标系Vx
// 			// vp.Vy_veh;//车身坐标系Vy
// 		}
// 		if (!flag)
// 		{
// 			std::cout << "Get GPS Fail" << std::endl;
// 		}
// 		std::this_thread::sleep_for(std::chrono::milliseconds(10));
// 	}
// }

bool simapi::SimAPI_GetRestLength(double& restLength, std::vector<long>& roadIdList, std::map<std::string, bool>& route_section_ori)
{
	// ------------ Route Length Till Next Junction Begin ------------
	std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
	if (!SimOneAPI::GetGps(mainVehicleId.c_str(), pGps.get()))
	{
		std::cout << "Get GPS Failed" << std::endl;
		return false;
	}
	
	std::vector<string> routeToJunctionLanesString = {};
	double restSuccessorLength = 0;
	double restPredecessorLength = 0;

	SSD::SimString laneName;
	double s, t, s_toCenterLine, t_toCenterLine;
	SSD::SimPoint3D pos = {pGps->posX, pGps->posY, pGps->posZ};
	if (!SimOneAPI::GetNearMostLane(pos, laneName, s, t, s_toCenterLine, t_toCenterLine))
	{
		return false;
	}

	// std::cout << "----------------- laneName ----------------- : " << laneName.GetString() << std::endl;

	// get road id Road_Section_Lane
	auto e_road = std::string(laneName.GetString()).find_first_of('_');
	std::string roadId = std::string(laneName.GetString()).substr(0, e_road); // Road
	// std::cout << "----------------- roadId -----------------" << roadId << std::endl;


	auto iter = std::find(roadIdList.begin(), roadIdList.end(), std::stol(roadId));
	if (iter == roadIdList.end())
	{
		// std::cout << "not in nativate list ======================" << std::endl;
		return false;
	}

	HDMapStandalone::MSideState sideState;
	if (!SimOneAPI::IsInsideLane(pos, laneName, sideState))
	{
		// std::cout << "not IsInsideLane======================" << std::endl;
		return false;
	}

	long junctionId = -1;
	if (SimOneAPI::IsInJunction(laneName, junctionId))
	{
		// std::cout << "laneName is in Junction======================" <<std::endl;
		return true;
	}
	// else
	// {
	// 	restLength = SimOneAPI::GetLaneLength(laneName) - s;
	// }
	
	auto e_section = std::string(laneName.GetString()).find_last_of('_');
	// std::string sectionId = std::string(laneName.GetString()).substr(e_road+1, e_section); // Road
	std::string laneId = std::string(laneName.GetString()).substr(e_section+1, laneName.Length()); // Road
	// std::cout << "----------------- roadId ----------------- : " << roadId << std::endl;
	// std::cout << "----------------- laneId ----------------- : " << laneId << std::endl;


	bool lane_dir = route_section_ori[roadId];
	bool cur_lane_dir = std::stoi(laneId.c_str())>0?true:false;

	// std::map<std::string, bool>::iterator it;
	// for (it = route_section_ori.begin(); it != route_section_ori.end(); ++it) {
	//     std::cout << "Key: " << it->first << ", Value: " << it->second << std::endl;
	// }
	// std::cout << "lane_dir : " << lane_dir << ", cur_lane_dir :" << cur_lane_dir << std::endl;

	if (lane_dir ^ cur_lane_dir)
	{
		restLength = s;
		// std::cout << "laneName reverse ======================" <<std::endl;
		simapi::GetPredecessorTillJunction(laneName, routeToJunctionLanesString);

		for (auto &predecessor : routeToJunctionLanesString)
		{
			// cout << "---------successor---------------------" << successor << "    " << laneName.GetString() << std::endl;
			if (predecessor.compare(std::string(laneName.GetString())) != 0)
			{
				double predecessorLength = SimOneAPI::GetLaneLength(predecessor.c_str());
				restPredecessorLength += predecessorLength;
			}
		}
		restLength += restPredecessorLength;
	}
	else
	{
		restLength = SimOneAPI::GetLaneLength(laneName) - s;
		long jid = simapi::GetSuccessorTillJunction(laneName, routeToJunctionLanesString);

		for (auto &successor : routeToJunctionLanesString)
		{
			// cout << "---------successor---------------------" << successor << "    " << laneName.GetString() << std::endl;
			if (successor.compare(std::string(laneName.GetString())) != 0)
			{
				double successorLength = SimOneAPI::GetLaneLength(successor.c_str());
				restSuccessorLength += successorLength;
			}
		}
		
		restLength += restSuccessorLength;
	}
	// std::cout << "====== restLength: " << restLength << "m" << std::endl;
	return true;
}

void simapi::SimAPI_V2XInfo(bool IsCallBackMode)
{
	if (IsCallBackMode) {
		auto function = [](const char* mainVehicleId, const char* sensorId, SimOne_Data_V2XNFS *pDetections) {
			std::cout << "########### sensorId:" << sensorId<<" SetV2XInfoUpdateCB strlen= "<<pDetections->V2XMsgFrameSize <<"  "<<pDetections->MsgFrameData << std::endl;
		};
		SimOneAPI::SetV2XInfoUpdateCB(function);
	}
	else {
		std::unique_ptr<SimOne_Data_V2XNFS> pDetections = std::make_unique<SimOne_Data_V2XNFS>();
		while (1) {
			if (SimOneAPI::GetV2XInfo(mainVehicleId.c_str(), "obu1", ESimOne_V2X_MessageFrame_PR::ESimOne_V2X_MessageFrame_PR_rsiFrame, pDetections.get())) {
				std::cout << "########### GetV2XInfo strlen = " << strlen(pDetections->MsgFrameData) << "  " << pDetections->MsgFrameData << std::endl;
				std::this_thread::sleep_for(std::chrono::milliseconds(20));
			}
			else {
				std::cout << "########### GetV2XInfo Fail" << std::endl;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}
	}
}

bool simapi::SimAPI_HDMap_ALL(const std::vector<std::string> &apiNames)
{
	int success_count = 0;
	if (SimOneAPI::LoadHDMap(3)) {
		for (auto apiName : apiNames) {
			std::cout << "get hdmap data success" << std::endl;
			if (apiName == "GetTrafficSignList") {
				SSD::SimVector<HDMapStandalone::MSignal> list;
				SimOneAPI::GetTrafficSignList(list);
				int listSize = list.size();
				if (listSize > 0) {
					std::cout << ">>>>>>>>>>>>>>>>>>>>>  GetTrafficSignList Size = " << listSize << std::endl;
					success_count++;
				}
			}
			else if (apiName == "GetTrafficLightList") {
				SSD::SimVector<HDMapStandalone::MSignal> list;
				SimOneAPI::GetTrafficLightList(list);
				int listSize = list.size();
				if (listSize > 0) {
					std::cout << ">>>>>>>>>>>>>>>>>>>>>  GetTrafficLightList Size = " << listSize << std::endl;
					success_count++;
				}
				for (auto signal : list){
					std::unique_ptr<SimOne_Data_TrafficLight> pDetections = std::make_unique<SimOne_Data_TrafficLight>();
					if(SimOneAPI::GetTrafficLight(mainVehicleId.c_str(), signal.id, pDetections.get()))
						dbg_data.dump_traffic_light(mainVehicleId.c_str(), signal.id, pDetections.get());
				}
			}
			else if (apiName == "GetCrossHatchList") {
				SSD::SimString id;
				SSD::SimVector<HDMapStandalone::MObject> crossHatchList;
				double s, t, s_toCenterLine, t_toCenterLine;
				std::unique_ptr<SimOne_Data_Gps> pDetections = std::make_unique<SimOne_Data_Gps>();
				SimOneAPI::GetGps(mainVehicleId.c_str(), pDetections.get());
				SSD::SimPoint3D pos = { pDetections->posX,pDetections->posY ,pDetections->posZ };
				SimOneAPI::GetNearMostLane(pos, id, s, t, s_toCenterLine, t_toCenterLine);
				SimOneAPI::GetCrossHatchList(id, crossHatchList);
				int listSize = crossHatchList.size();
				if (listSize > 0) {
					std::cout << ">>>>>>>>>>>>>>>>>>>>>  getCrossHatchList Size = " << listSize << std::endl;
					success_count++;
				}
			}
			else if (apiName == "GetLaneLink") {
				SSD::SimString id;
				HDMapStandalone::MLaneLink laneLink;
				double s, t, s_toCenterLine, t_toCenterLine;
				std::unique_ptr<SimOne_Data_Gps> pDetections = std::make_unique<SimOne_Data_Gps>();
				SimOneAPI::GetGps(mainVehicleId.c_str(), pDetections.get());
				SSD::SimPoint3D pos = { pDetections->posX,pDetections->posY ,pDetections->posZ };
				SimOneAPI::GetNearMostLane(pos, id, s, t, s_toCenterLine, t_toCenterLine);
				SimOneAPI::GetLaneLink(id, laneLink);
				std::string laneID = id.GetString();
				int listSize = laneLink.successorLaneNameList.size();
				SSD::SimString leftLaneName = laneLink.leftNeighborLaneName;
				if (listSize > 0 && !leftLaneName.Empty()) {
					std::cout << ">>>>>>>>>>>>>>>>>>>>>  getLaneLink successorLaneNameList Size = " << listSize << std::endl;
					success_count++;
				}
			}
		}
	}
	else {
		std::cout << "#####################  LoadHDMap Data fail!!!" << std::endl;
		std::cout << "#####################  SimAPI_HDMap_ALL fail!!!" << std::endl;
		return false;
	}
	if (success_count == apiNames.size()) {
		std::cout << "#####################  SimAPI_HDMap_ALL success!!!" << std::endl;
		return true;
	}
	else {
		std::cout << "#####################  SimAPI_HDMap_ALL fail!!!" << std::endl;
		return false;
	}
}

void simapi::GetPredecessorTillJunction(const SSD::SimString& laneName, std::vector<string>& routeToJunctionLanes, int levelsOfRecursion)
{
	long juncId = -1;

	if (levelsOfRecursion < 0)
	{
		// std::cout << "================= levelsOfRecursion has reached limit" <<std::endl;
		return;
	}
	if (SimOneAPI::IsInJunction(laneName, juncId))
	{
		// std::cout << "=========== Current lane is in junction : " << laneName.GetString() << std::endl;
		return;
	}

	HDMapStandalone::MLaneLink laneLink;
	SimOneAPI::GetLaneLink(laneName, laneLink);

	routeToJunctionLanes.push_back(laneName.GetString());
	// std::cout << "========================== laneName : " << laneName.GetString() << std::endl;
	for (SSD::SimString &predecessor : laneLink.predecessorLaneNameList)
	{
		levelsOfRecursion -= 1;
		GetPredecessorTillJunction(predecessor, routeToJunctionLanes, levelsOfRecursion);
		return;
	}
}

long simapi::GetSuccessorTillJunction(const SSD::SimString& laneName, std::vector<string>& routeToJunctionLanes, int levelsOfRecursion)
{
	long juncId = -1;

	if (levelsOfRecursion < 0)
	{
		// std::cout << "================= levelsOfRecursion has reached limit" <<std::endl;
		return juncId;
	}
	if (SimOneAPI::IsInJunction(laneName, juncId))
	{
		// std::cout << "=========== Current lane is in junction : " << laneName.GetString() << std::endl;
		return juncId;
	}

	SSD::SimStringVector successors;
	HDMapStandalone::MLaneLink laneLink;
	SimOneAPI::GetLaneLink(laneName, laneLink);

	routeToJunctionLanes.push_back(laneName.GetString());
	// std::cout << "========================== laneName : " << laneName.GetString() << std::endl;
	for (SSD::SimString &successor : laneLink.successorLaneNameList)
	{
		levelsOfRecursion -= 1;
		long jId = GetSuccessorTillJunction(successor, routeToJunctionLanes, levelsOfRecursion);
		return jId;
	}
	// return juncId;








	// long juncId = -1;
	// std::cout << "levelsOfRecursion : " << levelsOfRecursion  <<std::endl;
	// if (levelsOfRecursion == 0)
	// {

	// 	// routeToJunctionLanes.push_back(laneName);
	// 	// std::cout << "============= 4 ============= laneName : " << laneName.GetString() << std::endl;
	// 	return juncId;
	// }

	// SSD::SimStringVector successors;
	// HDMapStandalone::MLaneLink laneLink;
	// SimOneAPI::GetLaneLink(laneName, laneLink);

	// // if (SimOneAPI::IsInJunction(laneName, juncId))
	// // {
	// // 	return juncId;
	// // }

	// for (auto &successor : laneLink.successorLaneNameList)
	// {
	// 	// It is not possible that successorList belong to more than 1 junction, so just return early if found any.
	// 	if (SimOneAPI::IsInJunction(successor, juncId))
	// 	{
	// 		std::cout << "=========== 1 ============ inJunction : " << successor.GetString() << std::endl;
	// 		routeToJunctionLanes.push_back(laneName);
	// 		return juncId;
	// 	}
	// }

	// for (auto &successor : laneLink.successorLaneNameList)
	// {
	// 	// If same road but different lane section or different road, detect recursively.
	// 	levelsOfRecursion -= 1;
	// 	long jId = GetSuccessorTillJunction(successor, routeToJunctionLanes, levelsOfRecursion);
	// 	if (jId != -1)
	// 	{
	// 		// std ::cout << "============ 3 ============== jId : " << jId << std::endl;
	// 		// routeToJunctionLanes.push_back(laneName);
	// 		// std::cout << "============= 4 ============= laneName : " << laneName.GetString() << std::endl;
	// 		return jId;
	// 	}
	// }
	// return juncId;
}

// bool simapi::SimAPI_GetRestLength(double& restLength)
// {
// 	// ------------ Tutol Route Length Begin ------------
// 	// std::unique_ptr<SimOne_Data_WayPoints> pWayPoints = std::make_unique<SimOne_Data_WayPoints>();
// 	// std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
// 
// 	// if (!SimOneAPI::GetWayPoints(mainVehicleId.c_str(), pWayPoints.get()))
// 	// {
// 	// 	std::cout << "Get WayPoints Failed" << std::endl;
// 	// 	return false;
// 	// }
// 	// if (!SimOneAPI::GetGps(mainVehicleId.c_str(), pGps.get()))
// 	// {
// 	// 	std::cout << "Get GPS Failed" << std::endl;
// 	// 	return false;
// 	// }
// 	
// 	// SSD::SimPoint3DVector inputPoints;
// 	// SSD::SimPoint3D p;
// 	// p.x = pGps->posX;
// 	// p.y = pGps->posY;
// 	// p.z = pGps->posZ;
// 	// inputPoints.push_back(p);
// 	// p.x = pWayPoints->wayPoints[pWayPoints->wayPointsSize-1].posX;
// 	// p.y = pWayPoints->wayPoints[pWayPoints->wayPointsSize-1].posY;
// 	// inputPoints.push_back(p);
// 
// 	// std::cout << "WayPoints: " << std::endl;
// 	// for (int i=0; i<pWayPoints->wayPointsSize; i++)
// 	// {
// 	// 	SSD::SimPoint3D p;
// 	// 	p.x = pWayPoints->wayPoints[i].posX;
// 	// 	p.y = pWayPoints->wayPoints[i].posY;
// 	// 	p.z = 0;
// 	// 	inputPoints.push_back(p);
// 	// 	std::cout << "[" << p.x << "," << p.y << "] ";
// 	// }
// 	// std::cout << std::endl;
// 
// 	// SSD::SimPoint3DVector route;
// 	// SSD::SimVector<int> indexOfValidPoints;
// 	// if (!SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, route))
// 	// {
// 	// 	std::cout << "GenerateRouteeFailed!" << std::endl;
// 	// 	return false;
// 	// }
// 
// 	// int n = static_cast<int>(route.size());
// 	// if (n > 0)
// 	// {
// 	// 	std::cout << "------ route " << std::endl;
// 	// 	for (int i = 0; i < n-1; ++i)
// 	// 	{
// 	// 		std::cout << '[' << route[i].x << ", " << route[i].y << ']' << std::endl;
// 	// 	}
// 	// 	std::cout << '[' << route[n-1].x << ',' << route[n-1].y << ']' << std::endl;
// 	// 	std::cout << "===================================" << std::endl;
// 	// }
// 
// 	// restLength = route.size();
// 	// std::cout << "====== restLength: " << restLength << "m" << std::endl;
// 
// 	// ------------ Tutol Route Length End ------------
// 
// 	// ------------ Route Length Till Next Junction Begin ------------
// 	std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
// 	if (!SimOneAPI::GetGps(mainVehicleId.c_str(), pGps.get()))
// 	{
// 		std::cout << "Get GPS Failed" << std::endl;
// 		return false;
// 	}
// 	
// 	static SSD::SimStringVector routeToJunctionLanes;
// 	static SSD::SimString preLaneName;
// 	static std::queue<double> restLaneLengths;
// 	static double restSuccessorLength = 0;
// 	static bool isNewRoute = true;
// 
// 	SSD::SimString laneName;
// 	double s, t, s_toCenterLine, t_toCenterLine;
// 	SSD::SimPoint3D pos = {pGps->posX, pGps->posY, pGps->posZ};
// 	if (SimOneAPI::GetNearMostLane(pos, laneName, s, t, s_toCenterLine, t_toCenterLine))
// 	{
// 		// auto e = std::string(laneName.GetString()).find_first_of('_');
// 		// std::string roadId = std::string(laneName.GetString()).substr(0, e);
// 		restLength = SimOneAPI::GetLaneLength(laneName) - s;
// 	}
// 	// printf("posx : %f, posy : %f, posz : %f\n", pos.x, pos.y, pos.z);
// 	// printf("=============laneName:%s\n", laneName.GetString());
// 	if (isNewRoute)
// 	{
// 		isNewRoute = false;
// 		std::queue<double> emptyQ;
// 		restLaneLengths.swap(emptyQ);
// 		routeToJunctionLanes.clear();
// 
// 
// 		preLaneName = SSD::SimString(std::string(laneName.GetString()).substr(0, std::string(laneName.GetString()).find_last_of('_')).c_str());
// 		simapi::GetSuccessorTillJunction(laneName, routeToJunctionLanes);
// 		// printf("=============routeToJunctionLanes : \n");
// 		for (SSD::SimString &successor : routeToJunctionLanes)
// 		{
// 			if (successor != laneName)
// 			{
// 				// std::cout << "\t ======== " << successor.GetString()<< " : " << SimOneAPI::GetLaneLength(successor) << std::endl;
// 				double successorLength = SimOneAPI::GetLaneLength(successor);
// 				restLaneLengths.push(successorLength);
// 				restSuccessorLength += successorLength;
// 			}
// 		}
// 	}
// 
// 	HDMapStandalone::MLaneLink laneLink;
// 	SimOneAPI::GetLaneLink(laneName, laneLink);
// 
// 	for (SSD::SimString &predecessor : laneLink.predecessorLaneNameList)
// 	{
// 
// 		std::string str(predecessor.GetString());
// 		std::vector<std::string> elems;
// 		auto e = str.find_last_of('_');
// 		SSD::SimString lane_section = SSD::SimString(str.substr(0, e).c_str());
// 		// std::cout << "--------------------------- " << lane_section.GetString()  << std::endl;
// 
// 		if (preLaneName == lane_section && !restLaneLengths.empty())
// 		{
// 			// if (!restLaneLengths.empty())
// 			// {
// 				// std::cout << "\t restSuccessorLength " << restSuccessorLength << ", restLaneLengths.front() " << restLaneLengths.front()<< std::endl; 
// 				restSuccessorLength -= restLaneLengths.front();
// 				restLaneLengths.pop();
// 				preLaneName = SSD::SimString(std::string(laneName.GetString()).substr(0, std::string(laneName.GetString()).find_last_of('_')).c_str());
// 				// break;
// 
// 			// }
// 			// else 
// 			// {
// 			// 	std::cout << "\t preLaneName " << preLaneName.GetString() << std::endl; 
// 			// 	long junctionId = -1;
// 			// 	if (SimOneAPI::IsInJunction(laneName, junctionId))
// 			// 	{
// 			// 		std::cout << "laneName is in Junction======================" <<std::endl;
// 			// 		isNewRoute = true;
// 			// 		break;
// 			// 	}
// 			// }
// 		}
// 	}
// 
// 	// std::cout << "restLaneLengths.size : " << restLaneLengths.size() << std::endl;
// 	// std::cout << "restLength : " << restLength << std::endl;
// 
// 	// for(auto& successor : laneLink.successorLaneNameList)
// 	// {
// 	// 	if (successor == preLaneName)
// 	// 	{
// 	// 		std::cout << "successor : " << successor.GetString() << std::endl;
// 	// 		isNewRoute = true;
// 	// 	}
// 	// }
// 
// 	if (restLaneLengths.empty() && restLength < 1.0) 
// 	{
// 		// printf("=============== restLaneLengths is empty\n");
// 		isNewRoute = true;
// 	}
// 	bool isOnLane = false;
// 	for (SSD::SimString &successor : routeToJunctionLanes)
// 	{
// 		if (preLaneName == successor)
// 		{
// 			isOnLane = true;
// 		}
// 	}
// 
// 	// if (preLaneName == laneName)
// 	if (preLaneName == SSD::SimString(std::string(laneName.GetString()).substr(0, std::string(laneName.GetString()).find_last_of('_')).c_str()))
// 	{
// 		isOnLane = true;
// 	}
// 
// 	if (isOnLane)
// 	{
// 		restLength += restSuccessorLength;
// 	}
// 	else
// 	{
// 		return false;
// 	}
// 
// 	// std::cout << "====== restLength: " << restLength << "m " << "routeToJunctionLanes: " << std::endl;
// 	// for (auto &successor : routeToJunctionLanes)
// 	// {
// 	// 	std::cout << successor.GetString() << " : " << SimOneAPI::GetLaneLength(successor) << std::endl;
// 	// }
// 	
// 	// std::cout << "====== restLength: " << restLength << "m " << "restLaneLengths: " << restLaneLengths.front() << std::endl;
// 
// 
// 	return true;
// 
// 	// ------------ Route Length Till Next Junction End ------------
// }

// bool simapi::SimAPI_GetRestLength(double& restLength)
// {
// 	// ------------ Tutol Route Length Begin ------------
// 	// std::unique_ptr<SimOne_Data_WayPoints> pWayPoints = std::make_unique<SimOne_Data_WayPoints>();
// 	// std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
// 
// 	// if (!SimOneAPI::GetWayPoints(mainVehicleId.c_str(), pWayPoints.get()))
// 	// {
// 	// 	std::cout << "Get WayPoints Failed" << std::endl;
// 	// 	return false;
// 	// }
// 	// if (!SimOneAPI::GetGps(mainVehicleId.c_str(), pGps.get()))
// 	// {
// 	// 	std::cout << "Get GPS Failed" << std::endl;
// 	// 	return false;
// 	// }
// 	
// 	// SSD::SimPoint3DVector inputPoints;
// 	// SSD::SimPoint3D p;
// 	// p.x = pGps->posX;
// 	// p.y = pGps->posY;
// 	// p.z = pGps->posZ;
// 	// inputPoints.push_back(p);
// 	// p.x = pWayPoints->wayPoints[pWayPoints->wayPointsSize-1].posX;
// 	// p.y = pWayPoints->wayPoints[pWayPoints->wayPointsSize-1].posY;
// 	// inputPoints.push_back(p);
// 
// 	// std::cout << "WayPoints: " << std::endl;
// 	// for (int i=0; i<pWayPoints->wayPointsSize; i++)
// 	// {
// 	// 	SSD::SimPoint3D p;
// 	// 	p.x = pWayPoints->wayPoints[i].posX;
// 	// 	p.y = pWayPoints->wayPoints[i].posY;
// 	// 	p.z = 0;
// 	// 	inputPoints.push_back(p);
// 	// 	std::cout << "[" << p.x << "," << p.y << "] ";
// 	// }
// 	// std::cout << std::endl;
// 
// 	// SSD::SimPoint3DVector route;
// 	// SSD::SimVector<int> indexOfValidPoints;
// 	// if (!SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, route))
// 	// {
// 	// 	std::cout << "GenerateRouteeFailed!" << std::endl;
// 	// 	return false;
// 	// }
// 
// 	// int n = static_cast<int>(route.size());
// 	// if (n > 0)
// 	// {
// 	// 	std::cout << "------ route " << std::endl;
// 	// 	for (int i = 0; i < n-1; ++i)
// 	// 	{
// 	// 		std::cout << '[' << route[i].x << ", " << route[i].y << ']' << std::endl;
// 	// 	}
// 	// 	std::cout << '[' << route[n-1].x << ',' << route[n-1].y << ']' << std::endl;
// 	// 	std::cout << "===================================" << std::endl;
// 	// }
// 
// 	// restLength = route.size();
// 	// std::cout << "====== restLength: " << restLength << "m" << std::endl;
// 
// 	// ------------ Tutol Route Length End ------------
// 
// 	// ------------ Route Length Till Next Junction Begin ------------
// 	std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
// 	if (!SimOneAPI::GetGps(mainVehicleId.c_str(), pGps.get()))
// 	{
// 		std::cout << "Get GPS Failed" << std::endl;
// 		return false;
// 	}
// 	
// 	static SSD::SimString preLaneName;
// 	static SSD::SimStringVector routeToJunctionLanes;
// 	static std::queue<double> restLaneLengths;
// 	static double restSuccessorLength = 0;
// 	static bool isNewRoute = true;
// 
// 	bool isOnLane = false;
// 	SSD::SimString laneName;
// 	double s, t, s_toCenterLine, t_toCenterLine;
// 	SSD::SimPoint3D pos = {pGps->posX, pGps->posY, pGps->posZ};
// 	if (SimOneAPI::GetNearMostLane(pos, laneName, s, t, s_toCenterLine, t_toCenterLine))
// 	{
// 		// auto e = std::string(laneName.GetString()).find_first_of('_');
// 		// std::string roadId = std::string(laneName.GetString()).substr(0, e);
// 
// 		long junctionId = -1;
// 		if (SimOneAPI::IsInJunction(laneName, junctionId))
// 		{
// 			std::cout << "laneName is in Junction======================" <<std::endl;
// 			restLength = 0.0;
// 			isNewRoute = true;
// 			isOnLane = true;
// 			return true;
// 		}
// 		else{
// 			if (isNewRoute)
// 			{
// 				std::cout << "*******First Out Junction======================" <<std::endl;
// 				std::this_thread::sleep_for(std::chrono::milliseconds(1000));
// 			}
// 			restLength = SimOneAPI::GetLaneLength(laneName) - s;
// 		}
// 	}
// 	// printf("posx : %f, posy : %f, posz : %f\n", pos.x, pos.y, pos.z);
// 	// printf("=============laneName:%s\n", laneName.GetString());
// 	// std::cout << "isNewRoute : " <<  isNewRoute << std::endl;
// 	// double successorLength_964 = SimOneAPI::GetLaneLength("964_0_-4");
// 	// cout << "====================== successorLength_964 : " <<  successorLength_964 << endl; 
// 	if (isNewRoute)
// 	{
// 		isNewRoute = false;
// 		std::queue<double> emptyQ;
// 		restLaneLengths.swap(emptyQ);
// 		routeToJunctionLanes.clear();
// 
// 		// printf("=====================0\n");
// 
// 		long jId = simapi::GetSuccessorTillJunction(laneName, routeToJunctionLanes);
// 
// 		string tempPreLaneName = "";
// 		tempPreLaneName = std::string(laneName.GetString()).substr(0, std::string(laneName.GetString()).find_last_of('_'));
// 		preLaneName = SSD::SimString(tempPreLaneName.c_str());
// 		// preLaneName = SSD::SimString(std::string(laneName.GetString()).substr(0, std::string(laneName.GetString()).find_last_of('_')).c_str());
// 		// cout << "-=!!!!-=-=-=--New prelaneName " << preLaneName.GetString() << "      " << laneName.GetString() << "   " << tempPreLaneName << endl;
// 		// long jId = simapi::GetSuccessorTillJunction("709_0_-4", routeToJunctionLanes);
// 		// long jId = simapi::GetSuccessorTillJunction(laneName, routeToJunctionLanes);
// 		// cout << "----------------------============    " << preLaneName.GetString() << "       " << tempPreLaneName << endl;
// 		// printf("=============routeToJunctionLanes : \n");
// 		try
// 		{
// 			// cout << "!!!!!!!!!!!!!!!!    " << preLaneName.GetString() << endl;
// 			// std::cout << "routeToJunctionLanes size : " << routeToJunctionLanes.size() << std::endl;
// 			for (SSD::SimString &successor : routeToJunctionLanes)
// 			{
// 				// std::cout << "\t ========= successor : " << successor.GetString() << std::endl;
// 				// std::cout << "\t ========= laneName : " << laneName.GetString() << std::endl;
// 				// cout << "*****************    " << preLaneName.GetString() << endl;
// 
// 				if (successor != laneName)
// 				{
// 					// std::cout << "\t ======== " << successor.GetString() << " : " << SimOneAPI::GetLaneLength(successor) << std::endl;
// 					double successorLength = 0;
// 
// 					while (1)
// 					{
// 						bool getlaneSuccess = true;
// 						try
// 						{
// 							successorLength = SimOneAPI::GetLaneLength(successor);
// 						}
// 						catch (const std::exception &ee)
// 						{
// 							std::cout << "std exception: " << ee.what() << std::endl;
// 						}
// 						catch (const char* e)
// 						{
// 							std::cout << "####################Caught exception########################: " << e << std::endl;
// 							getlaneSuccess = false;
// 							isNewRoute = true;
// 							return false;
// 							// std::this_thread::sleep_for(std::chrono::milliseconds(10));
// 						}
// 						catch (...)
// 						{
// 							std::cout << "^^^^^^^^^^^^^^ Caught Other Exception ^^^^^^^^^^^^^^^^^^^^^" << std::endl;
// 						}
// 
// 						cout << "#$#$#$#   " << getlaneSuccess << endl;
// 
// 						if (getlaneSuccess)
// 						{
// 							break;
// 						}
// 						
// 					}
// 					
// 					
// 					// double successorLength = SimOneAPI::GetLaneLength(successor);
// 					// printf("=====================1\n");
// 					restLaneLengths.push(successorLength);
// 					// printf("=====================2\n");
// 					restSuccessorLength += successorLength;
// 					// printf("=====================3\n");
// 				}
// 				else{
// 					printf("===================== successor == laneName\n");
// 				}
// 			}
// 		}
// 		catch (const std::exception &ee)
// 		{
// 			std::cout << "std exception: " << ee.what() << std::endl;
// 		}
// 		catch (const char* e)
// 		{
// 			std::cout << "!!!!!!Caught exception: " << e << std::endl;
// 		}
// 		catch (...)
// 		{
// 			std::cout << "#@#@#@#@#@#@#@#@#@Caught Other Exception: " << std::endl;
// 		}
// 	}
// 
// 	HDMapStandalone::MLaneLink laneLink;
// 	SimOneAPI::GetLaneLink(laneName, laneLink);
// 
// 	for (SSD::SimString &predecessor : laneLink.predecessorLaneNameList)
// 	{
// 
// 		std::string str(predecessor.GetString());
// 		std::vector<std::string> elems;
// 		auto e = str.find_last_of('_');
// 		SSD::SimString lane_section = SSD::SimString(str.substr(0, e).c_str());
// 		// std::cout << "-------------predecessor-------------- " << lane_section.GetString()  << std::endl;
// 
// 		// cout << "$$$$$$$$$$$ " << preLaneName.GetString() << endl;
// 
// 		if (preLaneName == lane_section && !restLaneLengths.empty())
// 		{
// 				restSuccessorLength -= restLaneLengths.front();
// 				restLaneLengths.pop();
// 				preLaneName = SSD::SimString(std::string(laneName.GetString()).substr(0, std::string(laneName.GetString()).find_last_of('_')).c_str());
// 
// 				// cout << "@@@@@@@@@@@@@ " << lane_section.GetString() << "     " << preLaneName.GetString() << "          " << laneName.GetString() << endl;
// 		}
// 	}
// 
// 	std::cout << "restLaneLengths.size : " << restLaneLengths.size() << std::endl;
// 	// std::cout << "restLength : " << restLength << std::endl;
// 
// 	if (restLaneLengths.empty() && restLength == 0.0) 
// 	{
// 		printf("=============== restLaneLengths is empty\n");
// 		isNewRoute = true;
// 	}
// 	// for (SSD::SimString &successor : routeToJunctionLanes)
// 	// {
// 	// 	if (preLaneName == successor)
// 	// 	{
// 	// 		isOnLane = true;
// 	// 	}
// 	// }
// 
// 	// if (preLaneName == laneName)
// 	// cout << "#############   " << preLaneName.GetString() << "     " << laneName.GetString() << endl;
// 
// 	if (preLaneName == SSD::SimString(std::string(laneName.GetString()).substr(0, std::string(laneName.GetString()).find_last_of('_')).c_str()))
// 	{
// 		isOnLane = true;
// 	}
// 
// 	// cout << "compare prelane ------- " << preLaneName.GetString() << " " << (std::string(laneName.GetString()).substr(0, std::string(laneName.GetString()).find_last_of('_')).c_str()) << " " << isOnLane << endl;
// 
// 	// if (preLaneName == laneName)
// 	if (isOnLane)
// 	{
// 		restLength += restSuccessorLength;
// 	}
// 	else
// 	{
// 		return false;
// 	}
// 	if (!routeToJunctionLanes.empty())
// 	{
// 
// 	std::cout << "====== restLength: " << restLength << "m " << "routeToJunctionLanes: " << std::endl;
// 	for (SSD::SimString &successor : routeToJunctionLanes)
// 	{
// 		std::cout << successor.GetString() << " : " << SimOneAPI::GetLaneLength(successor) << std::endl;
// 	}
// 	}
// 	
// 	// std::cout << "====== restLength: " << restLength << "m " << "restLaneLengths: " << restLaneLengths.front() << std::endl;
// 	// std::cout << "====== restLength: " << restLength << "m " << std::endl; 
// 
// 
// 	return true;
// 
// 	// ------------ Route Length Till Next Junction End ------------
// }

bool simapi::SimAPI_GetRouteInfo(double& routeLength, int& routeCountDown, std::vector<long>& roadIdList, std::map<std::string, bool>& route_section_ori)
{
	static double length = 0.0;
	static std::vector<long> passedIdList;

	if (length < 1.0)
	{
		std::unique_ptr<SimOne_Data_WayPoints> pWayPoints = std::make_unique<SimOne_Data_WayPoints>();
		std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();

		if (!SimOneAPI::GetWayPoints(mainVehicleId.c_str(), pWayPoints.get()))
		{
			std::cout << "Get WayPoints Failed" << std::endl;
			return false;
		}

		if (pWayPoints->wayPointsSize < 2)
		{
			return false;
		}

		SSD::SimPoint3DVector inputPoints;
		for (int i = 0; i < pWayPoints->wayPointsSize; i++)
		{
			SSD::SimPoint3D p;
			p.x = pWayPoints->wayPoints[i].posX;
			p.y = pWayPoints->wayPoints[i].posY;
			p.z = 0;
			inputPoints.push_back(p);
		}

		SSD::SimPoint3DVector route;
		SSD::SimVector<int> indexOfValidPoints;
		if (!SimOneAPI::GenerateRoute(inputPoints, indexOfValidPoints, route))
		{
			std::cout << "GenerateRouteeFailed!" << std::endl;
			return false;
		}

		double s, t, s_toCenterLine, t_toCenterLine;
		SSD::SimString laneName;
		SimOneAPI::GetNearMostLane(route[0], laneName, s, t, s_toCenterLine, t_toCenterLine);
		auto e_lane = std::string(laneName.GetString()).find_last_of('_');
		std::string laneId = std::string(laneName.GetString()).substr(e_lane+1, laneName.Length());
		bool ori = std::atoi(laneId.c_str())>0?true:false;

		auto e_road = std::string(laneName.GetString()).find_first_of('_');
		std::string preRoadId = std::string(laneName.GetString()).substr(0, e_road); // Road
		route_section_ori[preRoadId] = ori;
		for (int i=0; i<route.size(); i++)
		{
			SimOneAPI::GetNearMostLane(route[i], laneName, s, t, s_toCenterLine, t_toCenterLine);
			auto e_road = std::string(laneName.GetString()).find_first_of('_');
			std::string roadId = std::string(laneName.GetString()).substr(0, e_road); // Road
			auto e_lane = std::string(laneName.GetString()).find_last_of('_');
			std::string laneId = std::string(laneName.GetString()).substr(e_lane+1, laneName.Length());

			if (roadId != preRoadId)
			{
				preRoadId = roadId;
				ori = std::atoi(laneId.c_str())>0?true:false;
				route_section_ori[roadId] = ori;
			}
		}

		indexOfValidPoints.clear();
		SSD::SimVector<long> roadId_List;
		if (!SimOneAPI::Navigate(inputPoints, indexOfValidPoints, roadId_List))
		{
			std::cout << "NavigateFailed!" << std::endl;
			return false;
		}

		int roadIdListSize = static_cast<int>(roadId_List.size());
		if (roadIdListSize <= 0)
		{
			std::cout << "Null Valid Points!" << std::endl;
			return false;
		}

		for (auto roadId: roadId_List)
		{
			roadIdList.push_back(roadId);
		}

		length = route.size();
	}

	routeLength = length;

	std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
	if (!SimOneAPI::GetGps(mainVehicleId.c_str(), pGps.get()))
	{
		std::cout << "Get GPS Failed" << std::endl;
		return false;
	}

	SSD::SimString laneId;
	double s, t, s_toCenterLine, t_toCenterLine;
	SSD::SimPoint3D pos = {pGps->posX, pGps->posY, pGps->posZ};
	if(!SimOneAPI::GetNearMostLane(pos, laneId, s, t, s_toCenterLine, t_toCenterLine))
	{
		std::cout << "GetNearMostLane Fail" << endl;
		return false;
	}

	auto e = std::string(laneId.GetString()).find_first_of('_');
	std::string sectionId = std::string(laneId.GetString()).substr(0, e);
	// std::cout << "-------------- laneId: " << sectionId << std::endl;

	// std::cout << "------ roadIdList " << std::endl;
	// std::cout << "[";
	int roadIdIndex = 0;
	for (int i = 0; i < roadIdList.size(); ++i)
	{
		if (roadIdList[i] == std::stol(sectionId))
		{
			if (0 == roadIdIndex)
			{
				roadIdIndex = i;
				break;
			}
		}
		// if (i == roadIdList.size() - 1)
		// {
		// 	std::cout << roadIdList[roadIdList.size() - 1] << "]" << std::endl;
		// }
		// else
		// {
		// 	std::cout << roadIdList[i] << ", ";
		// }
	}

	if (0 != roadIdIndex)
	{
		passedIdList.push_back(roadIdList[0]);
		roadIdList.erase(roadIdList.begin());
	}

	// std::cout << "------ passedIdList " << std::endl;
	if (passedIdList.size() > 0)
	{
		// std::cout << "[";
		int passedIdIndex = 0;
		for (int i = 0; i < passedIdList.size(); ++i)
		{
			if (passedIdList[i] == std::stol(sectionId))
			{
				if (0 == passedIdIndex)
				{
					passedIdIndex = i;
					break;
				}
			}
			// if (i == passedIdList.size() - 1)
			// {
			// 	std::cout << passedIdList[passedIdList.size() - 1] << "]" << std::endl;
			// }
			// else
			// {
			// 	std::cout << passedIdList[i] << ", ";
			// }
		}

		// for (int i = passedIdList.size() - 1; i >= 0; --i)
		// {
		// 	if (passedIdList[i] == std::stol(sectionId))
		// 	{
		// 		if (passedIdIndex == -1)
		// 		{
		// 			passedIdIndex = i;
		// 		}
		// 	}
		// 	if (i == 0)
		// 	{
		// 		std::cout << passedIdList[0] << "]" << std::endl;
		// 	}
		// 	else
		// 	{
		// 		std::cout << passedIdList[passedIdList.size() - 1 - i] << ", ";
		// 	}
		// }

		if (0 != passedIdIndex)
		{
			// roadIdList.push_back(passedIdList[passedIdIndex]);
			roadIdList.insert(roadIdList.begin(), passedIdList[passedIdList.size() - 1]);
			passedIdList.erase(passedIdList.end() - 1);
		}
	}

	// std::cout << "roadIdList Size: " << roadIdList.size() << std::endl;
	// std::cout << "passedIdList Size: " << passedIdList.size() << std::endl;
	// std::cout << "=================================" << std::endl;

	// SSD::SimStringVector sectionLaneList;
	// if (!SimOneAPI::GetSectionLaneList(laneId, sectionLaneList))
	// {
	// 	std::cout << "GetSectionLaneListFailed!" << std::endl;
	// 	return false;
	// }
	// std::cout << "------ sectionLaneList: " << std::endl;
	// for (const auto &lane : sectionLaneList)
	// {
	// 	std::cout << lane.GetString() << std::endl;
	// }

	routeCountDown = roadIdList.size();

	return true;
}

void simapi::SimAPI_GetNearLanes()
{
	std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
	if (!SimOneAPI::GetGps(mainVehicleId.c_str(), pGps.get()))
	{
		std::cout << "Get GPS Fail" << std::endl;
		return;
	}
	SSD::SimPoint3D pos(pGps->posX, pGps->posY, pGps->posZ);
	double distance = 3.0;
	SSD::SimStringVector nearLanes;
	if(!SimOneAPI::GetNearLanes(pos, distance, nearLanes))
	{
		std::cout << "GetNearLanes Failed!" << std::endl;
		return;
	}
	if (nearLanes.size() == 0)
	{
		std::cout << "nearLanes size: " << nearLanes.size() << std::endl;
		return;
	}
	for (int i=0; i<nearLanes.size(); i++)
	{
		std::cout << "nearLanes[" << i << "]: " << nearLanes[i].GetString();
	}
}

void simapi::SimAPI_GetTrafficLightList(SSD::SimString &laneName, std::vector<MainTrafficLight_t> &lights)
{
	SSD::SimVector<HDMapStandalone::MSignal> list;
	SimOneAPI::GetSpecifiedLaneTrafficLightList(laneName, list);
	for (int i=0; i<list.size(); i++)
	{
		MainTrafficLight_t mtl;
		mtl.id = list[i].id;
		mtl.pt = list[i].pt;
		lights.push_back(mtl);
	}
}

bool simapi::SimAPI_IsInJunction(SSD::SimString &mvLaneName)
{
		long junctionId = 0;
		return SimOneAPI::IsInJunction(mvLaneName, junctionId);
}

void simapi::SimAPI_RegisterVehicleState()
{
	// ------------ RegisterSimOneVehicleState Start ------------
	ESimOne_Data_Vehicle_State index[INDEX_SIZE];
	for (int i = 0; i < INDEX_SIZE; i++)
	{
		index[i] = (ESimOne_Data_Vehicle_State)i;
	}
	while (true)
	{
		if (!SimOneAPI::RegisterVehicleState(mainVehicleId.c_str(), index, INDEX_SIZE))
		{
			printf("\033[1m\033[31mSimOneAPI::RegisterVehicleState Faile!\033[0m\n");
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			continue;
		}
		break;
	}
	printf("RegisterVehicleState Done!\n");
}

void simapi::SimAPI_SetDrive(Control_t &ctrl, Control_Mode ctlMode)
{
	std::unique_ptr<SimOne_Data_Control> pCtrl = std::make_unique<SimOne_Data_Control>();
	switch(ctlMode)
	{
		case Control_Mode_Percent:
			pCtrl->steeringMode = ESimOne_Steering_Mode::ESimOne_Steering_Mode_Percent;
			pCtrl->throttleMode = ESimOne_Throttle_Mode::ESimOne_Throttle_Mode_Percent;
			pCtrl->brakeMode = ESimOne_Brake_Mode::ESimOne_Brake_Mode_Percent;
			pCtrl->throttle = ctrl.throttle / 100.0;
			break;
		case Control_Mode_Speed:
			pCtrl->steeringMode = ESimOne_Steering_Mode::ESimOne_Steering_Mode_Percent;
			pCtrl->throttleMode = ESimOne_Throttle_Mode::ESimOne_Throttle_Mode_Speed;
			pCtrl->brakeMode = ESimOne_Brake_Mode::ESimOne_Brake_Mode_Percent;
			pCtrl->throttle = ctrl.throttle;
			break;
		case Control_Mode_Torque:
		case Control_Mode_Accel:
		case Control_Mode_EngineAV:
		case Control_Mode_WheelTorque:
		default:;
	}
	pCtrl->timestamp = ctrl.timestamp;
	pCtrl->steering = ctrl.steering;
	pCtrl->brake = ctrl.brake / 100.0;

	// std::cout << "SimAPI_SetDrive: ctrl.timestamp : " << ctrl.timestamp << std::endl;
	// std::cout << "SimAPI_SetDrive: ctrl.steering : " << ctrl.steering << std::endl;
	// std::cout << "SimAPI_SetDrive: ctrl.throttle : " << ctrl.throttle << std::endl;
	// std::cout << "SimAPI_SetDrive: ctrl.brake : " << ctrl.brake << std::endl;
	// std::cout << "SimAPI_SetDrive: ctrl.gear : " << ctrl.gear << std::endl;

	switch (ctrl.gear)
	{
		case 0x5: // Parking
			pCtrl->gear = ESimOne_Gear_Mode::ESimOne_Gear_Mode_Parking;
			break;
		case 0x6: // Reverse
			pCtrl->gear = ESimOne_Gear_Mode::ESimOne_Gear_Mode_Reverse;
			break;
		case 0x7: // Neutral
			pCtrl->gear = ESimOne_Gear_Mode::ESimOne_Gear_Mode_Neutral;
			break;
		case 0x8: // Drive
		case 0x9: // Sport
			pCtrl->gear = ESimOne_Gear_Mode::ESimOne_Gear_Mode_Drive;
			break;
		default:
			pCtrl->gear = ESimOne_Gear_Mode::ESimOne_Gear_Mode_Drive;
			break;
	}


	// std::cout << "------ sys_time_us: " << pCtrl->timestamp << std::endl;
	// std::cout << "------ steering: " << pCtrl->steering << std::endl;
	// std::cout << "------ throttle: " << pCtrl->throttle << std::endl;
	// std::cout << "------ brake: " << pCtrl->brake << std::endl;
	// std::cout << "------ gear_mode: " << pCtrl->gear << std::endl;


	// pCtrl->isManualGear = false;

	if (!SimOneAPI::SetDrive("0", pCtrl.get()))
	{
		printf("Set Drive Failed!\n");
	}
	printf("Set Drive Done!\n");

	// while (true)
	// {
	// 	std::unique_ptr<SimOne_Data_Gps> gpsInfo = std::make_unique<SimOne_Data_Gps>();
	// 	if (!SimOneAPI::GetGps(0, gpsInfo.get()))
	// 	{
	// 		std::cout << "Get Gps Failed!" << std::endl;
	// 		return;
	// 	}
	// 	std::unique_ptr<SimOne_Data_Control> pCtrl = std::make_unique<SimOne_Data_Control>();
	// 	pCtrl->timestamp = gpsInfo->timestamp;
	// 	pCtrl->steeringMode = ESimOne_Steering_Mode::ESimOne_Steering_Mode_SteeringWheelAngle;
	// 	pCtrl->steering = 0;
	// 	pCtrl->throttleMode = ESimOne_Throttle_Mode::ESimOne_Throttle_Mode_Accel;
	// 	pCtrl->throttle = 0; // 4;
	// 	pCtrl->gear = ESimOne_Gear_Mode::ESimOne_Gear_Mode_Drive;
	// 	pCtrl->isManualGear = false;
	// 	std::cout << "sys_time_us: " << pCtrl->timestamp << std::endl;
	// 	std::cout << "steering: " << pCtrl->steering << std::endl;
	// 	std::cout << "throttle: " << pCtrl->throttle;
	// 	std::cout << "gear_mode: " << pCtrl->gear;
	// 	std::cout << "------ SetDrive ------";
	// 	if (!SimOneAPI::SetDrive(0, pCtrl.get()))
	// 	{
	// 		std::cout << "Set Drive Failed!" << std::endl;
	// 	}
	// 	SimAPI_SetSignalLights();
	// 	// static int counter = 0;
	// 	// if (counter == 11)
	// 	// {
	// 	// 	SimOneAPI::SetDriveMode(mainVehicleId.c_str(), ESimOne_Drive_Mode_API);
	// 	// 	std::cout << std::endl << "turn SimOneDriver" << std::endl;
	// 	// }
	// 	// if (counter == 21)
	// 	// {
	// 	// 	SimOneAPI::SetDriveMode(mainVehicleId.c_str(), ESimOne_Drive_Mode_Driver);
	// 	// 	std::cout << std::endl << "turn API" << std::endl;
	// 	// }
	// 	// counter++;
	// 	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	// }
}

// void simapi::SimAPI_SetPose(std::string& str)
void simapi::SimAPI_SetPose(PoseControl_t *pData)
{
	//std::vector<std::string> elems;
	//auto s = str.find_first_not_of(';', 0);
	//auto e = str.find_first_of(';', s);
	//while (s != std::string::npos || e != std::string::npos)
	//{
	//	elems.emplace_back(str.substr(s, e - s));
	//	s = str.find_first_not_of(';', e);
	//	e = str.find_first_of(';', s);
	//}
	//std::unique_ptr<SimOne_Data_Pose_Control> pPose = std::make_unique<SimOne_Data_Pose_Control>();
	//for (int i = 0; i < elems.size(); i++)
	//{
	//	switch (i)
	//	{
	//	default:
	//	case 0:
	//		pPose->posX = std::atof(elems[i].c_str());
	//		break;
	//	case 1:
	//		pPose->posY = std::atof(elems[i].c_str());
	//		break;
	//	case 2:
	//		pPose->posZ = std::atof(elems[i].c_str());
	//		break;
	//	case 3:
	//		pPose->oriX = std::atof(elems[i].c_str());
	//		break;
	//	case 4:
	//		pPose->oriY = std::atof(elems[i].c_str());
	//		break;
	//	case 5:
	//		pPose->oriZ = std::atof(elems[i].c_str());
	//		break;
	//	case 6:
	//		pPose->autoZ = std::atof(elems[i].c_str());
	//		break;
	//	case 7:
	//		std::atof(elems[i].c_str());
	//		break;
	//	case 8:
	//		std::atoll(elems[i].c_str());
	//		break;
	//	}
	//}

	std::unique_ptr<SimOne_Data_Pose_Control> pPose = std::make_unique<SimOne_Data_Pose_Control>();
	pPose->posX = pData->posX;
	pPose->posY = pData->posY;
	pPose->posZ = pData->posZ;
	pPose->oriX = pData->oriX;
	pPose->oriY = pData->oriY;
	pPose->oriZ = pData->oriZ;

	if (!SimOneAPI::SetPose(mainVehicleId.c_str(), pPose.get()))
	{
		std::cout << "Set Pose Failed!" << std::endl;
	}
}

void simapi::SimAPI_SetTrajectory()
{
	while (true)
	{
		std::unique_ptr<SimOne_Data_Gps> pGps = std::make_unique<SimOne_Data_Gps>();
		std::unique_ptr<SimOne_Data_Trajectory> pTrajectory = std::make_unique<SimOne_Data_Trajectory>();

		if (!SimOneAPI::GetGps(mainVehicleId.c_str(), pGps.get()))
		{
			std::cout << "Get GPS Fail" << std::endl;
			continue;
		}
		pTrajectory->trajectorySize = 20;
		pTrajectory->trajectory[0].posX = pGps->posX;
		pTrajectory->trajectory[0].posY = pGps->posY;
		pTrajectory->trajectory[0].vel = pGps->velX;
		for (int i=1; i<pTrajectory->trajectorySize; i++)
		{
			pTrajectory->trajectory[i].posX = pGps->posX+i;
			pTrajectory->trajectory[i].posY = pGps->posY+i;
			pTrajectory->trajectory[i].vel = pGps->velX;
		}

		if (!SimOneAPI::SetTrajectory(mainVehicleId.c_str(), pTrajectory.get()))
		{
			std::cout << "SetTrajectory Failed!" << std::endl;
		}
	}
}

void simapi::SimAPI_SetVehicleEvent()
{
	SimOne_Data_Vehicle_EventInfo event; 
	event.type =  ESimone_Vehicle_EventInfo_Type::ESimOne_VehicleEventInfo_V2XWarningInfoType_FCW; // ESimone_Vehicle_EventInfo_Type_Forward_Collision_Warning; // front_crash_warning
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Backward_Collision_Warning; // back_crash_warning
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Left_Turn_Decision; // turn_left
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Left_Turn_Warning; // left_warning
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Right_Turn_Decision; // turn_right
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Right_Turn_Warning; // right_warning
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Forward_Straight_Decision; // straight_through
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Forward_Straight_Warning; // straight_warning
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Over_Speed_Warning; // overspeeding_warning
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Lane_Change_Decision; // lane_change
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Lane_Change_Warning; // lane_change_warning
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Overtake_Decision; // overtake
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Emergency_Braking_Decision; // emergency_braking
	// event.type = ESimone_Vehicle_EventInfo_Type::ESimone_Vehicle_EventInfo_Type_Accelerate_Decision; // accelerate

	if (! SimOneAPI::SetVehicleEvent(mainVehicleId.c_str(), &event))
	{
		std::cout << "SetVehicleEvent Failed!" << std::endl;
		return;
	}
	std::cout << "SetVehicleEvent Done!" << std::endl;
}

void simapi::SimAPI_SetSignalLights(SignalLight_t &light)
{

	SimOne_Data_Signal_Lights signalLights;
	signalLights.timestamp = light.timestamp;
	signalLights.signalLights = (light.HighBeam == 1 ? ESimOne_Signal_Light_HighBeam : 0) |
								(light.LeftBlinker == 1 ? ESimOne_Signal_Light_LeftBlinker : 0) |
								(light.RightBlinker == 1 ? ESimOne_Signal_Light_RightBlinker : 0);

	// std::cout << "------------ HighBeam: " << light.HighBeam << std::endl;
	// std::cout << "------------ LeftBlinker: " << light.LeftBlinker  << std::endl;
	// std::cout << "------------ RightBlinker: " << light.RightBlinker << std::endl;

	// signalLights.signalLights = ESimOne_Signal_Light_None;
	// signalLights.signalLights = ESimOne_Signal_Light_RightBlinker;
	// signalLights.signalLights = ESimOne_Signal_Light_LeftBlinker;
	// signalLights.signalLights = ESimOne_Signal_Light_DoubleFlash;
	// signalLights.signalLights = ESimOne_Signal_Light_BrakeLight;
	// signalLights.signalLights = ESimOne_Signal_Light_FrontLight;
	// signalLights.signalLights = ESimOne_Signal_Light_HighBeam;
	// signalLights.signalLights = ESimOne_Signal_Light_BackDrive;

	lights[0] = light.LeftBlinker == 1 ? true: false;
	lights[1] = light.RightBlinker == 1 ? true: false;

	// std::cout << "----------------------------------lights[0]: " << lights[0] << std::endl;
	// std::cout << "----------------------------------lights[1]: " << lights[1] << std::endl;

	if(!SimOneAPI::SetSignalLights(mainVehicleId.c_str(), &signalLights))
	{
		std::cout << "SetSignalLights Failed!" << std::endl;
		return;
	}
	// std::cout << "SetSignalLights Done!" << std::endl;
}

void simapi::SimAPI_SetDriveMode()
{
	ESimOne_Drive_Mode driveMode = ESimOne_Drive_Mode_API; // ESimOne_Drive_Mode_Driver

	if (!SimOneAPI::SetDriveMode(mainVehicleId.c_str(), driveMode))
	{
		std::cout << "SetDriveMode Failed!" << std::endl;
	}
	std::cout << "SetDriveMode Done!" << std::endl;
}

bool simapi::SimAPI_GetControlMode()
{
	SimOne_Data_Control_Mode controlMode;
	if(!SimOneAPI::GetControlMode(mainVehicleId.c_str(), &controlMode))
	{
		std::cout << "GetControlMode Failed!" << std::endl;
		return false;
	}

	switch(controlMode.controlMode)
	{
		case ESimOne_Control_Mode_API:
		case ESimOne_Control_Mode_Unknown:
			// std::cout << "ESimOne_Control_Mode_API" << std::endl;
			return true;
			// std::cout << "ESimOne_Control_Mode_Unknown" << std::endl;
		case ESimOne_Control_Mode_Manual:
			// std::cout << "ESimOne_Control_Mode_Manual" << std::endl;
		case ESimOne_Control_Mode_Auto:
			// std::cout << "ESimOne_Control_Mode_Auto" << std::endl;
			// break;
		default:
			// std::cout << "Invalid ControlMode!" << std::endl;
			return false;
	}

	std::cout << "GetControlMode Done!" << std::endl;
}

void simapi::SimAPI_GetWayPoints()
{
	std::unique_ptr<SimOne_Data_WayPoints> pWayPoints = std::make_unique<SimOne_Data_WayPoints>();
	// int lastFrame = 0;

	while (1)
	{
		bool flag = SimOneAPI::GetWayPoints(mainVehicleId.c_str(), pWayPoints.get());
		// if (flag && pWayPoints->frame != lastFrame)
		if (flag)
		{
			// lastFrame = pWayPoints->frame;
			dbg_data.dump_waypoints(mainVehicleId.c_str(), pWayPoints.get());
		}
		if (!flag)
		{
			std::cout << "Get WayPoints Fail" << std::endl;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

bool simapi::SimAPI_SetScenarioEventCB(void(*cb)(const char* source, const char* target, const char* type, const char* content))
{
	if (!SimOneAPI::SetScenarioEventCB(cb))
	{
		std::cout << "SetScenarioEventCB Failed!" << std::endl;
		return false;
	}
	return true;
}

bool simapi::SimAPI_SetJudgeEventCB(void(*cb)(const char* mainVehicleId, SimOne_Data_JudgeEvent *judgeEventDetailInfo))
{
	if (!SimOneAPI::SetJudgeEventCB(cb))
	{
		std::cout << "SetJudgeEventCB Failed!" << std::endl;
		return false;
	}
	return true;
}

bool simapi::SimAPI_GetLaneName(SSD::SimPoint3D pos, SSD::SimString &mvLaneName)
{
        double s, t, s_toCenterLine, t_toCenterLine;
        if (!SimOneAPI::GetNearMostLane(pos, mvLaneName, s, t, s_toCenterLine, t_toCenterLine))
        {
                std::cout << "GetNearMostLane Failed!" << std::endl;
                return false;
        }
        return true;
}

bool simapi::SimAPI_GetObjVehicleDist(SSD::SimPoint3D mvPos, SSD::SimPoint3D objPos, SSD::SimString &mvLaneName, double &dist)
{
        HDMapStandalone::MSideState sideState;
        if (!SimOneAPI::IsInsideLane(objPos, mvLaneName, sideState))
        {
                return false;
        }
        dist = std::sqrt(std::pow(objPos.x - mvPos.x, 2) + std::pow(objPos.y - mvPos.y, 2) + std::pow(objPos.z - mvPos.z, 2));  

        return true;
}
// void simapi::SimAPI_GetStreamingImage(const  char* ip, unsigned short port, bool IsCallBackMode)
// {
// 	if (IsCallBackMode)
// 	{
// 		auto function = [](SimOne_Streaming_Image* pImage){
// 				dbg_data.dump_streaming_image(pImage);
// 		};
// 		 SimOneAPI::SetStreamingImageUpdateCB(ip, port, function);
// 	}
// 	else {
// 		std::unique_ptr<SimOne_Streaming_Image> pImage = std::make_unique<SimOne_Streaming_Image>();
// 		int lastFrame = 0;
// 		while(1)
// 		{
// 			bool flag = SimOneAPI::GetStreamingImage(ip, port, pImage.get());
// 			if (flag && pImage->frame != lastFrame)
// 			{
// 				lastFrame  = pImage->frame;
// 				dbg_data.dump_streaming_image(pImage.get());
// 			}
// 			if (!flag)
// 			{
// 				std::cout<<"Get Streaming Image Fail"<< std::endl;
// 			}
// 			std::this_thread::sleep_for(std::chrono::milliseconds(10));
// 		}
// 	}
// }
// void simapi::SimAPI_GetStreamingImage_H265(const  char* ip, unsigned short port, bool IsCallBackMode, const char* prefix)
// {
// 	prefix_h265 = const_cast<char *>(prefix);
// 
// 	if (IsCallBackMode)
// 	{
// 		auto function = [](SimOne_Streaming_Image* pImage){
// 				dbg_data.dump_streaming_image(pImage, prefix_h265);
// 		};
// 		 SimOneAPI::SetStreamingImageUpdateCB(ip, port, function);
// 	}
// 	else {
// 		std::unique_ptr<SimOne_Streaming_Image> pImage = std::make_unique<SimOne_Streaming_Image>();
// 		int lastFrame = 0;
// 		while(1)
// 		{
// 			bool flag = SimOneAPI::GetStreamingImage(ip, port, pImage.get());
// 			if (flag && pImage->frame != lastFrame)
// 			{
// 				lastFrame  = pImage->frame;
// 				dbg_data.dump_streaming_image(pImage.get(), prefix_h265);
// 			}
// 			if (!flag)
// 			{
// 				std::cout<<"Get Streaming Image Fail"<< std::endl;
// 			}
// 			std::this_thread::sleep_for(std::chrono::milliseconds(10));
// 		}
// 	}
// }
// void simapi::SimAPI_GetStreamingImage_RLE(const  char* ip, unsigned short port, bool IsCallBackMode, const char* prefix)
// {
// 	prefix_rle = const_cast<char *>(prefix);
// 
// 	if (IsCallBackMode)
// 	{
// 		auto function = [](SimOne_Streaming_Image* pImage){
// 				dbg_data.dump_streaming_image(pImage, prefix_rle);
// 		};
// 		 SimOneAPI::SetStreamingImageUpdateCB(ip, port, function);
// 	}
// 	else {
// 		std::unique_ptr<SimOne_Streaming_Image> pImage = std::make_unique<SimOne_Streaming_Image>();
// 		int lastFrame = 0;
// 		while(1)
// 		{
// 			bool flag = SimOneAPI::GetStreamingImage(ip, port, pImage.get());
// 			if (flag && pImage->frame != lastFrame)
// 			{
// 				lastFrame  = pImage->frame;
// 				dbg_data.dump_streaming_image(pImage.get(), prefix_rle);
// 			}
// 			if (!flag)
// 			{
// 				std::cout<<"Get Streaming Image Fail"<< std::endl;
// 			}
// 			std::this_thread::sleep_for(std::chrono::milliseconds(10));
// 		}
// 	}
// }
// 
// void simapi::SimAPI_GetStreamingPointCloud(const  char* ip, unsigned short port, unsigned short infoPort, bool IsCallBackMode, const char* prefix)
// {
// 	if (IsCallBackMode) {
// 		auto function = [](SimOne_Streaming_Point_Cloud *pPointCloud)
// 		{
// 			dbg_data.dump_point_cloud(pPointCloud);
// 		};
// 		SimOneAPI::SetStreamingPointCloudUpdateCB(ip, port, infoPort, function);
// 	}
// 	else {
// 		std::unique_ptr<SimOne_Streaming_Point_Cloud> pPointCloud = std::make_unique<SimOne_Streaming_Point_Cloud>();
// 		int lastFrame = 0;
// 		while(1)
// 		{
// 			bool flag = SimOneAPI::GetStreamingPointCloud(ip, port, infoPort, pPointCloud.get());
// 			if (flag && pPointCloud->frame != lastFrame)
// 			{
// 				lastFrame  = pPointCloud->frame;
// 				dbg_data.dump_point_cloud(pPointCloud.get());
// 			}
// 			if (!flag)
// 			{
// 				std::cout<<"Get Streaming Image Fail"<< std::endl;
// 			}
// 			std::this_thread::sleep_for(std::chrono::milliseconds(10));
// 		}
// 	}
// }
// 
