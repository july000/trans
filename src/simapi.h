#ifndef SIMAPI_H
#define SIMAPI_H

#include "SimOneSensorAPI.h"
#include "SimOneV2XAPI.h"
#include "SimOnePNCAPI.h"
#include "SimOneServiceAPI.h"
#include "SimOneHDMapAPI.h"
#include "SimOneEvaluationAPI.h"
// #include "SimOneStreamingAPI.h"
#include "dumper.h"
// #include "infoReq_51.h"
#include "zd/MsgStruct.h"

#include <thread>
#include <chrono>
#include <iostream>
#include <string.h>
#include <queue>
#include <stack>
// #include <windows.h>
// using namespace std;

#define INDEX_SIZE 110

class simapi
{
	public:
		typedef struct PoseControl
		{
			float posX; // Position X on Opendrive (by meter)
			float posY; // Position Y on Opendrive (by meter)
			float posZ; // Position Z on Opendrive (by meter)
			float oriX; // Rotation X on Opendrive (by radian)
			float oriY; // Rotation Y on Opendrive (by radian)
			float oriZ; // Rotation Z on Opendrive (by radian)
		} PoseControl_t;

		typedef struct MainTrafficLight
		{
			long id;
			SSD::SimPoint3D pt;
		}MainTrafficLight_t;

		enum Control_Mode
		{
			Control_Mode_Percent = 0,
			Control_Mode_Torque = 1,
			Control_Mode_Speed = 2,
			Control_Mode_Accel = 3,
			Control_Mode_EngineAV = 4,
			Control_Mode_WheelTorque = 5
		};

		simapi(const char* mv_id);
		~simapi();

		// Service API
		void SimAPI_GetVersion();
		void SimAPI_GetHdMapData();
		// ------ void SimAPI_SendRouteMessage(int length, void* pBuffer, int msgId, int toNodeId, ESimOne_Client_Type toNodeType);
		// ------ void SimAPI_ReceiveRouteMessageCB(void(*cb)(int fromId, ESimOne_Client_Type fromType, int length, const void* pBuffer, int commandId));
		// ------ void SimAPI_SetLogOut(ESimOne_LogLevel_Type level, const char *format, ...);
		void SimAPI_InitSimOneAPI(bool isJoinTimeLoop, const char *serverIP);
		bool SimAPI_IsCaseStop();
		void SimAPI_TerminateSimOneAPI();

		void SimAPI_GetCaseInfo();
		void SimAPI_GetCaseRunStatus();
		void SimAPI_GetMainVehicleList();
		// ------ int Wait();
		// ------ void NextFrame(int frame);
		// ------ bool SetFrameCB(void(*FrameStart)(int frame), void(*FrameEnd)(int frame));
		void SimAPI_GetMainVehicleStatus(bool IsCallBackMode);
		void SimAPI_GetHDMapData();

		long GetSuccessorTillJunction(const SSD::SimString& laneName, std::vector<string>& routeToJunctionLanes, int levelsOfRecursion = 10);
		void GetPredecessorTillJunction(const SSD::SimString& laneName, std::vector<string>& routeToJunctionLanes, int levelsOfRecursion = 10);
		bool SimAPI_GetRestLength(double& restLength, std::vector<long>& roadIdList, std::map<std::string, bool>& route_section_ori);
		bool SimAPI_GetRouteInfo(double& routeLength, int& routeCountDown, std::vector<long>& roadIdList, std::map<std::string, bool>& route_section_ori);

		// HDMap API
		bool SimAPI_HDMap_ALL(const std::vector<std::string> &apiNames);
		// LoadHDMap: Test in SimAPI_HDMap_ALL
		// GetNearMostLane: Test in SimAPI_HDMap_ALL
		void SimAPI_GetNearLanes();
    	bool SimAPI_GetLaneName(SSD::SimPoint3D pos, SSD::SimString &mvLaneName);
    	bool SimAPI_GetObjVehicleDist(SSD::SimPoint3D mvPos, SSD::SimPoint3D objPos, SSD::SimString &mvLaneName, double &dist);
		void SimAPI_GetTrafficLightList(SSD::SimString &laneName, std::vector<MainTrafficLight_t> &ids);
		bool SimAPI_IsInJunction(SSD::SimString &mvLaneName);

		// ------ bool GetNearLanesWithAngle(const SSD::SimPoint3D& pos, const double& distance, const double& headingAngle, const double& angleShift, SSD::SimStringVector& nearLanes);
		// ------ bool GetDistanceToLaneBoundary(const SSD::SimPoint3D& pos, SSD::SimString& id, double& distToLeft, double& distToRight, double& distToLeft2D, double& distToRight2D);
		// ------ bool GetLaneSample(const SSD::SimString &id, HDMapStandalone::MLaneInfo& info);
		// GetLaneLink: Test in SimAPI_HDMap_ALL
		// ------ bool GetLaneType(const SSD::SimString& id, HDMapStandalone::MLaneType& laneType);
		// ------ bool GetLaneWidth(const SSD::SimString& id, const SSD::SimPoint3D& pos, double& width);
		// ------ bool GetLaneST(const SSD::SimString& id, const SSD::SimPoint3D& pos, double& s, double& t);
		// ------ bool GetRoadST(const SSD::SimString& id, const SSD::SimPoint3D& pos, double& s, double& t, double& z);
		// ------ bool GetInertialFromLaneST(const SSD::SimString& id, const double& s, const double& t, SSD::SimPoint3D& inertial, SSD::SimPoint3D& dir);
		// ------ bool ContainsLane(const SSD::SimString& id);
		// ------ void GetParkingSpaceList(SSD::SimVector<HDMapStandalone::MParkingSpace>& parkingSpaceList);
		// ------ bool GenerateRoute(const SSD::SimPoint3DVector& inputPoints, SSD::SimVector<int>& indexOfValidPoints, SSD::SimPoint3DVector& route);
		// ------ bool Navigate(const SSD::SimPoint3DVector& inputPoints, SSD::SimVector<int>& indexOfValidPoints, SSD::SimVector<long>& roadIdList);
		// ------ bool GetRoadMark(const SSD::SimPoint3D& pos, const SSD::SimString& id, HDMapStandalone::MRoadMark& left, HDMapStandalone::MRoadMark& right);
		// GetTrafficLightList: Test in SimAPI_HDMap_ALL
		// ------ void GetTrafficSignList(SSD::SimVector<HDMapStandalone::MSignal>& list);
		// ------ void GetStoplineList(const HDMapStandalone::MSignal& light, const SSD::SimString& id, SSD::SimVector<HDMapStandalone::MObject>& stoplineList);
		// ------ void GetCrosswalkList(const HDMapStandalone::MSignal& light, const SSD::SimString& id, SSD::SimVector<HDMapStandalone::MObject>& crosswalkList);
		// GetCrossHatchList: Test in SimAPI_HDMap_ALL
		// ------ bool GetLaneMiddlePoint(const SSD::SimPoint3D& inputPt, const SSD::SimString& id, SSD::SimPoint3D& targetPoint, SSD::SimPoint3D& dir);
		// ------ bool GetHeights(const SSD::SimPoint3D& inputPt, const double& radius, SSD::SimVector<double>& heights, SSD::SimVector<long>& roadIds, SSD::SimVector<bool>& insideRoadStates);
		// ------ void GetLaneData(SSD::SimVector<HDMapStandalone::MLaneInfo>& data);
		// ------ SSD::SimVector<long> GetJunctionList();
		// ------ double GetRoadLength(const long& roadId);
		// ------ bool GetSectionLaneList(const SSD::SimString& laneId, SSD::SimStringVector& sectionLaneList);
		// ------ bool IsTwoSideRoad(const long& roadId);
		// ------ double GetLaneLength(const SSD::SimString& id);
		// ------ bool IsDriving(const SSD::SimString& id);
		// ------ bool IsInJunction(const SSD::SimString& id, long& juncId);
		// ------ bool IsInsideLane(const SSD::SimPoint3D& inputPt, const SSD::SimString& laneName, HDMapStandalone::MSideState& sideState);
		// ------ bool GetLaneSampleByLocation(const SSD::SimPoint3D& pos, HDMapStandalone::MLaneInfo& info);

		// PNC API
		void SimAPI_RegisterVehicleState();
		bool SimAPI_GetVehicleState(SimOne_Data_Vehicle_Extra* pVehExtraState);
		// void SimAPI_SetPose(std::string& str);
		void SimAPI_SetPose(PoseControl_t *pPose);
		void SimAPI_SetDrive(Control_t &ctrl, Control_Mode ctrMode);
		// ------ bool SetDriveTrajectory(const char* mainVehicleId, SimOne_Data_Control_Trajectory *pControlTrajectory);
		// ------ void SetDriverName(const char* mainVehicleId, const char* name);
		void SimAPI_SetTrajectory();
		void SimAPI_SetVehicleEvent();
		void SimAPI_SetSignalLights(SignalLight_t &light);
		void SimAPI_SetDriveMode();

		bool SimAPI_GetControlMode();
		// ------ bool GetDriverControl(const char* mainVehicleId, SimOne_Data_Control* pControl);
		// ------ bool GetWayPoints(const char* mainVehicleId, SimOne_Data_WayPoints* pWayPoints);
		void SimAPI_GetWayPoints();
		bool SimAPI_SetScenarioEventCB(void(*cb)(const char* source, const char* target, const char* type, const char* content));
		// ------ bool SetTrafficEventCB(void(*cb)(const char* mainVehicleId, const char* data));

		// Sensor API
		// SimOneSensor
		void SimAPI_GetEnvironment();
		void SimAPI_GetMessage();
		bool SimAPI_GPS(SimOne_Data_Gps *pGps);
		// void Get_GPS(Veh_Data_Para &vp);
		bool SimAPI_SetGpsUpdateCB(void(*cb)(const char* mainVehicleId, SimOne_Data_Gps *pGps));
		void SimAPI_GetGroundTruth(bool IsCallBackMode);
		void SimAPI_RadarDetection(bool IsCallBackMode);
		void SimAPI_UltrasonicRadar();
		void SimAPI_UltrasonicRadars(bool IsCallBackMode);
		bool SimAPI_GetSensorDetections(SimOne_Data_SensorDetections *pDetections);
		bool SimAPI_SetSensorDetectionsUpdateCB(void (*cb)(const char *mainVehicleId, const char *sensorId, SimOne_Data_SensorDetections *pGroundtruth));
		void SimAPI_GetSensorRoadMark(bool IsCallBackMode);
		void SimAPI_GetSensorConfigurations();
		void SimAPI_SetEnvironment();
		// GetTrafficLight: Test in SimAPI_HDMap_ALL
		bool SimAPI_GetTrafficLight(int lightId, SimOne_Data_TrafficLight *pTrafficLight);
		bool SimAPI_SensorLaneInfo(SimOne_Data_LaneInfo *pLaneInfo);

		// Sensor-V2X API
		void SimAPI_V2XInfo(bool IsCallBackMode);

		// Streaming API
		// void SimAPI_GetStreamingImage(const char *ip, unsigned short port, bool IsCallBackMode);
		// void SimAPI_GetStreamingImage_H265(const char *ip, unsigned short port, bool IsCallBackMode, const char* prefix = "");
		// void SimAPI_GetStreamingImage_RLE(const char *ip, unsigned short port, bool IsCallBackMode, const char* prefix = "");
		// void SimAPI_GetStreamingPointCloud(const  char* ip, unsigned short port, unsigned short infoPort, bool IsCallBackMode, const char* prefix = "");

		// Evaluation API
		bool SimAPI_SetJudgeEventCB(void(*cb)(const char* mainVehicleId, SimOne_Data_JudgeEvent *judgeEventDetailInfo));

		static dumper dbg_data;
		static char* prefix_h265;
		static char* prefix_rle;
		static bool lights[2];

		std::string mainVehicleId;
	// private:
};

#endif
