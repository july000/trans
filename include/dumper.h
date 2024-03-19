#ifndef DUMPER_H
#define DUMPER_H

#include <iostream>

#ifdef WIN32
#include <Windows.h>
// struct timeval
// {
// 	__int64 tv_sec;
// 	__int64 tv_usec;
// };
// void gettimeofday(struct timeval* tv, void *tzp);
#else
#include <sys/time.h>
#endif

#include "Service/SimOneIOStruct.h"
// #include "GNSS.h"
#include "SimOneStreamingAPI.h"

using namespace std;

class dumper
{
public:
	dumper();
	~dumper();

	std::string GetNowTime();
	int64_t getCurrentTime();

	void dump_gps(const char* mainVehicleId, SimOne_Data_Gps* pData);
	void dump_ground_truth(const char* mainVehicleId, SimOne_Data_Obstacle* pData);
	void dump_radar_detection(const char* mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection * pData);
	void dump_ultrasonic_radar(const char* mainVehicleId, const char* sensorId, SimOne_Data_UltrasonicRadar* pData);
	void dump_ultrasonic_radars(const char* mainVehicleId, SimOne_Data_UltrasonicRadars* pData);
	void dump_sensor_detections(const char* mainVehicleId, const char* sensorId, SimOne_Data_SensorDetections* pData);
	void dump_sensor_configurations(const char* mainVehicleId, SimOne_Data_SensorConfigurations* pData);
	void dump_environment(SimOne_Data_Environment* pData);
	void dump_traffic_light(const char* mainVehicleId, int opendriveLightId, SimOne_Data_TrafficLight* pData);
	void dump_sensor_laneInfo(const char* mainVehicleId, const char* sensorId, SimOne_Data_LaneInfo* pData);
	void dump_waypoints(const char* mainVehicleId, SimOne_Data_WayPoints* pData);
	void dump_streaming_image(SimOne_Streaming_Image* pData, const char* prefix="");
	void dump_point_cloud(SimOne_Streaming_Point_Cloud* pData, const char* prefix="");
};

#endif