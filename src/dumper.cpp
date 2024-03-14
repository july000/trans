#include "dumper.h"
#include <fstream>
#include <string>
#include "math.h"
using namespace std;

dumper::dumper(){
	cout.setf(ios::fixed);
}
dumper::~dumper(){}

#ifdef WIN32
int64_t CurrentTime()
{
	struct timeval tv;
	time_t clock;
	struct tm tm;
	SYSTEMTIME wtm;

	GetLocalTime(&wtm);
	tm.tm_year = wtm.wYear - 1900;
	tm.tm_mon = wtm.wMonth - 1;
	tm.tm_mday = wtm.wDay;
	tm.tm_hour = wtm.wHour;
	tm.tm_min = wtm.wMinute;
	tm.tm_sec = wtm.wSecond;
	tm.tm_isdst = -1;
	clock = mktime(&tm);
	tv.tv_sec = clock;
	tv.tv_usec = wtm.wMilliseconds * 1000;

	// gettimeofday(&tv, NULL);
	return tv.tv_sec * 1000 + tv.tv_usec / 1000; // milliseconds
	// return tv.tv_sec * 1e6 + tv.tv_usec; // microseconds
}

std::string GetNowTime()
{
	time_t setTime;
	time(&setTime);
	tm *ptm = localtime(&setTime);
	std::string time = std::to_string(ptm->tm_year + 1900) + "_" + std::to_string(ptm->tm_mon + 1) + "_" + std::to_string(ptm->tm_mday) + "-" + std::to_string(ptm->tm_hour) + "_" + std::to_string(ptm->tm_min) + "_" + std::to_string(ptm->tm_sec);
	return time;
}

#else
int64_t CurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000; // milliseconds
    // return tv.tv_sec * 1e6 + tv.tv_usec; // microseconds
}

std::string GetNowTime()
{
    time_t setTime;
    time(&setTime);
    tm *ptm = localtime(&setTime);
    std::string time = std::to_string(ptm->tm_year + 1900) + "/" + std::to_string(ptm->tm_mon + 1) + "/" + std::to_string(ptm->tm_mday) + " " + std::to_string(ptm->tm_hour) + ":" + std::to_string(ptm->tm_min) + ":" + std::to_string(ptm->tm_sec);
    return time;
}
#endif

std::ofstream outFile;
void dumper::dump_gps(const char* mainVehicleId, SimOne_Data_Gps* pData)
{

	static bool csv_gen = true;
	static double pre_distX = 0.0;
	static double pre_distY = 0.0;
	static double total_dist = 0.0;
	static double total_dur = 0.0;
	if (csv_gen)
	{
		csv_gen = false;
		outFile.open("debug_delay.csv", std::ios::out | std::ios::trunc);
		outFile << "Frame" << ',' << "Timestamp_GPS" << ',' << "Timestamp_SysLocal" << ',' << "Interval" << ','
		<< "Velocity" << ',' << "Distance" << ',' << "Total_Dist" << ',' << "Totoal_Duration" << std::endl;
	}

	cout.setf(ios::fixed);
	static uint64_t preTime=0.0;
	uint64_t curTime = CurrentTime();
	// std::cout <<"------ mainVehicleId:"<< mainVehicleId << std::endl;
	// std::cout << "------ frame:"<< pData->frame << std::endl;	
	// std::cout << "------ gps timestamp:"<< pData->timestamp << std::endl;	
	// std::cout << "------ loc timestamp:"<<  curTime << std::endl;	
	// std::cout << "------ interval: " << curTime - preTime << std::endl;

	float vel = sqrt(pData->velX*pData->velX + pData->velY*pData->velY + pData->velZ*pData->velZ) * 3.6;
	double dist = 0.0;
	if (pre_distX != 0.0)
	{
		dist = sqrt(pow((pData->posX - pre_distX), 2) + pow((pData->posY - pre_distY), 2));
		total_dist += dist;
		total_dur += (curTime - preTime);
	}
	pre_distX = pData->posX;
	pre_distY = pData->posY;

	outFile << pData->frame << ',' << pData->timestamp << ',' << curTime  << ',' << curTime - preTime 
	<< ',' << vel << ',' << dist << ',' << total_dist << ',' << total_dur << std::endl;


	preTime = curTime;
	// std::cout << "posX/Y/Z: [" << pData->posX << ", " << pData->posY << ", " << pData->posZ << "]" << std::endl;
	// std::cout << "oriX/Y/Z: [" << pData->oriX << ", " << pData->oriY << ", " << pData->oriZ << "]" << std::endl;
	// std::cout << "velX/Y/Z: [" << pData->velX << ", " << pData->velY << ", " << pData->velZ << "]" << std::endl;
	// std::cout << "throttle: " << pData->throttle << std::endl;
	// std::cout << "brake: " << pData->brake << std::endl;
	// std::cout << "steering: " << pData->steering << std::endl;
	// std::cout << "gear: " << pData->gear << std::endl;
	// std::cout << "accelX/Y/Z: " << pData->accelX << ", "  << pData->accelY << ", " << pData->accelZ << "]" << std::endl;
	// std::cout << "angVelX/Y/Z: " << pData->angVelX << ", "  << pData->angVelY << ", " << pData->angVelZ << "]" << std::endl;
	// std::cout << "wheelSpeedFL: " << pData->wheelSpeedFL << std::endl;
	// std::cout << "wheelSpeedFR: " << pData->wheelSpeedFR << std::endl;
	// std::cout << "wheelSpeedRL: " << pData->wheelSpeedRL << std::endl;
	// std::cout << "wheelSpeedRR: " << pData->wheelSpeedRR << std::endl;
	// std::cout << "engineRpm: " << pData->engineRpm << std::endl;
	// std::cout << "odometer: " << pData->odometer << std::endl;
	// std::cout << "extraStateSize: " << pData->extraStateSize << std::endl;

	// GNSS gnss(12.4, 124.55, 1.0);
	// GNSSMessage g = gnss.getGNSSMessage(pData);
	// std::cout << g.GPGGA_Size << " :: " << g.GNSS_GPGGA;
	// std::cout << g.GPGLL_Size << " :: " << g.GNSS_GPGLL;
	// std::cout << g.GPGSA_Size << " :: " << g.GNSS_GPGSA;
	// std::cout << g.GPRMC_Size << " :: " << g.GNSS_GPRMC;
	// std::cout << g.GPVTG_Size << " :: " << g.GNSS_GPVTG << endl;

	// for (int i=0; i<pData->extraStateSize; i++)
	// {
	// 	std::cout << "extraStates[" << i << "]: " << pData->extraStates[i] << std::endl;
	// }
}

void dumper::dump_ground_truth(const char* mainVehicleId, SimOne_Data_Obstacle* pData)
{
	std::cout <<"------ mainVehicleId:"<< mainVehicleId << std::endl;
	std::cout <<"------ frame:"<< pData->frame << std::endl;
	std::cout << "obstacleSize: " << pData->obstacleSize << std::endl;
	for (int i=0; i<pData->obstacleSize ; i++)
	{
		std::cout << "obstacle[" << i << "].id: " << pData->obstacle[i].id << std::endl;
		std::cout << "obstacle[" << i << "].viewId: " << pData->obstacle[i].viewId << std::endl;
		std::cout << "obstacle[" << i << "].type: " << pData->obstacle[i].type << std::endl;
		std::cout << "obstacle[" << i << "].theta: " << pData->obstacle[i].theta << std::endl;
		std::cout << "obstacle[" << i << "].posX/Y/Z: [" << pData->obstacle[i].posX << ", " << pData->obstacle[i].posY << ", " << pData->obstacle[i].posZ << "]" << std::endl;
		std::cout << "obstacle[" << i << "].oriX/Y/Z: [" << pData->obstacle[i].oriX << ", " << pData->obstacle[i].oriY << ", " << pData->obstacle[i].oriZ << "]" << std::endl;
		std::cout << "obstacle[" << i << "].velX/Y/Z: [" << pData->obstacle[i].velX << ", " << pData->obstacle[i].velY << ", " << pData->obstacle[i].velZ << "]" << std::endl;
		std::cout << "obstacle[" << i << "].length: " << pData->obstacle[i].length << std::endl;
		std::cout << "obstacle[" << i << "].width: " << pData->obstacle[i].width << std::endl;
		std::cout << "obstacle[" << i << "].height: " << pData->obstacle[i].height << std::endl;
		std::cout << "obstacle[" << i << "].accelX/Y/Z: [" << pData->obstacle[i].accelX << ", " << pData->obstacle[i].accelY << ", " << pData->obstacle[i].accelZ << "]" << std::endl;
	}
}

void dumper::dump_radar_detection(const char* mainVehicleId, const char* sensorId, SimOne_Data_RadarDetection * pData)
{
	std::cout<<"------ mainVehicleId:"<< mainVehicleId << std::endl;
	std::cout<<"------ sensorId:"<< sensorId << std::endl;
	std::cout<<"------ frame:"<< pData->frame << std::endl;
	std::cout<<"detectNum :"<< pData->detectNum << std::endl;
	for (int i = 0; i < pData->detectNum; i++)
	{
		std::cout << "detections[" << i << "].id: " << pData->detections[i].id << std::endl;
		std::cout << "detections[" << i << "].subId: " << pData->detections[i].subId << std::endl;
		std::cout << "detections[" << i << "].type: " << pData->detections[i].type << std::endl;
		std::cout << "detections[" << i << "].posX/Y/Z: [" << pData->detections[i].posX << ", " << pData->detections[i].posY << ", " << pData->detections[i].posZ << "]"<< std::endl;
		std::cout << "detections[" << i << "].velX/Y/Z: [" << pData->detections[i].velX << ", " << pData->detections[i].velY << ", " << pData->detections[i].velZ << "]"<< std::endl;
		std::cout << "detections[" << i << "].accelX/Y/Z: [" << pData->detections[i].accelX << ", " << pData->detections[i].accelY << ", " << pData->detections[i].accelZ << "]"<< std::endl;
		std::cout << "detections[" << i << "].oriX/Y/Z: [" << pData->detections[i].oriX << ", " << pData->detections[i].oriY << ", " << pData->detections[i].oriZ << "]"<< std::endl;
		std::cout << "detections[" << i << "].length: " << pData->detections[i].length << std::endl;
		std::cout << "detections[" << i << "].width: " << pData->detections[i].width << std::endl;
		std::cout << "detections[" << i << "].height: " << pData->detections[i].height << std::endl;
		std::cout << "detections[" << i << "].range: " << pData->detections[i].range << std::endl;
		std::cout << "detections[" << i << "].rangeRate: " << pData->detections[i].rangeRate << std::endl;
		std::cout << "detections[" << i << "].azimuth: " << pData->detections[i].azimuth << std::endl;
		std::cout << "detections[" << i << "].vertical: " << pData->detections[i].vertical << std::endl;
		std::cout << "detections[" << i << "].snrdb: " << pData->detections[i].snrdb << std::endl;
		std::cout << "detections[" << i << "].rcsdb: " << pData->detections[i].rcsdb << std::endl;
		std::cout << "detections[" << i << "].probability: " << pData->detections[i].probability << std::endl;
	}
}

void dumper::dump_ultrasonic_radar(const char* mainVehicleId, const char* sensorId, SimOne_Data_UltrasonicRadar* pData)
{
	if (mainVehicleId != "")
		std::cout <<"------ mainVehicleId:"<< mainVehicleId << std::endl;
	if (sensorId != "")
		std::cout <<"------ sensorId:"<< sensorId << std::endl;
	std::cout <<"------ frame:"<< pData->frame << std::endl;
	std::cout <<"sensorId:"<< pData->sensorId << std::endl;
	std::cout <<"obstacleNum:"<< pData->obstacleNum << std::endl;
	for (int i = 0; i < pData->obstacleNum; i++)
	{
		std::cout << "obstacleDetections[" << i << "].obstacleRanges: "<< pData->obstacleDetections[i].obstacleRanges << std::endl;
		std::cout << "obstacleDetections[" << i << "].x: "<< pData->obstacleDetections[i].x << std::endl;
		std::cout << "obstacleDetections[" << i << "].y: "<< pData->obstacleDetections[i].y << std::endl;
		std::cout << "obstacleDetections[" << i << "].z: "<< pData->obstacleDetections[i].z << std::endl;
	}
}

void dumper::dump_ultrasonic_radars(const char* mainVehicleId, SimOne_Data_UltrasonicRadars* pData)
{
	std::cout <<"------ mainVehicleId: " << mainVehicleId << std::endl;
	std::cout <<"ultrasonicRadarNum: " << pData->ultrasonicRadarNum << std::endl;
	for (int i = 0; i < pData->ultrasonicRadarNum; i++)
	{
		dump_ultrasonic_radar("", "", &(pData->ultrasonicRadars[i]));
	}
}

void dumper::dump_sensor_detections(const char* mainVehicleId, const char* sensorId, SimOne_Data_SensorDetections* pData)
{
	std::cout <<"------ mainVehicleId:"<< mainVehicleId << std::endl;
	std::cout <<"------ sensorId:"<< sensorId << std::endl;
	std::cout <<"------ frame:"<< pData->frame << std::endl;
	std::cout <<"objectSize: "<< pData->objectSize << std::endl;
	for (int i = 0; i < pData->objectSize; i++)
	{
		std::cout << "objects[" << i  << "].id" << pData->objects[i].id << std::endl;
		switch (pData->objects[i].type)
		{
		case ESimOne_Obstacle_Type_Unknown:
			std::cout << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Unknown" << std::endl;
			break;
		case ESimOne_Obstacle_Type_Pedestrian:
			std::cout << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Pedestrian" << std::endl;
			break;
		case ESimOne_Obstacle_Type_Car:
			std::cout << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Car" << std::endl;
			break;
		case ESimOne_Obstacle_Type_Bicycle:
			std::cout << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Bicycle" << std::endl;
			break;
		case ESimOne_Obstacle_Type_BicycleStatic:
			std::cout << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_BicycleStatic" << std::endl;
			break;
		case ESimOne_Obstacle_Type_Motorcycle:
			std::cout << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Motorcycle" << std::endl;
			break;
		case ESimOne_Obstacle_Type_Truck:
			std::cout << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Truck" << std::endl;
			break;
		case ESimOne_Obstacle_Type_Pole:
			std::cout << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Pole" << std::endl;
			break;
		case ESimOne_Obstacle_Type_Static:
			std::cout << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Static" << std::endl;
			break;
		case ESimOne_Obstacle_Type_Fence:
			std::cout << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Fence" << std::endl;
			break;
		case ESimOne_Obstacle_Type_RoadMark:
			std::cout << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_RoadMark" << std::endl;
			break;
		case ESimOne_Obstacle_Type_TrafficSign:
			std::cout << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_TrafficSign" << std::endl;
			break;
		case ESimOne_Obstacle_Type_TrafficLight:
			std::cout << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_TrafficLight" << std::endl;
			break;
		case ESimOne_Obstacle_Type_Rider:
			std::cout << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Rider" << std::endl;
			break;
		case ESimOne_Obstacle_Type_Bus:
			std::cout << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Bus" << std::endl;
			break;
		case ESimOne_Obstacle_Type_SpecialVehicle:
			std::cout << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_SpecialVehicle" << std::endl;
			break;
		case ESimOne_Obstacle_Type_Dynamic:
			std::cout << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_Dynamic" << std::endl;
			break;
		case ESimOne_Obstacle_Type_GuardRail:
			std::cout << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_GuardRail" << std::endl;
			break;
		case ESimOne_Obstacle_Type_SpeedLimitSign:
			std::cout << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_SpeedLimitSign" << std::endl;
			break;
		case ESimOne_Obstacle_Type_RoadObstacle:
			std::cout << "obstacles[" << i << "].type: ESimOne_Obstacle_Type_RoadObstacle" << std::endl;
			break;
		default:
			std::cout << "obstacles[" << i << "].type: Invalid Object Type" << std::endl;
		}
		std::cout << "objects[" << i  << "].posX/Y/Z: " << "[" << pData->objects[i].posX << ", " << pData->objects[i].posY << ", " << pData->objects[i].posZ << "]" << std::endl;
		std::cout << "objects[" << i  << "].oriX/Y/Z: " << "[" << pData->objects[i].oriX << ", " << pData->objects[i].oriY << ", " << pData->objects[i].oriZ << "]" << std::endl;
		std::cout << "objects[" << i  << "].length: " << pData->objects[i].length << std::endl;
		std::cout << "objects[" << i  << "].width: " << pData->objects[i].width << std::endl;
		std::cout << "objects[" << i  << "].height: " << pData->objects[i].height << std::endl;
		std::cout << "objects[" << i  << "].range: " << pData->objects[i].range << std::endl;
		std::cout << "objects[" << i  << "].velX/Y/Z: " << "[" << pData->objects[i].velX << ", " << pData->objects[i].velY << ", " << pData->objects[i].velZ << "]" << std::endl;
		std::cout << "objects[" << i  << "].accelX/Y/Z: " << "[" << pData->objects[i].accelX << ", " << pData->objects[i].accelY << ", " << pData->objects[i].accelZ << "]" << std::endl;
		std::cout << "objects[" << i  << "].probability: " << pData->objects[i].probability << std::endl;
		std::cout << "objects[" << i  << "].relativePosX/Y/Z: " << "[" << pData->objects[i].relativePosX << ", " << pData->objects[i].relativePosY << ", " << pData->objects[i].relativePosZ << "]" << std::endl;
		std::cout << "objects[" << i  << "].relativeRotX/Y/Z: " << "[" << pData->objects[i].relativeRotX << ", " << pData->objects[i].relativeRotY << ", " << pData->objects[i].relativeRotZ << "]" << std::endl;
		std::cout << "objects[" << i  << "].relativeVelX/Y/Z: " << "[" << pData->objects[i].relativeVelX << ", " << pData->objects[i].relativeVelY << ", " << pData->objects[i].relativeVelZ << "]" << std::endl;
		std::cout << "objects[" << i  << "].bbox2dMinX: " << pData->objects[i].bbox2dMinX  << std::endl;
		std::cout << "objects[" << i  << "].bbox2dMinY: " << pData->objects[i].bbox2dMinY  << std::endl;
		std::cout << "objects[" << i  << "].bbox2dMaxX: " << pData->objects[i].bbox2dMaxX  << std::endl;
		std::cout << "objects[" << i  << "].bbox2dMaxY: " << pData->objects[i].bbox2dMaxY  << std::endl;
		std::cout << "objects[" << i  << "].name: " << pData->objects[i].name << std::endl;
	}
}

void dumper::dump_sensor_configurations(const char* mainVehicleId, SimOne_Data_SensorConfigurations* pData)
{
	std::cout <<"------ mainVehicleId:"<< mainVehicleId << std::endl;
	std::cout << "dataSize: " << pData->dataSize << std::endl;
	for (int i=0; i< pData->dataSize; i++)
	{
		std::cout << "data[" << i << "].id: "  << pData->data[i].id << std::endl;
		std::cout << "data[" << i << "].mainVehicleId: "  << pData->data[i].mainVehicleId << std::endl;
		std::cout << "data[" << i << "].sensorId: "  << pData->data[i].sensorId << std::endl;
		std::cout << "data[" << i << "].sensorType: "  << pData->data[i].sensorType << std::endl;
		std::cout << "data[" << i << "].x/y/z: ["  << pData->data[i].x << ", " << pData->data[i].y << ", "<< pData->data[i].z << "]" << std::endl;
		std::cout << "data[" << i << "].roll: "  << pData->data[i].roll << std::endl;
		std::cout << "data[" << i << "].pitch: "  << pData->data[i].pitch << std::endl;
		std::cout << "data[" << i << "].yaw: "  << pData->data[i].yaw << std::endl;
		std::cout << "data[" << i << "].hz: "  << pData->data[i].hz << std::endl;
	}
}

void dumper::dump_environment(SimOne_Data_Environment* pData)
{
	std::cout <<  "timeOfDay: " << pData->timeOfDay << std::endl; // GetEnvironment timeOfDay ([0, 2400])
	// std::cout <<  "heightAngle: " << pData->heightAngle << std::endl; // GetEnvironment heightAngle ([0, 90])
	// std::cout <<  "directionalLight: " << pData->directionalLight << std::endl; // GetEnvironment directionalLight ([0, 1])
	// std::cout <<  "ambientLight: " << pData->ambientLight << std::endl; // GetEnvironment ambientLight ([0, 1])
	// std::cout <<  "artificialLight: " << pData->artificialLight << std::endl; // GetEnvironment artificialLight ([0, 1])
	// std::cout <<  "cloudDensity: " << pData->cloudDensity << std::endl; // GetEnvironment cloudDensity ([0, 1])
	// std::cout <<  "fogDensity: " << pData->fogDensity << std::endl; // GetEnvironment fogDensity ([0, 1])
	// std::cout <<  "rainDensity: " << pData->rainDensity << std::endl; // GetEnvironment rainDensity ([0, 1])
	// std::cout <<  "snowDensity: " << pData->snowDensity << std::endl; // GetEnvironment snowDensity ([0, 1])
	// std::cout <<  "groundHumidityLevel: " << pData->groundHumidityLevel << std::endl; // GetEnvironment groundHumidityLevel ([0, 1])
	// std::cout <<  "groundDirtyLevel: " << pData->groundDirtyLevel << std::endl; // GetEnvironment groundDirtyLevel ([0, 1])
}

void dumper::dump_traffic_light(const char* mainVehicleId, int opendriveLightId, SimOne_Data_TrafficLight* pData)
{
	std::cout << "------ mainVehicleId:" << mainVehicleId << std::endl;
	std::cout << "isMainVehicleNextTrafficLight: " << pData->isMainVehicleNextTrafficLight << std::endl;
	std::cout << "opendriveLightId: " << pData->opendriveLightId << std::endl;
	std::cout << "countDown: " << pData->countDown << std::endl;
	switch (pData->status)
	{
		case ESimOne_TrafficLight_Status::ESimOne_TrafficLight_Status_Red:
			std::cout << "status: ESimOne_TrafficLight_Status_Red" << std::endl;
			break;
		case ESimOne_TrafficLight_Status::ESimOne_TrafficLight_Status_Green:
			std::cout << "status: ESimOne_TrafficLight_Status_Green" << std::endl;
			break;
		case ESimOne_TrafficLight_Status::ESimOne_TrafficLight_Status_Yellow:
			std::cout << "status: ESimOne_TrafficLight_Status_Yellow" << std::endl;
			break;
		case ESimOne_TrafficLight_Status::ESimOne_TrafficLight_Status_RedBlink:
			std::cout << "status: ESimOne_TrafficLight_Status_RedBlink" << std::endl;
			break;
		case ESimOne_TrafficLight_Status::ESimOne_TrafficLight_Status_GreenBlink:
			std::cout << "status: ESimOne_TrafficLight_Status_GreenBlink" << std::endl;
			break;
		case ESimOne_TrafficLight_Status::ESimOne_TrafficLight_Status_YellowBlink:
			std::cout << "status: ESimOne_TrafficLight_Status_YellowBlink" << std::endl;
			break;
		case ESimOne_TrafficLight_Status::ESimOne_TrafficLight_Status_Black:
			std::cout << "status: ESimOne_TrafficLight_Status_Black" << std::endl;
			break;
		default: 
			std::cout << "status: ESimOne_TrafficLight_Status_Invalid" << std::endl;
			break;
	}
}

void dumper::dump_sensor_laneInfo(const char* mainVehicleId, const char* sensorId, SimOne_Data_LaneInfo* pData)
{
	std::cout <<"------ mainVehicleId:"<< mainVehicleId << std::endl;
	std::cout <<"------ sensorId:"<< sensorId << std::endl;
	std::cout <<"------ frame:"<< pData->frame << std::endl;
	std::cout << "id: " << pData->id  << std::endl;
	switch (pData->laneType)
	{
		case ESimOne_Lane_Type::ESimOne_Lane_Type_driving:
			std::cout << "laneType: ESimOne_Lane_Type_driving"  << std::endl;
			break;
		case ESimOne_Lane_Type::ESimOne_Lane_Type_stop:
			std::cout << "laneType: ESimOne_Lane_Type_stop" << std::endl;
			break;
		case ESimOne_Lane_Type::ESimOne_Lane_Type_shoulder:
			std::cout << "laneType: ESimOne_Lane_Type_shoulder" << std::endl;
			break;
		case ESimOne_Lane_Type::ESimOne_Lane_Type_biking:
			std::cout << "laneType: ESimOne_Lane_Type_biking" << std::endl;
			break;
		case ESimOne_Lane_Type::ESimOne_Lane_Type_sidewalk:
			std::cout << "laneType: ESimOne_Lane_Type_sidewalk" << std::endl;
			break;
		case ESimOne_Lane_Type::ESimOne_Lane_Type_border:
			std::cout << "laneType: ESimOne_Lane_Type_border" << std::endl;
			break;
		case ESimOne_Lane_Type::ESimOne_Lane_Type_restricted:
			std::cout << "laneType: ESimOne_Lane_Type_restricted" << std::endl;
			break;
		case ESimOne_Lane_Type::ESimOne_Lane_Type_parking:
			std::cout << "laneType: ESimOne_Lane_Type_parking" << std::endl;
			break;
		case ESimOne_Lane_Type::ESimOne_Lane_Type_bidirectional:
			std::cout << "laneType: ESimOne_Lane_Type_bidirectional" << std::endl;
			break;
		case ESimOne_Lane_Type::ESimOne_Lane_Type_median:
			std::cout << "laneType: ESimOne_Lane_Type_median" << std::endl;
			break;
		case ESimOne_Lane_Type::ESimOne_Lane_Type_special1:
			std::cout << "laneType: ESimOne_Lane_Type_special1" << std::endl;
			break;
		case ESimOne_Lane_Type::ESimOne_Lane_Type_special2:
			std::cout << "laneType: ESimOne_Lane_Type_special2" << std::endl;
			break;
		case ESimOne_Lane_Type::ESimOne_Lane_Type_special3:
			std::cout << "laneType: ESimOne_Lane_Type_special3" << std::endl;
			break;
		case ESimOne_Lane_Type::ESimOne_Lane_Type_roadWorks:
			std::cout << "laneType: ESimOne_Lane_Type_roadWorks" << std::endl;
			break;
		case ESimOne_Lane_Type::ESimOne_Lane_Type_tram:
			std::cout << "laneType: ESimOne_Lane_Type_tram" << std::endl;
			break;
		case ESimOne_Lane_Type::ESimOne_Lane_Type_rail:
			std::cout << "laneType: ESimOne_Lane_Type_rail" << std::endl;
			break;
		case ESimOne_Lane_Type::ESimOne_Lane_Type_entry:
			std::cout << "laneType: ESimOne_Lane_Type_entry" << std::endl;
			break;
		case ESimOne_Lane_Type::ESimOne_Lane_Type_exit:
			std::cout << "laneType: ESimOne_Lane_Type_exit" << std::endl;
			break;
		case ESimOne_Lane_Type::ESimOne_Lane_Type_offRamp:
			std::cout << "laneType: ESimOne_Lane_Type_offRamp" << std::endl;
			break;
		case ESimOne_Lane_Type::ESimOne_Lane_Type_onRamp:
			std::cout << "laneType: ESimOne_Lane_Type_onRamp" << std::endl;
			break;
		case ESimOne_Lane_Type::ESimOne_Lane_Type_mwyEntry:
			std::cout << "laneType: ESimOne_Lane_Type_mwyEntry" << std::endl;
			break;
		case ESimOne_Lane_Type::ESimOne_Lane_Type_mwyExit:
			std::cout << "laneType: ESimOne_Lane_Type_mwyExit" << std::endl;
			break;
		default:
			std::cout << "laneType: ESimOne_Lane_Type_none"  << std::endl;
			break;
	}
	std::cout << "laneLeftID: " << pData->laneLeftID << std::endl;
	std::cout << "laneRightID: " << pData->laneRightID << std::endl;
	// for ( int i=0; i<(sizeof(pData->lanePredecessorID) / sizeof(int)); i++)
	for ( int i=0; i<3; i++)
	{
		std::cout << "lanePredecessorID[" << i << "]: " << pData->lanePredecessorID[i] << std::endl;
	}
	// for (int i=0; i<sizeof(pData->laneSuccessorID) / sizeof(int); i++)
	for (int i=0; i<3; i++)
	{
		std::cout << "laneSuccessorID[" << i << "]: " << pData->laneSuccessorID[i] << std::endl;
	}

	std::cout << "l_Line.lineID: " << pData->l_Line.lineID << std::endl;
	switch (pData->l_Line.lineType)
	{
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_solid:
			std::cout << "l_Line.lineType: ESimOne_Boundary_Type_solid" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_broken:
			std::cout << "l_Line.lineType: ESimOne_Boundary_Type_broken" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_solid_solid:
			std::cout << "l_Line.lineType: ESimOne_Boundary_Type_solid_solid" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_solid_broken:
			std::cout << "l_Line.lineType: ESimOne_Boundary_Type_solid_broken" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_broken_solid:
			std::cout << "l_Line.lineType: ESimOne_Boundary_Type_broken_solid" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_broken_broken:
			std::cout << "l_Line.lineType: ESimOne_Boundary_Type_broken_broken" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_botts_dots:
			std::cout << "l_Line.lineType: ESimOne_Boundary_Type_botts_dots" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_grass:
			std::cout << "l_Line.lineType: ESimOne_Boundary_Type_grass" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_curb:
			std::cout << "l_Line.lineType: ESimOne_Boundary_Type_curb" << std::endl;
			break;
		default:
			std::cout << "l_Line.lineType: ESimOne_Boundary_Type_none"  << std::endl;
	}
	switch (pData->l_Line.lineColor)
	{
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_blue:
			std::cout << "l_Line.lineColor: " << std::endl;
			break;
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_green:
			std::cout << "l_Line.lineColor: ESimOne_Boundary_Color_green" << std::endl;
			break;
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_red:
			std::cout << "l_Line.lineColor: ESimOne_Boundary_Color_red" << std::endl;
			break;
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_white:
			std::cout << "l_Line.lineColor: ESimOne_Boundary_Color_white" << std::endl;
			break;
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_yellow:
			std::cout << "l_Line.lineColor: ESimOne_Boundary_Color_yellow" << std::endl;
			break;
		default:
			std::cout << "l_Line.lineColor: ESimOne_Boundary_Color_standard" << std::endl;
			break;
	}
	std::cout << "l_Line.linewidth: " << pData->l_Line.linewidth << std::endl;
	// for (int i=0; i<sizeof(pData->l_Line.linePoints) / sizeof(SimOne_Data_Vec3f); i++)
	// {
	// 	std::cout << "l_Line.linePoints[" << i << "]: [" << pData->l_Line.linePoints[i].x << ", " << pData->l_Line.linePoints[i].y << ", " << pData->l_Line.linePoints[i].z << "]" << std::endl;
	// }
	std::cout << "l_Line.linecurveParameter.C0: " << pData->l_Line.linecurveParameter.C0 << std::endl;
	std::cout << "l_Line.linecurveParameter.C1: " << pData->l_Line.linecurveParameter.C1 << std::endl;
	std::cout << "l_Line.linecurveParameter.C2: " << pData->l_Line.linecurveParameter.C2 << std::endl;
	std::cout << "l_Line.linecurveParameter.C3: " << pData->l_Line.linecurveParameter.C3 << std::endl;
	std::cout << "l_Line.linecurveParameter.firstPoints: [" << pData->l_Line.linecurveParameter.firstPoints.x << ", " << pData->l_Line.linecurveParameter.firstPoints.y << ", " << pData->l_Line.linecurveParameter.firstPoints.z << "]" << std::endl;
	std::cout << "l_Line.linecurveParameter.endPoints: [" << pData->l_Line.linecurveParameter.endPoints.x << ", " << pData->l_Line.linecurveParameter.endPoints.y << ", " << pData->l_Line.linecurveParameter.endPoints.z << "]" << std::endl;
	std::cout << "l_Line.linecurveParameter.length: " << pData->l_Line.linecurveParameter.length << std::endl;

	std::cout << "c_Line.lineID: " << pData->c_Line.lineID << std::endl;
	switch (pData->c_Line.lineType)
	{
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_solid:
			std::cout << "c_Line.lineType: ESimOne_Boundary_Type_solid" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_broken:
			std::cout << "c_Line.lineType: ESimOne_Boundary_Type_broken" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_solid_solid:
			std::cout << "c_Line.lineType: ESimOne_Boundary_Type_solid_solid" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_solid_broken:
			std::cout << "c_Line.lineType: ESimOne_Boundary_Type_solid_broken" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_broken_solid:
			std::cout << "c_Line.lineType: ESimOne_Boundary_Type_broken_solid" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_broken_broken:
			std::cout << "c_Line.lineType: ESimOne_Boundary_Type_broken_broken" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_botts_dots:
			std::cout << "c_Line.lineType: ESimOne_Boundary_Type_botts_dots" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_grass:
			std::cout << "c_Line.lineType: ESimOne_Boundary_Type_grass" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_curb:
			std::cout << "c_Line.lineType: ESimOne_Boundary_Type_curb" << std::endl;
			break;
		default:
			std::cout << "c_Line.lineType: ESimOne_Boundary_Type_none"  << std::endl;
	}
	switch (pData->c_Line.lineColor)
	{
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_blue:
			std::cout << "c_Line.lineColor: " << std::endl;
			break;
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_green:
			std::cout << "c_Line.lineColor: ESimOne_Boundary_Color_green" << std::endl;
			break;
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_red:
			std::cout << "c_Line.lineColor: ESimOne_Boundary_Color_red" << std::endl;
			break;
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_white:
			std::cout << "c_Line.lineColor: ESimOne_Boundary_Color_white" << std::endl;
			break;
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_yellow:
			std::cout << "c_Line.lineColor: ESimOne_Boundary_Color_yellow" << std::endl;
			break;
		default:
			std::cout << "c_Line.lineColor: ESimOne_Boundary_Color_standard" << std::endl;
			break;
	}
	std::cout << "c_Line.linewidth: " << pData->c_Line.linewidth << std::endl;
	// for (int i=0; i<sizeof(pData->c_Line.linePoints) / sizeof(SimOne_Data_Vec3f); i++)
	// {
	// 	std::cout << "c_Line.linePoints[" << i << "]: [" << pData->c_Line.linePoints[i].x << ", " << pData->c_Line.linePoints[i].y << ", " << pData->c_Line.linePoints[i].z << "]" << std::endl;
	// }
	std::cout << "c_Line.linecurveParameter.C0: " << pData->c_Line.linecurveParameter.C0 << std::endl;
	std::cout << "c_Line.linecurveParameter.C1: " << pData->c_Line.linecurveParameter.C1 << std::endl;
	std::cout << "c_Line.linecurveParameter.C2: " << pData->c_Line.linecurveParameter.C2 << std::endl;
	std::cout << "c_Line.linecurveParameter.C3: " << pData->c_Line.linecurveParameter.C3 << std::endl;
	std::cout << "c_Line.linecurveParameter.firstPoints: [" << pData->c_Line.linecurveParameter.firstPoints.x << ", " << pData->c_Line.linecurveParameter.firstPoints.y << ", " << pData->c_Line.linecurveParameter.firstPoints.z << "]" << std::endl;
	std::cout << "c_Line.linecurveParameter.endPoints: [" << pData->c_Line.linecurveParameter.endPoints.x << ", " << pData->c_Line.linecurveParameter.endPoints.y << ", " << pData->c_Line.linecurveParameter.endPoints.z << "]" << std::endl;
	std::cout << "c_Line.linecurveParameter.length: " << pData->c_Line.linecurveParameter.length << std::endl;

	std::cout << "r_Line.lineID: " << pData->r_Line.lineID << std::endl;
	switch (pData->r_Line.lineType)
	{
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_solid:
			std::cout << "r_Line.lineType: ESimOne_Boundary_Type_solid" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_broken:
			std::cout << "r_Line.lineType: ESimOne_Boundary_Type_broken" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_solid_solid:
			std::cout << "r_Line.lineType: ESimOne_Boundary_Type_solid_solid" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_solid_broken:
			std::cout << "r_Line.lineType: ESimOne_Boundary_Type_solid_broken" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_broken_solid:
			std::cout << "r_Line.lineType: ESimOne_Boundary_Type_broken_solid" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_broken_broken:
			std::cout << "r_Line.lineType: ESimOne_Boundary_Type_broken_broken" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_botts_dots:
			std::cout << "r_Line.lineType: ESimOne_Boundary_Type_botts_dots" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_grass:
			std::cout << "r_Line.lineType: ESimOne_Boundary_Type_grass" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_curb:
			std::cout << "r_Line.lineType: ESimOne_Boundary_Type_curb" << std::endl;
			break;
		default:
			std::cout << "r_Line.lineType: ESimOne_Boundary_Type_none"  << std::endl;
	}
	switch (pData->r_Line.lineColor)
	{
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_blue:
			std::cout << "r_Line.lineColor: " << std::endl;
			break;
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_green:
			std::cout << "r_Line.lineColor: ESimOne_Boundary_Color_green" << std::endl;
			break;
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_red:
			std::cout << "r_Line.lineColor: ESimOne_Boundary_Color_red" << std::endl;
			break;
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_white:
			std::cout << "r_Line.lineColor: ESimOne_Boundary_Color_white" << std::endl;
			break;
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_yellow:
			std::cout << "r_Line.lineColor: ESimOne_Boundary_Color_yellow" << std::endl;
			break;
		default:
			std::cout << "r_Line.lineColor: ESimOne_Boundary_Color_standard" << std::endl;
			break;
	}
	std::cout << "r_Line.linewidth: " << pData->r_Line.linewidth << std::endl;
	// for (int i=0; i<sizeof(pData->r_Line.linePoints) / sizeof(SimOne_Data_Vec3f); i++)
	// {
	// 	std::cout << "r_Line.linePoints[" << i << "]: [" << pData->r_Line.linePoints[i].x << ", " << pData->r_Line.linePoints[i].y << ", " << pData->r_Line.linePoints[i].z << "]" << std::endl;
	// }
	std::cout << "r_Line.linecurveParameter.C0: " << pData->r_Line.linecurveParameter.C0 << std::endl;
	std::cout << "r_Line.linecurveParameter.C1: " << pData->r_Line.linecurveParameter.C1 << std::endl;
	std::cout << "r_Line.linecurveParameter.C2: " << pData->r_Line.linecurveParameter.C2 << std::endl;
	std::cout << "r_Line.linecurveParameter.C3: " << pData->r_Line.linecurveParameter.C3 << std::endl;
	std::cout << "r_Line.linecurveParameter.firstPoints: [" << pData->r_Line.linecurveParameter.firstPoints.x << ", " << pData->r_Line.linecurveParameter.firstPoints.y << ", " << pData->r_Line.linecurveParameter.firstPoints.z << "]" << std::endl;
	std::cout << "r_Line.linecurveParameter.endPoints: [" << pData->r_Line.linecurveParameter.endPoints.x << ", " << pData->r_Line.linecurveParameter.endPoints.y << ", " << pData->r_Line.linecurveParameter.endPoints.z << "]" << std::endl;
	std::cout << "r_Line.linecurveParameter.length: " << pData->r_Line.linecurveParameter.length << std::endl;

	std::cout << "ll_Line.lineID: " << pData->ll_Line.lineID << std::endl;
	switch (pData->ll_Line.lineType)
	{
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_solid:
			std::cout << "ll_Line.lineType: ESimOne_Boundary_Type_solid" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_broken:
			std::cout << "ll_Line.lineType: ESimOne_Boundary_Type_broken" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_solid_solid:
			std::cout << "ll_Line.lineType: ESimOne_Boundary_Type_solid_solid" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_solid_broken:
			std::cout << "ll_Line.lineType: ESimOne_Boundary_Type_solid_broken" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_broken_solid:
			std::cout << "ll_Line.lineType: ESimOne_Boundary_Type_broken_solid" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_broken_broken:
			std::cout << "ll_Line.lineType: ESimOne_Boundary_Type_broken_broken" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_botts_dots:
			std::cout << "ll_Line.lineType: ESimOne_Boundary_Type_botts_dots" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_grass:
			std::cout << "ll_Line.lineType: ESimOne_Boundary_Type_grass" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_curb:
			std::cout << "ll_Line.lineType: ESimOne_Boundary_Type_curb" << std::endl;
			break;
		default:
			std::cout << "ll_Line.lineType: ESimOne_Boundary_Type_none"  << std::endl;
	}
	switch (pData->ll_Line.lineColor)
	{
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_blue:
			std::cout << "ll_Line.lineColor: " << std::endl;
			break;
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_green:
			std::cout << "ll_Line.lineColor: ESimOne_Boundary_Color_green" << std::endl;
			break;
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_red:
			std::cout << "ll_Line.lineColor: ESimOne_Boundary_Color_red" << std::endl;
			break;
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_white:
			std::cout << "ll_Line.lineColor: ESimOne_Boundary_Color_white" << std::endl;
			break;
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_yellow:
			std::cout << "ll_Line.lineColor: ESimOne_Boundary_Color_yellow" << std::endl;
			break;
		default:
			std::cout << "ll_Line.lineColor: ESimOne_Boundary_Color_standard" << std::endl;
			break;
	}
	std::cout << "ll_Line.linewidth: " << pData->ll_Line.linewidth << std::endl;
	// for (int i=0; i<sizeof(pData->ll_Line.linePoints) / sizeof(SimOne_Data_Vec3f); i++)
	// {
	// 	std::cout << "ll_Line.linePoints[" << i << "]: [" << pData->ll_Line.linePoints[i].x << ", " << pData->ll_Line.linePoints[i].y << ", " << pData->ll_Line.linePoints[i].z << "]" << std::endl;
	// }
	std::cout << "ll_Line.linecurveParameter.C0: " << pData->ll_Line.linecurveParameter.C0 << std::endl;
	std::cout << "ll_Line.linecurveParameter.C1: " << pData->ll_Line.linecurveParameter.C1 << std::endl;
	std::cout << "ll_Line.linecurveParameter.C2: " << pData->ll_Line.linecurveParameter.C2 << std::endl;
	std::cout << "ll_Line.linecurveParameter.C3: " << pData->ll_Line.linecurveParameter.C3 << std::endl;
	std::cout << "ll_Line.linecurveParameter.firstPoints: [" << pData->ll_Line.linecurveParameter.firstPoints.x << ", " << pData->ll_Line.linecurveParameter.firstPoints.y << ", " << pData->ll_Line.linecurveParameter.firstPoints.z << "]" << std::endl;
	std::cout << "ll_Line.linecurveParameter.endPoints: [" << pData->ll_Line.linecurveParameter.endPoints.x << ", " << pData->ll_Line.linecurveParameter.endPoints.y << ", " << pData->ll_Line.linecurveParameter.endPoints.z << "]" << std::endl;
	std::cout << "ll_Line.linecurveParameter.length: " << pData->ll_Line.linecurveParameter.length << std::endl;

	std::cout << "rr_Line.lineID: " << pData->rr_Line.lineID << std::endl;
	switch (pData->rr_Line.lineType)
	{
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_solid:
			std::cout << "rr_Line.lineType: ESimOne_Boundary_Type_solid" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_broken:
			std::cout << "rr_Line.lineType: ESimOne_Boundary_Type_broken" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_solid_solid:
			std::cout << "rr_Line.lineType: ESimOne_Boundary_Type_solid_solid" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_solid_broken:
			std::cout << "rr_Line.lineType: ESimOne_Boundary_Type_solid_broken" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_broken_solid:
			std::cout << "rr_Line.lineType: ESimOne_Boundary_Type_broken_solid" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_broken_broken:
			std::cout << "rr_Line.lineType: ESimOne_Boundary_Type_broken_broken" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_botts_dots:
			std::cout << "rr_Line.lineType: ESimOne_Boundary_Type_botts_dots" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_grass:
			std::cout << "rr_Line.lineType: ESimOne_Boundary_Type_grass" << std::endl;
			break;
		case ESimOne_Boundary_Type::ESimOne_Boundary_Type_curb:
			std::cout << "rr_Line.lineType: ESimOne_Boundary_Type_curb" << std::endl;
			break;
		default:
			std::cout << "rr_Line.lineType: ESimOne_Boundary_Type_none"  << std::endl;
	}
	switch (pData->rr_Line.lineColor)
	{
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_blue:
			std::cout << "rr_Line.lineColor: " << std::endl;
			break;
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_green:
			std::cout << "rr_Line.lineColor: ESimOne_Boundary_Color_green" << std::endl;
			break;
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_red:
			std::cout << "rr_Line.lineColor: ESimOne_Boundary_Color_red" << std::endl;
			break;
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_white:
			std::cout << "rr_Line.lineColor: ESimOne_Boundary_Color_white" << std::endl;
			break;
		case ESimOne_Boundary_Color::ESimOne_Boundary_Color_yellow:
			std::cout << "rr_Line.lineColor: ESimOne_Boundary_Color_yellow" << std::endl;
			break;
		default:
			std::cout << "rr_Line.lineColor: ESimOne_Boundary_Color_standard" << std::endl;
			break;
	}
	std::cout << "rr_Line.linewidth: " << pData->rr_Line.linewidth << std::endl;
	// for (int i=0; i<sizeof(pData->rr_Line.linePoints) / sizeof(SimOne_Data_Vec3f); i++)
	// {
	// 	std::cout << "rr_Line.linePoints[" << i << "]: [" << pData->rr_Line.linePoints[i].x << ", " << pData->rr_Line.linePoints[i].y << ", " << pData->rr_Line.linePoints[i].z << "]" << std::endl;
	// }
	std::cout << "rr_Line.linecurveParameter.C0: " << pData->rr_Line.linecurveParameter.C0 << std::endl;
	std::cout << "rr_Line.linecurveParameter.C1: " << pData->rr_Line.linecurveParameter.C1 << std::endl;
	std::cout << "rr_Line.linecurveParameter.C2: " << pData->rr_Line.linecurveParameter.C2 << std::endl;
	std::cout << "rr_Line.linecurveParameter.C3: " << pData->rr_Line.linecurveParameter.C3 << std::endl;
	std::cout << "rr_Line.linecurveParameter.firstPoints: [" << pData->rr_Line.linecurveParameter.firstPoints.x << ", " << pData->rr_Line.linecurveParameter.firstPoints.y << ", " << pData->rr_Line.linecurveParameter.firstPoints.z << "]" << std::endl;
	std::cout << "rr_Line.linecurveParameter.endPoints: [" << pData->rr_Line.linecurveParameter.endPoints.x << ", " << pData->rr_Line.linecurveParameter.endPoints.y << ", " << pData->rr_Line.linecurveParameter.endPoints.z << "]" << std::endl;
	std::cout << "rr_Line.linecurveParameter.length: " << pData->rr_Line.linecurveParameter.length << std::endl;
	std::cout << "dump_sensor_laneInfo ------------------------------------------------------------------------" << std::endl;
}

void dumper::dump_waypoints(const char* mainVehicleId, SimOne_Data_WayPoints* pData)
{
	std::cout <<"------ mainVehicleId:"<< mainVehicleId << std::endl;
	std::cout <<"------ frame:"<< pData->frame << std::endl;
	std::cout <<"wayPointsSize:"<< pData->wayPointsSize << std::endl;
	for (int i=0; i<pData->wayPointsSize; i++)
	{
		std::cout << "index : " << pData->wayPoints[i].index << std::endl;
		std::cout << "posX: " << pData->wayPoints[i].posX << std::endl;
		std::cout << "posY: " << pData->wayPoints[i].posY << std::endl;
		std::cout << "heading_x: " << pData->wayPoints[i].heading_x << std::endl;
		std::cout << "heading_y: " << pData->wayPoints[i].heading_y << std::endl;
		std::cout << "heading_z: " << pData->wayPoints[i].heading_z << std::endl;
		std::cout << "heading_w: " << pData->wayPoints[i].heading_w << std::endl;
	}
}

void dumper::dump_streaming_image(SimOne_Streaming_Image* pData, const char* prefix)
{
	std::cout << prefix << "------ frame: " << pData->frame << std::endl;
	std::cout << prefix << "------ timestamp: " << pData->timestamp << std::endl;
	std::cout << prefix << "width: " << pData->width << std::endl;
	std::cout << prefix << "height :" << pData->height << std::endl;
	switch(pData->format)
	{
	case ESimOne_Streaming_Image_Format_RGB:
		std::cout << prefix << "format: ESimOne_Streaming_Image_Format_RGB" << std::endl;
		break;
	// case ESimOne_Streaming_Image_Format_RLESegmentation:
	// 	std::cout << prefix << "format: ESimOne_Streaming_Image_Format_RLESegmentation" << std::endl;
	// break;
	// case ESimOne_Streaming_Image_Format_JPEG:
	// 	std::cout << prefix << "format: ESimOne_Streaming_Image_Format_JPEG" << std::endl;
	// break;
	// case ESimOne_Streaming_Image_Format_H265:
	// 	std::cout << prefix << "format: ESimOne_Streaming_Image_Format_H265" << std::endl;
	}
	std::cout << prefix << "imageDataSize: " << pData->imageDataSize << std::endl;
	// char imageData[SOSM_IMAGE_DATA_SIZE_MAX]; // 1920 x 1200 x 3 max
}

void dumper::dump_point_cloud(SimOne_Streaming_Point_Cloud* pData, const char* prefix)
{
	std::cout << prefix << "------ frame: " << pData->frame << std::endl;
	std::cout << prefix << "width: " << pData->width << std::endl;
	std::cout << prefix << "height: " << pData->height << std::endl;
	std::cout << prefix << "pointStep: " << pData->pointStep << std::endl;
	std::cout << prefix << "pointCloudDataSize: " << pData->pointCloudDataSize << std::endl;
	// char pointCloudData[SOSM_POINT_DATA_SIZE_MAX];
}
