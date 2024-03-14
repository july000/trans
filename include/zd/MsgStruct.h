#ifndef MsgSTRUCT_H_
#define MsgSTRUCT_H_
#include <string>
#include <map>

typedef struct
{
  bool isInitialized;
  int gear;
  float throttle;
  float brake;
  float steering;
  float unnormalized_steering;
  long long timestamp;
} Control_t;

typedef struct
{
  int HighBeam;
  int LeftBlinker;
  int RightBlinker;
  int gear;
  long long timestamp;
} SignalLight_t;

typedef struct
{
  long long timestamp;
} Wiper_t;

// enum CRUISE_CNTRL
// {
//   SPEED_UNKNOWN = 0,
//   SPEED_KEEP = 1,
//   SPEED_UP = 2,
//   SPEED_DOWN = 3
// };

typedef struct
{
  // CRUISE_CNTRL speed_ctrl;
  bool isCruiseOn = false;
  bool isSpeedSet = false;
  bool isSpeedUp = false;
  bool isSpeedDown = false;
  bool isCanceled = false;
  bool isResumed = false;
  long long timestamp;
} DrivingLever_t;

typedef struct {
    double timestamp;       // 时间戳
    int network;            // 网络
    int id;         // ID
    std::string direction;  // 方向
    std::string state;      // 状态
    int dlc;                // 数据长度
    std::vector<int> data;  // 信息数据
} ASC_t;

typedef struct {
  std::string name;
  std::string raw;
} SIGNAL_t;

// class RrConfig
// {
// public:
// 	RrConfig()
// 	{
// 	}
// 	~RrConfig()
// 	{
// 	}
// 	bool ReadConfig(const std::string & filename);
// 	std::string ReadString(const char* section, const char* item, const char* default_value);
// 	int ReadInt(const char* section, const char* item, const int& default_value);
// 	float ReadFloat(const char* section, const char* item, const float& default_value);
// private:
// 	bool IsSpace(char c);
// 	bool IsCommentChar(char c);
// 	void Trim(std::string & str);
// 	bool AnalyseLine(const std::string & line, std::string& section, std::string & key, std::string & value);

// private:
// 	//std::map<std::string, std::string> settings_;
// 	std::map<std::string, std::map<std::string, std::string> >settings_;
// };
#endif