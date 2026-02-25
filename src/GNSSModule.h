#ifndef GNSS_MODULE_H
#define GNSS_MODULE_H

#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// 位置结构体（使用float节省内存）
struct Location {
  float latitude;
  float longitude;
  float altitude;
  bool isValid;
  
  Location() : latitude(0), longitude(0), altitude(0), isValid(false) {}
  Location(double lat, double lng, double alt = 0) : 
    latitude((float)lat), longitude((float)lng), altitude((float)alt), isValid(true) {}
};

class GNSSModule {
public:
  GNSSModule();
  
  // 初始化GNSS模块
  void begin(int rxPin, int txPin, long baudRate);
  
  // 关闭GNSS模块
  void end();
  
  // 更新GNSS数据
  bool update();
  
  // 获取当前位置
  Location getCurrentLocation();
  
  // 获取卫星数量
  int getSatelliteCount();
  
  // 获取定位状态
  bool isFixed();
  
  // 检查是否有可用数据
  int available();
  
  // 读取一个字符
  char read();
  
  // 检查是否初始化
  bool isModuleInitialized();
  
  // 获取速度（公里/小时）
  double getSpeedKmph();
  
  // 获取航向（度）
  double getCourseDeg();
  
  // 获取HDOP
  double getHDOP();
  
  // 获取日期（日）
  int getDay();
  
  // 获取日期（月）
  int getMonth();
  
  // 获取日期（年）
  int getYear();
  
  // 获取时间（时）
  int getHour();
  
  // 获取时间（分）
  int getMinute();
  
  // 获取时间（秒）
  int getSecond();
  
  // 检查日期是否有效
  bool isDateValid();
  
  // 检查时间是否有效
  bool isTimeValid();
  
  // 时区计算（基于经度）
  int getTimezoneOffset();
  void setTimezoneOffset(int offsetHours);
  
  // 获取本地时间（时）
  int getLocalHour();
  
  // 获取本地时间（分）
  int getLocalMinute();
  
  // 获取本地时间（秒）
  int getLocalSecond();
  
  // 获取本地日期（日）
  int getLocalDay();
  
  // 获取本地日期（月）
  int getLocalMonth();
  
  // 获取本地日期（年）
  int getLocalYear();
  
  // 根据经度自动计算时区
  void calculateTimezoneFromLocation();
  
  // 获取调试计数器
  uint32_t getGpsChars();
  uint32_t getGpsSentences();

  // 供外部调用，传入字符进行解析（避免串口读取冲突）
  bool feed(char c);

  // 待机模式控制
  void enterStandbyMode();
  void exitStandbyMode();
  bool isInStandbyMode();
  void setStandbyMode(bool standby);

private:
  HardwareSerial gpsSerial;
  TinyGPSPlus gps;
  Location currentLocation;
  int rxPin;
  int txPin;
  long baudRate;
  bool isInitialized;
  bool isInStandby; // 跟踪是否处于待机模式
  int timezoneOffsetHours; // 时区偏移（小时）
  
  // 调试计数器
  static uint32_t gpsChars;
  static uint32_t gpsSentences;
  
  // 内部方法
  void sendCommand(const String& command);
};

#endif // GNSS_MODULE_H
