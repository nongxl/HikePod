#ifndef TRACKING_MANAGER_H
#define TRACKING_MANAGER_H

#include <vector>
#include <FS.h>
#include "GNSSModule.h"

// 途经标注点结构体（对齐两步路规范）
struct TrackWaypoint {
  String name;
  Location loc;
  String timeStr;
  uint64_t timeMs;
  float speed;      // 速度 (m/s)
  float accuracy;   // 精度 (HDOP/水平误差)
};

class TrackingManager {
public:
  TrackingManager();
  
  // 初始化跟踪管理器
  void begin();
  
  // 设置GNSS模块引用
  void setGNSSModule(GNSSModule* gnss);
  
  // 启动跟踪（可传入自定义文件名前缀）
  bool startTracking(const String& customPrefix = "");
  
  // 停止跟踪并生成标准KML
  void stopTracking();
  
  // 检查是否正在跟踪
  bool isTracking() const;
  
  // 更新跟踪状态，传入当前位置
  void updateTracking(const Location& currentLocation);
  
  // 获取已记录的轨迹点
  const std::vector<Location>& getTrackPoints() const;
  
  // 添加标注点 (POI)
  bool addWaypoint(const String& name, const Location& loc);
  
  // 获取已记录的标注点列表
  const std::vector<TrackWaypoint>& getWaypoints() const;
  
  // 获取统计数据
  double getTotalDistance() const { return totalDistance; }
  double getElevationGain() const { return elevationGain; }
  double getElevationLoss() const { return elevationLoss; }
  
  // 清除轨迹数据
  void clearTrack();
  
private:
  // 跟踪状态
  bool isTrackingEnabled;
  
  // GNSS模块引用
  GNSSModule* gnssModule;
  
  // 轨迹文件名与临时坐标文件
  String trackFileName;
  String tempCoordFileName;
  File tempCoordFile;
  
  // 已记录的轨迹点和标注点
  std::vector<Location> trackPoints;
  std::vector<TrackWaypoint> waypoints;
  
  // 统计数据
  double totalDistance;
  double elevationGain;
  double elevationLoss;
  unsigned long trackingStartTimeMs;
  String beginTimeIso;
  uint64_t beginTimeEpochMs;
  
  // 上一个记录点
  Location lastRecordedPoint;
  
  // 上一个记录时间
  unsigned long lastRecordTime;
  
  // 轨迹点索引
  int trackPointIndex;
  
  // 辅助函数
  bool shouldRecordPoint(const Location& currentLocation) const;
  double calculateDistance(const Location& p1, const Location& p2) const;
  String generateKMLFileName(const String& customPrefix = "") const;
  String getIsoTimeString() const;
  uint64_t getEpochTimeMs() const;
  
  // 生成并写入最终的两步路标准KML文件
  void generateFinalKML();
  
  // 确保目录存在
  void ensureTracksDirectoryExists();
};

#endif // TRACKING_MANAGER_H
