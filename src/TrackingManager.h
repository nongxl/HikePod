#ifndef TRACKING_MANAGER_H
#define TRACKING_MANAGER_H

#include <vector>
#include <FS.h>
#include "GNSSModule.h"

class TrackingManager {
public:
  TrackingManager();
  
  // 初始化跟踪管理器
  void begin();
  
  // 设置GNSS模块引用
  void setGNSSModule(GNSSModule* gnss);
  
  // 启动跟踪
  bool startTracking();
  
  // 停止跟踪
  void stopTracking();
  
  // 检查是否正在跟踪
  bool isTracking() const;
  
  // 更新跟踪状态，传入当前位置
  void updateTracking(const Location& currentLocation);
  
  // 获取已记录的轨迹点
  const std::vector<Location>& getTrackPoints() const;
  
  // 清除轨迹数据
  void clearTrack();
  
private:
  // 跟踪状态
  bool isTrackingEnabled;
  
  // GNSS模块引用
  GNSSModule* gnssModule;
  
  // 轨迹文件
  File trackFile;
  
  // 轨迹文件名
  String trackFileName;
  
  // 已记录的轨迹点
  std::vector<Location> trackPoints;
  
  // 上一个记录点
  Location lastRecordedPoint;
  
  // 上一个记录时间
  unsigned long lastRecordTime;
  
  // 轨迹点索引
  int trackPointIndex;
  
  // 检查是否应该记录新点
  bool shouldRecordPoint(const Location& currentLocation) const;
  
  // 计算两点之间的距离（米）
  double calculateDistance(const Location& p1, const Location& p2) const;
  
  // 生成KML文件名
  String generateKMLFileName() const;
  
  // 初始化KML文件
  bool initializeKMLFile();
  
  // 写入KML文件头
  void writeKMLHeader();
  
  // 写入轨迹点
  void writeTrackPoint(const Location& location);
  
  // 写入KML文件尾
  void writeKMLFooter();
  
  // 确保tracks目录存在
  void ensureTracksDirectoryExists();
};

#endif // TRACKING_MANAGER_H
