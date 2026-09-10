#ifndef KML_PARSER_H
#define KML_PARSER_H

#include <vector>
#include "GNSSModule.h"

// 关键点结构体
struct POI {
  String name;
  Location loc;
};

// 进度回调函数类型定义 (百分比, 已解析点数, 已读字节, 总字节, 自定义数据)
typedef void (*KMLProgressCallback)(int percent, int pointsLoaded, size_t bytesRead, size_t totalBytes, void* userData);

class KMLParser {
public:
  KMLParser();
  ~KMLParser();
  
  // 解析KML文件（支持实时进度反馈）
  bool parseFile(const char* filePath, KMLProgressCallback progressCallback = nullptr, void* userData = nullptr);
  
  // 获取解析后的路线点
  std::vector<Location> getRoutePoints();
  
  // 获取解析到的路径名称
  String getRouteName();
  
  // 检查是否达到内存限制
  bool isMemoryFull();
  
  // 获取路径起点
  Location getStartPoint();
  
  // 获取当前解析的点数量
  int getPointCount();
  
  // 重置解析器
  void reset();
  
  // 直接获取内存池中的点数据（用于计算边界框等操作）
  const Location* getPointPool();
  
  // 设置采样率（默认1，每N个点取一个）
  void setDownsampleRate(int rate);
  
  // POI 访问
  int getPOICount() const { return poiCount; }
  const POI* getPOIPool() const { return poiPool; }
  int getMaxPoints() const { return maxPoints; }
  
private:
  // 内存池相关
  static const int DEFAULT_MAX_POINTS = 3000; // 默认目标点数量
  static const int MAX_POIS = 80;             // 最大关键点数量 (80 * 40 = 3.2KB)
  
  Location* pointPool; // 预分配的内存池
  int maxPoints;       // 实际成功分配的最大容量
  int currentPointCount; // 当前点数量
  
  POI poiPool[MAX_POIS]; // 关键点池
  int poiCount;
  int downsampleRate; // 采样率
  int rawPointCounter; // 原始点计数器（用于采样）
  String routeName;
  
  // POI 解析临时状态
  String currentPoiName;
  Location currentPoiLoc;
  bool hasPoiLoc;
  
  // 直接从文件解析（低内存占用，流式实现，支持进度回调）
  bool parseFileDirect(const char* filePath, KMLProgressCallback progressCallback = nullptr, void* userData = nullptr);
  
  // 添加点到内存池
  bool addPointToPool(double lat, double lng, double alt);
  
  // 流式处理回调函数类型
  typedef void (*PointProcessor)(const Location& point, void* userData);
  
  // 流式处理点数据
  void processPointsInChunks(PointProcessor processor, void* userData, int chunkSize = 100);
};

#endif // KML_PARSER_H
