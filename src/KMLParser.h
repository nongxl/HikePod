#ifndef KML_PARSER_H
#define KML_PARSER_H

#include <vector>
#include "GNSSModule.h"

class KMLParser {
public:
  KMLParser();
  ~KMLParser();
  
  // 解析KML文件
  bool parseFile(const char* filePath);
  
  // 获取解析后的路线点
  std::vector<Location> getRoutePoints();
  
  // 获取轨迹名称
  String getRouteName();
  
  // 获取起点坐标
  Location getStartPoint();
  
  // 获取当前解析的点数量
  int getPointCount();
  
  // 重置解析器
  void reset();
  
  // 直接获取内存池中的点数据（用于计算边界框等操作）
  const Location* getPointPool();
  
private:
  // 内存池相关
  static const int MAX_POINTS = 10000; // 最大点数量
  Location* pointPool; // 预分配的内存池
  int currentPointCount; // 当前点数量
  String routeName;
  
  // 直接从文件解析（低内存占用）
  bool parseFileDirect(const char* filePath);
  
  // 解析坐标字符串
  bool parseCoordinates(const String& coordinatesStr);
  
  // 解析Google扩展gx:Track格式
  bool parseGxTrack(const String& trackContent);
  
  // 从文件中读取KML内容
  String readFileContent(const char* filePath);
  
  // 添加点到内存池
  bool addPointToPool(double lat, double lng, double alt);
  
  // 流式处理回调函数类型
  typedef void (*PointProcessor)(const Location& point, void* userData);
  
  // 流式处理点数据
  void processPointsInChunks(PointProcessor processor, void* userData, int chunkSize = 100);
};

#endif // KML_PARSER_H
