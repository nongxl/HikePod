#ifndef RENDER_ENGINE_H
#define RENDER_ENGINE_H

#include <vector>
#include "GNSSModule.h"
#include <M5GFX.h>

// 缩放级别定义：每个级别对应屏幕宽度覆盖的真实距离（米）
enum MapZoom {
  ZOOM_5M,
  ZOOM_10M,
  ZOOM_50M,
  ZOOM_100M,
  ZOOM_200M,
  ZOOM_400M,
  ZOOM_600M,
  ZOOM_1KM,
  ZOOM_5KM,
  ZOOM_10KM,
  ZOOM_25KM
};

class RenderEngine {
public:
  RenderEngine();
  
  // 初始化渲染引擎
  void begin(int width, int height);
  
  // 设置Canvas用于离屏渲染
  void setCanvas(M5Canvas* canvas);
  
  // 计算边界框
  void calculateBoundingBox(const std::vector<Location>& points);
  
  // 使用内存池计算边界框（更高效，避免复制大量点）
  void calculateBoundingBoxFromPool(const Location* pointPool, int pointCount);
  
  // 设置缩放级别
  void setZoomLevel(int level);
  
  // 获取缩放级别
  int getZoomLevel();
  
  // 设置平移偏移
  void setPanOffset(int x, int y);
  
  // 居中到指定位置
  void centerOnLocation(double lat, double lng);
  
  // 基于中心点缩放
  void zoomAroundPoint(double lat, double lng, int newZoomLevel);
  
  // 渲染整个界面
  void render(const std::vector<Location>& routePoints, const Location& currentLocation, const std::vector<Location>& trackPoints = std::vector<Location>(), bool sdInitialized = false, bool hasRoute = false, const Location* pointPool = nullptr, int pointCount = 0);
  
  // 坐标转换：经纬度到屏幕坐标
  void latLngToScreen(double lat, double lng, int& x, int& y);
  
  // 设置GNSS模块引用
  void setGNSSModule(GNSSModule* gnssModule);
  
  // 设置起点坐标
  void setStartPoint(const Location& startPoint);
  
  // 获取平移偏移
  void getPanOffset(int& x, int& y);
  
  // 切换调试信息显示/隐藏
  void toggleDebugVisibility();
  
  // 获取调试信息可见性
  bool getDebugVisible();
  
  // 获取调试信息位置
  int getDebugPosition();
  
  // 更新用户操作状态
  void updateUserAction(bool isUserAction);
  
  // 更新海拔图可见性
  void updateElevationChartVisibility();
  
  // 自动平移到路径图起点
  void autoPanToStartPoint();
  
  // 自动平移到当前定位点
  void autoPanToCurrentLocation();
  
  // 获取自动平移模式
  int getAutoPanMode();
  
  // 绘制海拔图
  void drawElevationChart(const std::vector<Location>& routePoints, const Location& currentLocation, const Location* pointPool = nullptr, int pointCount = 0);
  
  // 绘制电量信息
  void drawBatteryInfo();
  
  // 使用内存池绘制轨迹（更高效，避免复制大量点）
  void drawRouteFromPool(const Location* pointPool, int pointCount);
  
  // 绘制轨迹线
  void drawTrack(const std::vector<Location>& trackPoints);
  
  // 设置tracking状态
  void setTrackingState(bool isTracking);
  
  // 设置定位点锁定状态
  void setLocationLocked(bool locked);
  
  // 获取定位点锁定状态
  bool isLocationLockedState() const;
  
private:
  int screenWidth;
  int screenHeight;
  
  // Canvas用于离屏渲染
  M5Canvas* canvas;
  
  // GNSS模块引用
  GNSSModule* gnssModule;
  
  // 边界框（保留用于兼容性，但不再用于坐标转换）
  double minLat, maxLat, minLng, maxLng;
  
  // 统一的地图投影参数
  double viewCenterLat;    // 当前视图中心纬度（度）
  double viewCenterLng;    // 当前视图中心经度（度）
  double pixelsPerMeter;   // 统一缩放参数：每米对应的像素数
  
  // 缩放和平移
  int zoomLevel;
  int panOffsetX;
  int panOffsetY;
  
  // 起点坐标
  Location startPoint;
  
  // 调试信息显示状态
  bool debugVisible;
  // 调试信息动画位置
  int debugPosition;
  
  // 海拔图相关
  bool elevationChartVisible;  // 海拔图是否可见
  int elevationChartY;  // 海拔图Y坐标（用于动画）
  unsigned long lastUserActionTime;  // 最后用户操作时间
  
  // 自动平移相关
  int autoPanMode;  // 自动平移模式：0=无, 1=平移到起点, 2=平移到当前定位点
  bool isAutoPanning;  // 是否正在进行自动平移
  unsigned long autoPanStartTime;  // 自动平移开始时间
  int autoPanStartOffsetX;  // 自动平移开始时的X偏移
  int autoPanStartOffsetY;  // 自动平移开始时的Y偏移
  int autoPanTargetOffsetX;  // 自动平移目标X偏移
  int autoPanTargetOffsetY;  // 自动平移目标Y偏移
  
  // 电量显示相关
  int lastBatteryPercentage;
  unsigned long lastBatteryCheckTime;
  
  // tracking状态
  bool trackingState;
  
  // 定位点锁定状态
  bool isLocationLocked;
  
  // Tracking模式动态点计数器
  int trackingDotCounter;
  unsigned long lastDotUpdateTime;
  
  // 计算缩放因子
  double calculateScaleFactor();
  
  // 统一的地图投影方法
  void updatePixelsPerMeter();
  double getPixelsPerMeter();
  void latLngToMeters(double lat, double lng, double& dx, double& dy);
  void metersToScreen(double dx, double dy, int& x, int& y);
  
  // 绘制轨迹
  void drawRoute(const std::vector<Location>& routePoints);
  
  // 绘制当前位置
  void drawCurrentLocation(const Location& location, const std::vector<Location>& routePoints);
  
  // 绘制坐标信息
  void drawCoordinateInfo(const Location& location);
  
  // 绘制SD卡信息
  void drawSDCardInfo(bool sdInitialized, bool hasRoute, int routePointCount);
  
  // 绘制比例尺
  void drawScaleBar();
  
  // 绘制操作提示
  void drawOperationHint();
  
  // 绘制调试信息
  void drawDebugInfo(const Location& currentLocation, int routePointCount, bool sdInitialized = false, bool hasRoute = false, int pointCount = 0);
  
  // 计算点到线段的最短距离和投影参数
  double distanceToSegment(const Location& point, const Location& p1, const Location& p2, double& t, Location& projection);
  
  // 计算两个点之间的距离（米）
  double calculateDistance(const Location& p1, const Location& p2);
  
  // 找到最近的路线线段并计算进度
  bool findClosestSegment(const Location* pointPool, int pointCount, const Location& currentLocation, double& progress, double& distance);
};

#endif // RENDER_ENGINE_H
