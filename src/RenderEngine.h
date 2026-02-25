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

// 视图模式定义
enum ViewMode {
  MODE_2D,         // 2D视图模式
  MODE_3D          // 3D地形模式
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
  void centerOnLocation(float lat, float lng);
  
  // 基于中心点缩放
  void zoomAroundPoint(float lat, float lng, int newZoomLevel);
  
  // 渲染整个界面
  void render(const std::vector<Location>& routePoints, const Location& currentLocation, const std::vector<Location>& trackPoints = std::vector<Location>(), bool sdInitialized = false, bool hasRoute = false, const Location* pointPool = nullptr, int pointCount = 0);
  
  // 坐标转换：经纬度到屏幕坐标
  void latLngToScreen(float lat, float lng, int& x, int& y);
  
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
  
  // 3D渲染相关方法
  void setViewMode(ViewMode mode);
  void setVerticalExaggeration(float value);
  void setCameraDistance(float distance);
  void setScaleFactor(float scale);
  void setOrientation(float pitchAngle, float rollAngle);
  void setReferenceOrientation(float pitchAngle, float rollAngle);
  void updateCameraOrientation(float currentPitch, float currentRoll, float accelX, float accelY);
  void increaseVerticalExaggeration();
  void decreaseVerticalExaggeration();
  void zoom3D(float factor);  // 3D缩放
  
  // 3D视图平移
  void pan3D(int dx, int dy);  // 平移3D视图
  
  // 切换旋转中心模式
  void toggleRotationCenter();  // 切换旋转中心：起点/地面网格中心
  
  // 重置3D视图
  void reset3DView();  // 重置3D视图到默认状态
  
  // 3D世界坐标构建（KML加载时调用一次）
  void buildWorldPoints(const Location* pointPool, int pointCount);
  void releaseWorldPoints();
  void invalidateWorldPoints();  // 标记世界坐标需要重新构建
  
  // 3D渲染核心方法
  void render3D(const std::vector<Location>& routePoints, const Location& currentLocation, const std::vector<Location>& trackPoints = std::vector<Location>(), bool sdInitialized = false, bool hasRoute = false, const Location* pointPool = nullptr, int pointCount = 0);
  
private:
  int screenWidth;
  int screenHeight;
  
  // Canvas用于离屏渲染
  M5Canvas* canvas;
  
  // GNSS模块引用
  GNSSModule* gnssModule;
  
  // 边界框（保留用于兼容性，但不再用于坐标转换）
  float minLat, maxLat, minLng, maxLng;
  
  // 统一的地图投影参数
  float viewCenterLat;    // 当前视图中心纬度（度）
  float viewCenterLng;    // 当前视图中心经度（度）
  float pixelsPerMeter;   // 统一缩放参数：每米对应的像素数
  
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
  
  // 3D渲染相关
  ViewMode viewMode;         // 当前视图模式
  float verticalExaggeration; // 垂直放大系数
  float cameraDistance;     // 相机距离
  float scaleFactor;        // 缩放因子
  bool userScaleFactor;      // 用户是否手动设置了缩放因子
  float pitch;              // 俯仰角（绕X轴）
  float roll;               // 横滚角（绕Y轴）
  
  // 参考姿态（进入3D模式时的初始姿态）
  bool hasReferenceOrientation;  // 是否已设置参考姿态
  float refPitch;           // 参考俯仰角
  float refRoll;            // 参考横滚角
  
  // 平滑过渡的目标姿态
  float targetPitch;        // 目标俯仰角
  float targetRoll;         // 目标横滚角
  
  // 视角平移（基于加速度估算）
  float viewOffsetX;        // 视角X偏移
  float viewOffsetY;        // 视角Y偏移
  float targetViewOffsetX;  // 目标视角X偏移
  float targetViewOffsetY;  // 目标视角Y偏移
  
  // 3D视图平移（用户手动调整）
  float pan3DX;             // 3D视图X平移（像素）
  float pan3DY;             // 3D视图Y平移（像素）
  
  // 旋转中心模式
  bool useCenterRotation;   // true: 以地面网格中心旋转, false: 以起点旋转
  
  // 世界坐标缓存（KML加载时计算一次）
  struct WorldPoint {
    float x;  // 公里
    float y;  // 公里
    float z;  // 公里（已应用垂直放大）
  };
  
  // 线段索引结构（用于深度排序，不复制坐标）
  struct SegmentRef {
    uint16_t i1;     // 起点索引
    uint16_t i2;     // 终点索引
    float depth;     // 平均深度
  };
  
  // 动态分配的世界坐标数组
  WorldPoint* worldPoints;
  int worldPointCount;
  
  // 动态分配的线段索引数组
  SegmentRef* segments;
  int segmentCount;
  
  // 缓存的常量（加载时计算一次）
  float cosLat0;           // cos(lat0)
  float minWorldX, maxWorldX, minWorldY, maxWorldY;  // 世界坐标范围
  
  // 缓存原始数据引用
  const Location* cachedPointPool;
  int cachedPointCount;
  double cachedLat0, cachedLon0, cachedAlt0;
  
  // 地面平面相关
  double routeMinX, routeMaxX, routeMinY, routeMaxY;
  double groundGridSize;
  
  // 计算缩放因子
  double calculateScaleFactor();
  
  // 统一的地图投影方法
  void updatePixelsPerMeter();
  float getPixelsPerMeter();
  void latLngToMeters(float lat, float lng, float& dx, float& dy);
  void metersToScreen(float dx, float dy, int& x, int& y);
  
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
  
  // 绘制3D比例尺（横纵十字）
  void draw3DScaleBar();
  
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
  
  // 3D渲染私有方法
  void draw3DCurrentLocation(const Location& currentLocation);
  void draw3DUIInfo();
  void draw3DGroundPlane(float cosP, float sinP, float cosR, float sinR, float sinA, float cosA, float sinG, float cosG);
  void draw3DVerticalLines(float cosP, float sinP, float cosR, float sinR, float sinA, float cosA, float sinG, float cosG, float minZ, float zRange, float centerX, float centerY);
  void draw3DGroundProjection(float cosP, float sinP, float cosR, float sinR, float sinA, float cosA, float sinG, float cosG, float minZ, float zRange, float centerX, float centerY);
};

#endif // RENDER_ENGINE_H
