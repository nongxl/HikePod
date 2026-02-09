#include "RenderEngine.h"
#include <M5Cardputer.h>
#include <algorithm>
#include <cmath>

// 地球半径（米）
const double EARTH_RADIUS = 6378137.0;

// 每个缩放级别对应的视图宽度（米）
const double ZOOM_VIEW_WIDTHS[] = {
  5.0,        // ZOOM_5M
  10.0,       // ZOOM_10M
  50.0,       // ZOOM_50M
  100.0,      // ZOOM_100M
  200.0,      // ZOOM_200M
  400.0,      // ZOOM_400M
  600.0,      // ZOOM_600M
  1000.0,     // ZOOM_1KM
  5000.0,     // ZOOM_5KM
  10000.0,    // ZOOM_10KM
  25000.0     // ZOOM_25KM
};

RenderEngine::RenderEngine() : 
  screenWidth(240),
  screenHeight(135),
  canvas(nullptr),
  gnssModule(nullptr),
  minLat(90.0),
  maxLat(-90.0),
  minLng(180.0),
  maxLng(-180.0),
  viewCenterLat(39.9),
  viewCenterLng(116.4),
  pixelsPerMeter(0.01),
  zoomLevel(8),  // 默认使用 ZOOM_5KM
  panOffsetX(0),
  panOffsetY(0),
  debugVisible(false),
  debugPosition(-100),
  elevationChartVisible(true),  // 默认可见
  elevationChartY(0),  // 默认位置
  lastUserActionTime(0),  // 初始化为0
  autoPanMode(0),  // 默认无自动平移
  isAutoPanning(false),  // 默认不进行自动平移
  autoPanStartTime(0),  // 初始化为0
  autoPanStartOffsetX(0),  // 初始化为0
  autoPanStartOffsetY(0),  // 初始化为0
  autoPanTargetOffsetX(0),  // 初始化为0
  autoPanTargetOffsetY(0),  // 初始化为0
  lastBatteryPercentage(-1),
  lastBatteryCheckTime(0),
  trackingState(false),
  isLocationLocked(false),
  trackingDotCounter(0),
  lastDotUpdateTime(0) {
}

void RenderEngine::begin(int width, int height) {
  screenWidth = width;
  screenHeight = height;
  updatePixelsPerMeter();
}

void RenderEngine::setCanvas(M5Canvas* canvas) {
  this->canvas = canvas;
}

void RenderEngine::calculateBoundingBox(const std::vector<Location>& points) {
  if (points.empty()) {
    // 如果没有点，使用默认边界
    minLat = 39.8042;
    maxLat = 40.0042;
    minLng = 116.3074;
    maxLng = 116.5074;
    viewCenterLat = 39.9042;
    viewCenterLng = 116.4074;
    return;
  }
  
  minLat = 90.0;
  maxLat = -90.0;
  minLng = 180.0;
  maxLng = -180.0;
  
  for (const auto& point : points) {
    if (point.latitude < minLat) minLat = point.latitude;
    if (point.latitude > maxLat) maxLat = point.latitude;
    if (point.longitude < minLng) minLng = point.longitude;
    if (point.longitude > maxLng) maxLng = point.longitude;
  }
  
  // 添加一些边距
  double latMargin = (maxLat - minLat) * 0.1;
  double lngMargin = (maxLng - minLng) * 0.1;
  minLat -= latMargin;
  maxLat += latMargin;
  minLng -= lngMargin;
  maxLng += lngMargin;
  
  // 更新视图中心点
  viewCenterLat = (minLat + maxLat) / 2.0;
  viewCenterLng = (minLng + maxLng) / 2.0;
}

void RenderEngine::calculateBoundingBoxFromPool(const Location* pointPool, int pointCount) {
  if (pointCount == 0 || pointPool == nullptr) {
    // 如果没有点，使用默认边界
    minLat = 39.8042;
    maxLat = 40.0042;
    minLng = 116.3074;
    maxLng = 116.5074;
    viewCenterLat = 39.9042;
    viewCenterLng = 116.4074;
    return;
  }
  
  minLat = 90.0;
  maxLat = -90.0;
  minLng = 180.0;
  maxLng = -180.0;
  
  // 遍历内存池中的所有点
  for (int i = 0; i < pointCount; i++) {
    const Location& point = pointPool[i];
    if (point.latitude < minLat) minLat = point.latitude;
    if (point.latitude > maxLat) maxLat = point.latitude;
    if (point.longitude < minLng) minLng = point.longitude;
    if (point.longitude > maxLng) maxLng = point.longitude;
  }
  
  // 添加一些边距
  double latMargin = (maxLat - minLat) * 0.1;
  double lngMargin = (maxLng - minLng) * 0.1;
  minLat -= latMargin;
  maxLat += latMargin;
  minLng -= lngMargin;
  maxLng += lngMargin;
  
  // 更新视图中心点
  viewCenterLat = (minLat + maxLat) / 2.0;
  viewCenterLng = (minLng + maxLng) / 2.0;
}

void RenderEngine::setZoomLevel(int level) {
  zoomLevel = constrain(level, 0, 10);
  updatePixelsPerMeter();
}

int RenderEngine::getZoomLevel() {
  return zoomLevel;
}

void RenderEngine::setPanOffset(int x, int y) {
  panOffsetX = x;
  panOffsetY = y;
}

void RenderEngine::render(const std::vector<Location>& routePoints, const Location& currentLocation, const std::vector<Location>& trackPoints, bool sdInitialized, bool hasRoute, const Location* pointPool, int pointCount) {
  // 检查canvas是否已设置
  if (!canvas) return;
  
  // 更新统一的缩放参数（确保使用最新的缩放级别）
  updatePixelsPerMeter();
  
  // 清空canvas
  canvas->fillScreen(TFT_WHITE);
  // 确保canvas的颜色模式正确
  canvas->setTextColor(TFT_BLACK);
  
  // 绘制轨迹
  if (pointPool != nullptr && pointCount > 0) {
    // 使用内存池绘制完整路径
    drawRouteFromPool(pointPool, pointCount);
  } else {
    // 使用vector版本，保持兼容性
    drawRoute(routePoints);
  }
  
  // 绘制已记录的轨迹线
  if (!trackPoints.empty()) {
    drawTrack(trackPoints);
  }
  
  // 绘制当前位置
  drawCurrentLocation(currentLocation, routePoints);
  
  // 绘制坐标信息
  drawCoordinateInfo(currentLocation);
  
  // 绘制电量信息
  drawBatteryInfo();
  
  // 绘制Tracking状态
  if (trackingState) {
    // 更新动态点计数器
    unsigned long currentTime = millis();
    if (currentTime - lastDotUpdateTime > 500) { // 每500毫秒更新一次
      trackingDotCounter = (trackingDotCounter + 1) % 4;
      lastDotUpdateTime = currentTime;
    }
    
    canvas->setTextColor(TFT_RED);
    canvas->setTextSize(1);
    canvas->setCursor(screenWidth / 2 - 25, 10);
    
    // 根据计数器显示不同数量的点
    canvas->print("Tracking");
    for (int i = 0; i < trackingDotCounter; i++) {
      canvas->print(".");
    }
    // 补空格以覆盖之前的点
    for (int i = trackingDotCounter; i < 3; i++) {
      canvas->print(" ");
    }
    
    // 重置文本颜色为黑色
    canvas->setTextColor(TFT_BLACK);
  }
  
  // 绘制比例尺
  drawScaleBar();
  
  // 更新海拔图可见性
  updateElevationChartVisibility();
  
  // 更新自动平移动画
  if (isAutoPanning) {
    unsigned long currentTime = millis();
    unsigned long elapsed = currentTime - autoPanStartTime;
    const unsigned long ANIMATION_DURATION = 300; // 300ms动画持续时间
    
    if (elapsed >= ANIMATION_DURATION) {
      // 动画完成
      panOffsetX = autoPanTargetOffsetX;
      panOffsetY = autoPanTargetOffsetY;
      isAutoPanning = false;
      // 保持autoPanMode不变，这样可以维持当前的自动平移模式
      Serial.println("[AutoPan] Animation completed");
    } else {
      // 计算动画进度（使用缓动函数）
      float progress = (float)elapsed / ANIMATION_DURATION;
      float easedProgress = progress * (2 - progress); // 简单的缓出函数
      
      // 更新偏移量
      panOffsetX = autoPanStartOffsetX + (autoPanTargetOffsetX - autoPanStartOffsetX) * easedProgress;
      panOffsetY = autoPanStartOffsetY + (autoPanTargetOffsetY - autoPanStartOffsetY) * easedProgress;
    }
  } else if (autoPanMode == 2 && currentLocation.isValid) {
    // 如果当前是平移到当前定位点模式且GPS数据有效，确保当前位置保持在屏幕中心
    int currentX, currentY;
    latLngToScreen(currentLocation.latitude, currentLocation.longitude, currentX, currentY);
    
    // 计算需要的偏移量调整，使当前位置保持在屏幕中心
    int targetOffsetX = panOffsetX - (currentX - screenWidth / 2);
    int targetOffsetY = panOffsetY - (currentY - screenHeight / 2);
    
    // 平滑过渡到目标偏移量
    if (abs(targetOffsetX - panOffsetX) > 1 || abs(targetOffsetY - panOffsetY) > 1) {
      panOffsetX += (targetOffsetX - panOffsetX) * 0.1;
      panOffsetY += (targetOffsetY - panOffsetY) * 0.1;
    } else {
      panOffsetX = targetOffsetX;
      panOffsetY = targetOffsetY;
    }
  }
  
  // 更新海拔图动画位置
  const int CHART_HEIGHT = 35;
  int targetY = elevationChartVisible ? 0 : CHART_HEIGHT + 10;
  
  // 平滑动画：使用缓动效果
  if (elevationChartY != targetY) {
    int delta = targetY - elevationChartY;
    elevationChartY += delta * 0.5; // 50%的缓动效果，加快动画速度
    
    // 确保不会过度动画
    if (abs(delta) < 1) {
      elevationChartY = targetY;
    }
  }
  
  // 绘制海拔图（考虑动画位置）
  if (elevationChartY < CHART_HEIGHT + 5) { // 只有当海拔图部分可见时才绘制
    drawElevationChart(routePoints, currentLocation, pointPool, pointCount);
  }
  
  // 绘制Debug信息
  drawDebugInfo(currentLocation, routePoints.size(), sdInitialized, hasRoute, pointCount);
  
  // 不再在这里推送，由外部统一处理
  // canvas->pushSprite(0, 0);
}

void RenderEngine::drawOperationHint() {
  // 设置文本颜色为黑色，但不设置背景色以避免黑色背景
  canvas->setTextColor(TFT_BLACK);
  canvas->setTextSize(1);
  canvas->setCursor(10, screenHeight - 20);
  canvas->println("; , . / : Pan | +/-: Zoom");
}

void RenderEngine::drawDebugInfo(const Location& currentLocation, int routePointCount, bool sdInitialized, bool hasRoute, int pointCount) {
  // 只有当debugPosition大于-100时才绘制，确保抽屉收起时完全隐藏
  if (debugPosition > -100) {
    // 不绘制背景色，直接绘制文本
    
    // 设置文本颜色为黑色
    canvas->setTextColor(TFT_BLACK);
    canvas->setTextSize(1);
    
    // 保存当前光标位置，用于整体移动（向左移动5，整体向上移动5）
    int startX = debugPosition;
    int startY = 25;
    
    // 绘制Debug标题
    canvas->setCursor(startX, startY);
    canvas->println("=== GPS DEBUG ===");
    
    // 检查GPS模块是否初始化
    bool gpsInitialized = false;
    if (gnssModule) {
      gpsInitialized = gnssModule->isModuleInitialized();
    }
    
    if (!gpsInitialized) {
      // 如果GPS未初始化，显示未初始化提示（向上移动5）
      canvas->setCursor(startX, startY + 10);
      canvas->println("GPS: NOT INITIALIZED");
      canvas->setCursor(startX, startY + 20);
      canvas->println("Press 's' in GPS Info mode");
      canvas->setCursor(startX, startY + 30);
      canvas->println("to start GPS");
    } else {
      // 绘制GPS信息（向上移动5）
      bool gpsFixed = currentLocation.isValid;
      
      // 绘制波特率
      canvas->setCursor(startX, startY + 10);
      canvas->println("Baud Rate: 115200");
      
      // 绘制接收字符数
      uint32_t gpsChars = 0;
      if (gnssModule) {
        gpsChars = gnssModule->getGpsChars();
      }
      canvas->setCursor(startX, startY + 20);
      canvas->printf("Chars RX: %u\n", gpsChars);
      
      // 绘制句子数
      uint32_t gpsSentences = 0;
      if (gnssModule) {
        gpsSentences = gnssModule->getGpsSentences();
      }
      canvas->setCursor(startX, startY + 30);
      canvas->printf("Sentences: %u\n", gpsSentences);
      
      // 绘制信号/卫星数
      int satCount = 0;
      if (gnssModule) {
        satCount = gnssModule->getSatelliteCount();
      }
      canvas->setCursor(startX, startY + 40);
      canvas->printf("Signal/Sats: %s/%d\n", gpsFixed ? "FIXED" : "SEARCHING", satCount);
      
      // 绘制经纬度
      if (gpsFixed) {
        canvas->setCursor(startX, startY + 50);
        canvas->printf("Lat/Lon: %.6f/%.6f\n", currentLocation.latitude, currentLocation.longitude);
        canvas->setCursor(startX, startY + 60);
        canvas->printf("Altitude: %.2f m\n", currentLocation.altitude);
      } else {
        canvas->setCursor(startX, startY + 50);
        canvas->println("Lat/Lon: Waiting...");
        canvas->setCursor(startX, startY + 60);
        canvas->println("Altitude: Waiting...");
      }
    }
    
    // 绘制SD卡信息（向上移动10）
    canvas->setCursor(startX, startY + 70);
    canvas->println("=== SD CARD INFO ===");
    if (sdInitialized) {
      canvas->setCursor(startX, startY + 80);
      canvas->println("SD: Ready");
      if (hasRoute) {
        canvas->setCursor(startX, startY + 90);
        canvas->printf("Route: %d points\n", routePointCount);
      } else {
        canvas->setCursor(startX, startY + 90);
        canvas->println("Route: Default");
      }
    } else {
      canvas->setCursor(startX, startY + 80);
      canvas->println("SD: Not initialized");
      canvas->setCursor(startX, startY + 90);
      canvas->println("Route: Default");
    }
    
    // 绘制路线信息
    canvas->setCursor(startX, startY + 100);
    if (pointCount > 0) {
      canvas->printf("Route Points: %d (total: %d)\n", routePointCount, pointCount);
    } else {
      canvas->printf("Route Points: %d\n", routePointCount);
    }
  }
}

void RenderEngine::latLngToScreen(double lat, double lng, int& x, int& y) {
  // 使用统一的地图投影模型
  // 1. 经纬度 -> 相对中心的米数（局部平面近似）
  double dx, dy;
  latLngToMeters(lat, lng, dx, dy);
  
  // 2. 米数 -> 屏幕坐标（使用统一的 pixelsPerMeter）
  metersToScreen(dx, dy, x, y);
}

void RenderEngine::centerOnLocation(double lat, double lng) {
  // 更新视图中心点
  viewCenterLat = lat;
  viewCenterLng = lng;
  
  // 重置平移偏移，使目标位置位于屏幕中心
  panOffsetX = 0;
  panOffsetY = 0;
}

void RenderEngine::zoomAroundPoint(double lat, double lng, int newZoomLevel) {
  // 如果传入的坐标无效，使用屏幕中心作为缩放中心
  if (lat == 0 && lng == 0) {
    // 使用当前视图中心作为缩放中心
    lat = viewCenterLat;
    lng = viewCenterLng;
  }
  
  // 计算缩放中心点在当前缩放级别下的屏幕坐标
  double dx, dy;
  latLngToMeters(lat, lng, dx, dy);
  int oldScreenX, oldScreenY;
  metersToScreen(dx, dy, oldScreenX, oldScreenY);
  
  // 更新缩放级别
  zoomLevel = constrain(newZoomLevel, 0, 10);
  
  // 更新统一的缩放参数
  updatePixelsPerMeter();
  
  // 计算缩放中心点在新缩放级别下的屏幕坐标
  metersToScreen(dx, dy, oldScreenX, oldScreenY);
  
  // 调整平移偏移量，使得中心点在屏幕上的位置保持不变
  // 由于我们使用统一的投影模型，只需要保持视图中心不变
  // 如果缩放中心不是视图中心，需要调整视图中心
  if (lat != viewCenterLat || lng != viewCenterLng) {
    // 计算缩放中心相对于视图中心的偏移（米）
    double centerDx, centerDy;
    latLngToMeters(viewCenterLat, viewCenterLng, centerDx, centerDy);
    
    // 计算缩放中心在屏幕上的位置（应该等于 oldScreenX, oldScreenY）
    int expectedScreenX, expectedScreenY;
    metersToScreen(dx, dy, expectedScreenX, expectedScreenY);
    
    // 计算需要的平移偏移
    panOffsetX = oldScreenX - expectedScreenX;
    panOffsetY = oldScreenY - expectedScreenY;
  } else {
    // 缩放中心就是视图中心，重置平移偏移
    panOffsetX = 0;
    panOffsetY = 0;
  }
}

void RenderEngine::setGNSSModule(GNSSModule* module) {
  gnssModule = module;
}

void RenderEngine::setStartPoint(const Location& startPoint) {
  this->startPoint = startPoint;
}

void RenderEngine::getPanOffset(int& x, int& y) {
  x = panOffsetX;
  y = panOffsetY;
}

double RenderEngine::calculateScaleFactor() {
  // 基础缩放因子
  double baseScale = min(
    (screenWidth * 0.8) / (maxLng - minLng),
    (screenHeight * 0.8) / (maxLat - minLat)
  );
  
  // 应用缩放级别
  double zoomFactor = 1.0 + (zoomLevel - 1) * 0.5;
  
  return baseScale * zoomFactor;
}

// 更新统一的缩放参数 pixelsPerMeter
// 这个方法确保路径绘制和比例尺使用相同的缩放因子
void RenderEngine::updatePixelsPerMeter() {
  // 获取当前缩放级别对应的视图宽度（米）
  int zoomIndex = constrain(zoomLevel, 0, 10);  // zoomLevel 直接对应枚举值（0-10）
  double viewWidthMeters = ZOOM_VIEW_WIDTHS[zoomIndex];
  
  // 计算每米对应的像素数：屏幕宽度 / 视图宽度（米）
  pixelsPerMeter = screenWidth / viewWidthMeters;
}

// 获取当前的每米像素数
double RenderEngine::getPixelsPerMeter() {
  return pixelsPerMeter;
}

// 经纬度转换为相对中心的米数（局部平面近似）
// 使用 Equirectangular 投影，以视图中心为参考点
// dx = (lon - lon0) * cos(lat0) * R
// dy = (lat - lat0) * R
void RenderEngine::latLngToMeters(double lat, double lng, double& dx, double& dy) {
  // 将经纬度转换为弧度
  double lat0 = viewCenterLat * M_PI / 180.0;
  double lat1 = lat * M_PI / 180.0;
  double lng0 = viewCenterLng * M_PI / 180.0;
  double lng1 = lng * M_PI / 180.0;
  
  // 局部平面近似计算相对距离（米）
  dx = (lng1 - lng0) * cos(lat0) * EARTH_RADIUS;
  dy = (lat1 - lat0) * EARTH_RADIUS;
}

// 相对中心的米数转换为屏幕坐标
// px = screenCenterX + dx * pixelsPerMeter + panOffsetX
// py = screenCenterY - dy * pixelsPerMeter + panOffsetY
// 注意：Y轴方向反转，因为屏幕坐标Y向下增加，而地理坐标Y（北）向上增加
void RenderEngine::metersToScreen(double dx, double dy, int& x, int& y) {
  int screenCenterX = screenWidth / 2;
  int screenCenterY = screenHeight / 2;
  
  x = (int)(screenCenterX + dx * pixelsPerMeter + panOffsetX);
  y = (int)(screenCenterY - dy * pixelsPerMeter + panOffsetY);
}

void RenderEngine::drawRoute(const std::vector<Location>& routePoints) {
  if (routePoints.size() < 2) return;
  
  // 绘制轨迹
  for (size_t i = 1; i < routePoints.size(); i++) {
    int x1, y1, x2, y2;
    latLngToScreen(routePoints[i-1].latitude, routePoints[i-1].longitude, x1, y1);
    latLngToScreen(routePoints[i].latitude, routePoints[i].longitude, x2, y2);
    
    canvas->drawLine(x1, y1, x2, y2, TFT_BLUE);
  }
}

void RenderEngine::drawRouteFromPool(const Location* pointPool, int pointCount) {
  if (pointCount < 2 || pointPool == nullptr) return;
  
  // 绘制轨迹
  for (int i = 1; i < pointCount; i++) {
    int x1, y1, x2, y2;
    latLngToScreen(pointPool[i-1].latitude, pointPool[i-1].longitude, x1, y1);
    latLngToScreen(pointPool[i].latitude, pointPool[i].longitude, x2, y2);
    
    canvas->drawLine(x1, y1, x2, y2, TFT_BLUE);
  }
}

void RenderEngine::drawCurrentLocation(const Location& location, const std::vector<Location>& routePoints) {
  int x, y;
  
  if (location.isValid) {
    // 如果有有效的GPS位置，使用该位置
    latLngToScreen(location.latitude, location.longitude, x, y);
    
    // 检查位置是否在屏幕范围内
    if (x >= -10 && x < screenWidth + 10 && y >= -10 && y < screenHeight + 10) {
      canvas->fillCircle(x, y, 3, TFT_RED);
      canvas->fillCircle(x, y, 1, TFT_WHITE);
      
      // 如果定位点被锁定，添加十字标志表示锁定
      if (isLocationLocked) {
        // 基于缩放级别调整十字准星大小
        int crossSize = 8 + zoomLevel * 2;
        if (crossSize > 20) crossSize = 20; // 限制最大大小
        
        // 绘制十字准星，带有黑色边框和白色中心，使其更醒目
        // 绘制黑色边框
        canvas->drawLine(x - crossSize, y, x + crossSize, y, TFT_BLACK);
        canvas->drawLine(x, y - crossSize, x, y + crossSize, TFT_BLACK);
        // 绘制白色中心
        canvas->drawLine(x - crossSize + 1, y, x + crossSize - 1, y, TFT_WHITE);
        canvas->drawLine(x, y - crossSize + 1, x, y + crossSize - 1, TFT_WHITE);
      }
    }
  } else {
    // 如果没有有效的GPS位置，使用KML路径上的startpoint作为默认位置
    if (startPoint.isValid) {
      // 如果有有效的起点坐标，使用该坐标
      latLngToScreen(startPoint.latitude, startPoint.longitude, x, y);
      
      // 检查位置是否在屏幕范围内
      if (x >= -10 && x < screenWidth + 10 && y >= -10 && y < screenHeight + 10) {
        canvas->fillCircle(x, y, 5, TFT_RED);
        canvas->fillCircle(x, y, 2, TFT_WHITE);
      }
    } else if (!routePoints.empty()) {
      // 如果没有有效的起点坐标，但有路线数据，则使用路线的第一个点
      Location routeStartPoint = routePoints[0];
      latLngToScreen(routeStartPoint.latitude, routeStartPoint.longitude, x, y);
      
      // 检查位置是否在屏幕范围内
      if (x >= -10 && x < screenWidth + 10 && y >= -10 && y < screenHeight + 10) {
        canvas->fillCircle(x, y, 5, TFT_RED);
        canvas->fillCircle(x, y, 2, TFT_WHITE);
      }
    } else {
      // 如果既没有有效的起点坐标，也没有路线数据，则在屏幕中心显示
      x = screenWidth / 2;
      y = screenHeight / 2;
      canvas->fillCircle(x, y, 5, TFT_RED);
      canvas->fillCircle(x, y, 2, TFT_WHITE);
    }
  }
}

void RenderEngine::drawCoordinateInfo(const Location& location) {
  // 设置文本颜色为黑色，但不设置背景色以避免黑色背景
  canvas->setTextColor(TFT_BLACK);
  canvas->setTextSize(1);
  
  int xPos = 10;
  int yPos = 10;
  int lineHeight = 10;
  
  if (location.isValid) {
    canvas->setCursor(xPos, yPos);
    canvas->printf("Lat: %.6f", location.latitude);
    yPos += lineHeight;
    
    canvas->setCursor(xPos, yPos);
    canvas->printf("Lng: %.6f", location.longitude);
    yPos += lineHeight;
    
    canvas->setCursor(xPos, yPos);
    canvas->printf("Alt: %.2f m", location.altitude);
  } else {
    canvas->setCursor(xPos, yPos);
    canvas->println("No GPS fix");
  }

}

void RenderEngine::drawSDCardInfo(bool sdInitialized, bool hasRoute, int routePointCount) {
  // 设置文本颜色为黑色，但不设置背景色以避免黑色背景
  canvas->setTextColor(TFT_BLACK);
  canvas->setTextSize(1);
  canvas->setCursor(10, 40);
  
  if (sdInitialized) {
    canvas->println("SD: Ready");
    if (hasRoute) {
      canvas->printf("Route: %d points\n", routePointCount);
    } else {
      canvas->println("Route: Default");
    }
  } else {
    canvas->println("SD: Not initialized");
    canvas->println("Route: Default");
  }
}

void RenderEngine::drawScaleBar() {
  // 绘制位置（海拔图右上方）
  const int CHART_HEIGHT = 35;
  int x = screenWidth - 70;
  int y = screenHeight - CHART_HEIGHT - 15;
  
  // 固定比例尺线段的像素长度（保持视觉上的一致性）
  const int MAX_SCALE_PIXEL_LENGTH = 50;
  
  // 根据当前缩放级别获取视图宽度
  int zoomIndex = constrain(zoomLevel, 0, 10);  // zoomLevel 直接对应枚举值（0-10）
  double viewWidthMeters = ZOOM_VIEW_WIDTHS[zoomIndex];
  
  // 计算当前的像素/米转换比例
  double pixelsPerMeter = (double)screenWidth / viewWidthMeters;
  
  // 计算固定像素长度对应的实际距离（米）
  double scaleMeters = MAX_SCALE_PIXEL_LENGTH / pixelsPerMeter;
  
  // 规范化比例尺距离为"好看"的数值（如 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000 等）
  // 找到最接近的数值
  double niceScales[] = {1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 25000};
  int niceScaleCount = 15;
  double bestScale = niceScales[0];
  double minDiff = abs(scaleMeters - niceScales[0]);
  
  for (int i = 1; i < niceScaleCount; i++) {
    double diff = abs(scaleMeters - niceScales[i]);
    if (diff < minDiff) {
      minDiff = diff;
      bestScale = niceScales[i];
    }
  }
  scaleMeters = bestScale;
  
  // 根据规范化后的距离，重新计算像素长度
  double scalePixelLength = scaleMeters * pixelsPerMeter;
  
  // 确保线段不会超出屏幕右侧边界
  int maxAllowedLength = screenWidth - x - 10; // 留出10像素的边距
  if (scalePixelLength > maxAllowedLength) {
    scalePixelLength = maxAllowedLength;
    // 重新计算对应的距离
    scaleMeters = scalePixelLength / pixelsPerMeter;
    // 再次规范化
    minDiff = abs(scaleMeters - niceScales[0]);
    bestScale = niceScales[0];
    for (int i = 1; i < niceScaleCount; i++) {
      double diff = abs(scaleMeters - niceScales[i]);
      if (diff < minDiff) {
        minDiff = diff;
        bestScale = niceScales[i];
      }
    }
    scaleMeters = bestScale;
    scalePixelLength = scaleMeters * pixelsPerMeter;
  }
  
  // 绘制刻度线（使用计算出的像素长度）
  canvas->drawLine(x, y, x + (int)scalePixelLength, y, TFT_BLACK);
  canvas->drawLine(x, y - 5, x, y + 5, TFT_BLACK);
  canvas->drawLine(x + (int)scalePixelLength, y - 5, x + (int)scalePixelLength, y + 5, TFT_BLACK);
  
  // 绘制比例尺文本
  canvas->setTextSize(1);
  int textX = x + (int)(scalePixelLength / 2) - 15;
  canvas->setCursor(textX, y - 12);
  
  if (scaleMeters >= 1000) {
    canvas->printf("%.0fkm", scaleMeters / 1000.0);
  } else {
    canvas->printf("%.0fm", scaleMeters);
  }
}

void RenderEngine::drawBatteryInfo() {
  // 设置文本颜色为黑色
  canvas->setTextColor(TFT_BLACK);
  canvas->setTextSize(1);
  
  // 每2秒检查一次电量，避免频繁刷新
  const unsigned long BATTERY_CHECK_INTERVAL = 2000;
  const int BATTERY_CHANGE_THRESHOLD = 2; // 只有变化超过2%才更新显示
  
  unsigned long currentTime = millis();
  if (currentTime - lastBatteryCheckTime > BATTERY_CHECK_INTERVAL) {
    int currentBattery = M5Cardputer.Power.getBatteryLevel();
    
    // 只有当电量变化超过阈值时才更新显示
    if (abs(currentBattery - lastBatteryPercentage) >= BATTERY_CHANGE_THRESHOLD || lastBatteryPercentage == -1) {
      lastBatteryPercentage = currentBattery;
    }
    
    lastBatteryCheckTime = currentTime;
  }
  
  // 在右上角绘制电量百分比
  if (lastBatteryPercentage >= 0) {
    canvas->setCursor(screenWidth - 25, 10);
    canvas->printf("%d%%", lastBatteryPercentage);
  }
}

void RenderEngine::drawTrack(const std::vector<Location>& trackPoints) {
  if (trackPoints.size() < 2) {
    return; // 至少需要两个点才能绘制轨迹线
  }
  
  // 绘制轨迹线
  int x1, y1, x2, y2;
  for (size_t i = 0; i < trackPoints.size() - 1; i++) {
    const Location& p1 = trackPoints[i];
    const Location& p2 = trackPoints[i + 1];
    
    // 转换经纬度到屏幕坐标
    latLngToScreen(p1.latitude, p1.longitude, x1, y1);
    latLngToScreen(p2.latitude, p2.longitude, x2, y2);
    
    // 检查点是否在屏幕范围内
    if ((x1 >= -10 && x1 < screenWidth + 10 && y1 >= -10 && y1 < screenHeight + 10) ||
        (x2 >= -10 && x2 < screenWidth + 10 && y2 >= -10 && y2 < screenHeight + 10)) {
      // 绘制线段
      canvas->drawLine(x1, y1, x2, y2, TFT_RED);
    }
  }
}

void RenderEngine::setTrackingState(bool isTracking) {
  trackingState = isTracking;
}

void RenderEngine::setLocationLocked(bool locked) {
  isLocationLocked = locked;
}

bool RenderEngine::isLocationLockedState() const {
  return isLocationLocked;
}

// 计算两个点之间的距离（米）
double RenderEngine::calculateDistance(const Location& p1, const Location& p2) {
  // 使用Haversine公式计算两点之间的距离
  const double R = 6371000.0; // 地球半径（米）
  
  double lat1 = p1.latitude * M_PI / 180.0;
  double lat2 = p2.latitude * M_PI / 180.0;
  double deltaLat = (p2.latitude - p1.latitude) * M_PI / 180.0;
  double deltaLng = (p2.longitude - p1.longitude) * M_PI / 180.0;
  
  double a = sin(deltaLat / 2) * sin(deltaLat / 2) +
             cos(lat1) * cos(lat2) *
             sin(deltaLng / 2) * sin(deltaLng / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  
  return R * c;
}

// 计算点到线段的最短距离和投影参数
double RenderEngine::distanceToSegment(const Location& point, const Location& p1, const Location& p2, double& t, Location& projection) {
  // 计算向量
  double dx = p2.longitude - p1.longitude;
  double dy = p2.latitude - p1.latitude;
  
  // 计算线段长度的平方
  double len2 = dx * dx + dy * dy;
  if (len2 < 1e-10) {
    // 线段长度为0，返回点到p1的距离
    t = 0.0;
    projection = p1;
    return calculateDistance(point, p1);
  }
  
  // 计算参数t
  double t_numerator = (point.longitude - p1.longitude) * dx + (point.latitude - p1.latitude) * dy;
  t = t_numerator / len2;
  
  // 限制t在[0,1]范围内
  if (t < 0.0) t = 0.0;
  if (t > 1.0) t = 1.0;
  
  // 计算投影点
  projection.longitude = p1.longitude + t * dx;
  projection.latitude = p1.latitude + t * dy;
  
  // 计算距离
  return calculateDistance(point, projection);
}

// 找到最近的路线线段并计算进度
bool RenderEngine::findClosestSegment(const Location* pointPool, int pointCount, const Location& currentLocation, double& progress, double& distance) {
  if (pointCount < 2 || pointPool == nullptr) {
    progress = 0.0;
    distance = 999999.0;
    return false;
  }
  
  double minDistance = 999999.0;
  int closestSegmentIndex = -1;
  double closestT = 0.0;
  Location closestProjection;
  
  // 遍历所有线段
  for (int i = 0; i < pointCount - 1; i++) {
    const Location& p1 = pointPool[i];
    const Location& p2 = pointPool[i + 1];
    
    double t;
    Location projection;
    double d = distanceToSegment(currentLocation, p1, p2, t, projection);
    
    if (d < minDistance) {
      minDistance = d;
      closestSegmentIndex = i;
      closestT = t;
      closestProjection = projection;
    }
  }
  
  if (closestSegmentIndex == -1) {
    progress = 0.0;
    distance = 999999.0;
    return false;
  }
  
  // 计算沿路线的进度
  double totalLength = 0.0;
  double segmentLength = 0.0;
  
  // 计算到最近线段起点的总长度
  for (int i = 0; i < closestSegmentIndex; i++) {
    totalLength += calculateDistance(pointPool[i], pointPool[i + 1]);
  }
  
  // 计算最近线段的长度和在该线段上的进度
  segmentLength = calculateDistance(pointPool[closestSegmentIndex], pointPool[closestSegmentIndex + 1]);
  totalLength += closestT * segmentLength;
  
  // 计算总路线长度
  double routeTotalLength = 0.0;
  for (int i = 0; i < pointCount - 1; i++) {
    routeTotalLength += calculateDistance(pointPool[i], pointPool[i + 1]);
  }
  
  // 计算进度比例
  if (routeTotalLength > 0) {
    progress = totalLength / routeTotalLength;
  } else {
    progress = 0.0;
  }
  
  distance = minDistance;
  return true;
}

void RenderEngine::drawElevationChart(const std::vector<Location>& routePoints, const Location& currentLocation, const Location* pointPool, int pointCount) {
  // 使用 pointPool 和 pointCount 作为主要数据源
  const Location* dataPoints = (pointPool != nullptr && pointCount > 0) ? pointPool : routePoints.data();
  int dataPointCount = (pointPool != nullptr && pointCount > 0) ? pointCount : routePoints.size();
  
  if (dataPointCount < 2) return;
  
  // 海拔图配置
  const int CHART_HEIGHT = 35; // 减少高度5
  const int CHART_WIDTH = screenWidth - 20;
  const int CHART_X = 10;
  const int CHART_Y = screenHeight - CHART_HEIGHT - 5 + elevationChartY;
  
  // 绘制背景
  canvas->fillRect(CHART_X, CHART_Y, CHART_WIDTH, CHART_HEIGHT, TFT_WHITE);
  canvas->drawRect(CHART_X, CHART_Y, CHART_WIDTH, CHART_HEIGHT, TFT_DARKGREY);
  
  // 计算海拔范围
  double minElevation = 10000.0;
  double maxElevation = -10000.0;
  for (int i = 0; i < dataPointCount; i++) {
    double alt = dataPoints[i].altitude;
    if (alt < minElevation) minElevation = alt;
    if (alt > maxElevation) maxElevation = alt;
  }
  
  // 添加一些边距
  double elevationRange = maxElevation - minElevation;
  if (elevationRange < 10) {
    minElevation -= 5;
    maxElevation += 5;
    elevationRange = 10;
  } else {
    minElevation -= elevationRange * 0.1;
    maxElevation += elevationRange * 0.1;
    elevationRange = maxElevation - minElevation;
  }
  
  // 绘制网格线
  canvas->drawLine(CHART_X, CHART_Y + 5, CHART_X + CHART_WIDTH, CHART_Y + 5, TFT_LIGHTGREY);
  canvas->drawLine(CHART_X, CHART_Y + CHART_HEIGHT - 5, CHART_X + CHART_WIDTH, CHART_Y + CHART_HEIGHT - 5, TFT_LIGHTGREY);
  
  // 计算点间距
  int step = max(1, (int)dataPointCount / (CHART_WIDTH - 10)); // 调整间距计算
  
  // 绘制海拔折线
  int lastX = -1;
  int lastY = -1;
  
  for (int i = 0; i < dataPointCount; i += step) {
    const auto& point = dataPoints[i];
    
    // 计算X坐标（基于点索引，接近左侧框线）
    int x = CHART_X + 5 + (int)((double)i / (dataPointCount - 1) * (CHART_WIDTH - 10));
    
    // 计算Y坐标（基于海拔）
    int y = CHART_Y + CHART_HEIGHT - 5 - (int)((point.altitude - minElevation) / elevationRange * (CHART_HEIGHT - 10));
    
    // 绘制折线
    if (lastX != -1 && lastY != -1) {
      canvas->drawLine(lastX, lastY, x, y, TFT_BLUE);
    }
    
    lastX = x;
    lastY = y;
  }
  
  // 绘制海拔范围
  canvas->setTextSize(0);
  canvas->setTextColor(TFT_BLACK);
  canvas->setCursor(CHART_X + 5, CHART_Y + 2); // 往下移，显示在框线内
  canvas->printf("%.0fm", maxElevation);
  canvas->setCursor(CHART_X + 5, CHART_Y + CHART_HEIGHT - 10);
  canvas->printf("%.0fm", minElevation);
  
  // 绘制当前位置竖线
  double progress = 0.0;
  double distanceToRoute = 0.0;
  
  if (currentLocation.isValid) {
    if (findClosestSegment(dataPoints, dataPointCount, currentLocation, progress, distanceToRoute)) {
      // 如果距离大于50米，保持在起点
      if (distanceToRoute > 50.0) {
        progress = 0.0;
      }
    } else {
      // 如果找不到最近的线段，保持在起点
      progress = 0.0;
    }
  } else {
    // 如果没有有效的GPS位置，保持在起点
    progress = 0.0;
  }
  
  // 计算竖线位置
  int lineX = CHART_X + 5 + (int)(progress * (CHART_WIDTH - 10));
  
  // 绘制竖线
  canvas->drawLine(lineX, CHART_Y, lineX, CHART_Y + CHART_HEIGHT, TFT_RED);
}

void RenderEngine::toggleDebugVisibility() {
  debugVisible = !debugVisible;
  debugPosition = debugVisible ? 5 : -100;
}

bool RenderEngine::getDebugVisible() {
  return debugVisible;
}

int RenderEngine::getDebugPosition() {
  return debugPosition;
}

void RenderEngine::updateUserAction(bool isUserAction) {
  if (isUserAction) {
    // 当有用户操作时，立即隐藏海拔图
    elevationChartVisible = false;
    // 每次有用户操作时都更新lastUserActionTime
    // 确保海拔图在用户操作期间保持隐藏
    lastUserActionTime = millis();
  }
}

void RenderEngine::updateElevationChartVisibility() {
  unsigned long currentTime = millis();
  
  // 检测debug info抽屉是否打开
  bool debugAction = (debugVisible || debugPosition > -100);
  
  // 静态变量跟踪上一次的debug action状态
  static bool lastDebugActionState = false;
  
  // 如果debug info抽屉打开，立即隐藏海拔图
  if (debugAction) {
    elevationChartVisible = false;
    lastUserActionTime = currentTime;
    lastDebugActionState = true;
    return;
  }
  
  // 如果debug info抽屉从打开变为关闭，重置lastUserActionTime
  if (lastDebugActionState && !debugAction) {
    lastUserActionTime = currentTime;
    lastDebugActionState = false;
    return;
  }
  
  // 检查是否需要显示海拔图
  // 条件：1. 没有用户操作 2. 距离上次用户操作超过3秒
  if (currentTime - lastUserActionTime > 3000) {
    elevationChartVisible = true;
  }
}

void RenderEngine::autoPanToStartPoint() {
  autoPanMode = 1;  // 平移到起点模式
  isAutoPanning = true;
  autoPanStartTime = millis();
  autoPanStartOffsetX = panOffsetX;
  autoPanStartOffsetY = panOffsetY;
  
  // 计算目标偏移量：将起点移动到屏幕中心
  int startX, startY;
  latLngToScreen(startPoint.latitude, startPoint.longitude, startX, startY);
  
  autoPanTargetOffsetX = panOffsetX - (startX - screenWidth / 2);
  autoPanTargetOffsetY = panOffsetY - (startY - screenHeight / 2);
  
  Serial.println("[AutoPan] Starting auto pan to start point");
}

void RenderEngine::autoPanToCurrentLocation() {
  autoPanMode = 2;  // 平移到当前定位点模式
  isAutoPanning = true;
  autoPanStartTime = millis();
  autoPanStartOffsetX = panOffsetX;
  autoPanStartOffsetY = panOffsetY;
  
  // 计算目标偏移量：将当前定位点移动到屏幕中心
  int currentX, currentY;
  latLngToScreen(viewCenterLat, viewCenterLng, currentX, currentY);
  
  autoPanTargetOffsetX = panOffsetX - (currentX - screenWidth / 2);
  autoPanTargetOffsetY = panOffsetY - (currentY - screenHeight / 2);
  
  Serial.println("[AutoPan] Starting auto pan to current location");
}

int RenderEngine::getAutoPanMode() {
  return autoPanMode;
}
