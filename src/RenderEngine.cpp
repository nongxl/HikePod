#include "RenderEngine.h"
#include <M5Cardputer.h>
#include <algorithm>
#include <cmath>
#include <esp_system.h>

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
  25000.0,    // ZOOM_25KM
  50000.0,    // ZOOM_50KM
  100000.0    // ZOOM_100KM
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
  targetPixelsPerMeter(0.01),
  zoomAnchorLat(0.0),
  zoomAnchorLng(0.0),
  floatPanX(0.0f),
  floatPanY(0.0f),
  targetPanOffsetX(0.0f),
  targetPanOffsetY(0.0f),
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
  // 3D渲染相关初始化
  viewMode(MODE_2D),
  verticalExaggeration(1.5f),
  cameraDistance(500.0),
  scaleFactor(1.0),
  targetScaleFactor(1.0),
  userScaleFactor(false),  // 初始化为自动计算
  pitch(0.0),
  roll(0.0),
  hasReferenceOrientation(false),
  refPitch(0.0),
  refRoll(0.0),
  targetPitch(0.0),
  targetRoll(0.0),
  viewOffsetX(0.0),
  viewOffsetY(0.0),
  targetViewOffsetX(0.0),
  targetViewOffsetY(0.0),
  pan3DX(0.0f),
  pan3DY(0.0f),
  targetPan3DX(0.0f),
  targetPan3DY(0.0f),
  useCenterRotation(true),
  worldPoints(nullptr),
  worldPointCount(0),
  segments(nullptr),
  segmentCount(0),
  projectedVertices(nullptr),
  cosLat0(1.0f),
  minWorldX(0.0f),
  maxWorldX(0.0f),
  minWorldY(0.0f),
  maxWorldY(0.0f),
  minWorldZ(0.0f),
  maxWorldZ(0.0f),
  routeMinX(0.0),
  routeMaxX(0.0),
  routeMinY(0.0),
  routeMaxY(0.0),
  groundGridSize(10.0),
  cachedPointPool(nullptr),
  cachedPointCount(0),
  cachedLat0(0.0),
  cachedLon0(0.0),
  cachedAlt0(0.0),
  lastBatteryPercentage(-1),
  lastBatteryCheckTime(0),
  trackingState(false),
  isLocationLocked(true),
  trackingDotCounter(0),
  lastDotUpdateTime(0) {
}

void RenderEngine::begin(int width, int height) {
  screenWidth = width;
  screenHeight = height;
  updatePixelsPerMeter();
  targetPixelsPerMeter = pixelsPerMeter;
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

void RenderEngine::autoFitToRoute() {
  if (minLat >= maxLat || minLng >= maxLng) return;

  // 设置中心点为起点（用户要求起点居中）
  viewCenterLat = startPoint.latitude;
  viewCenterLng = startPoint.longitude;
  
  // 重置平移偏移（配合起点居中）
  panOffsetX = 0;
  panOffsetY = 0;

  // 计算路径对角线跨度
  double distLat = calculateDistance(Location(minLat, minLng), Location(maxLat, minLng));
  double distLng = calculateDistance(Location(minLat, minLng), Location(minLat, maxLng));
  double maxSpan = (distLat > distLng) ? distLat : distLng;
  
  // 加上 20% 的余量
  double requiredWidth = maxSpan * 1.2;
  if (requiredWidth < 5.0) requiredWidth = 5.0;

  // 连续比例计算
  pixelsPerMeter = (float)(screenWidth / requiredWidth);
  targetPixelsPerMeter = pixelsPerMeter;
  floatPanX = 0.0f;
  floatPanY = 0.0f;
  targetPanOffsetX = 0.0f;
  targetPanOffsetY = 0.0f;

  // 寻找合适的缩放级别：找到第一个能容纳 requiredWidth 的级别
  int bestZoom = 12; // 默认最大宽度（最小缩放）
  for (int i = 0; i <= 12; i++) {
    if (ZOOM_VIEW_WIDTHS[i] >= requiredWidth) {
      bestZoom = i;
      break;
    }
  }
  zoomLevel = bestZoom;
  
  Serial.print("=== RenderEngine: Auto-fit. MaxSpan=");
  Serial.print(maxSpan);
  Serial.print("m, RequiredWidth=");
  Serial.print(requiredWidth);
  Serial.print("m, Chosen Zoom=");
  Serial.print(zoomLevel);
  Serial.print(" (PPM: ");
  Serial.print(pixelsPerMeter, 4);
  Serial.println(")");
}

void RenderEngine::setZoomLevel(int level) {
  zoomLevel = constrain(level, 0, 12);
  updatePixelsPerMeter();
  targetPixelsPerMeter = pixelsPerMeter;
}

int RenderEngine::getZoomLevel() {
  return zoomLevel;
}

void RenderEngine::setPanOffset(int x, int y) {
  panOffsetX = x;
  panOffsetY = y;
  floatPanX = (float)x;
  floatPanY = (float)y;
  targetPanOffsetX = (float)x;
  targetPanOffsetY = (float)y;
}

void RenderEngine::pan2D(float dx, float dy) {
  targetPanOffsetX += dx;
  targetPanOffsetY += dy;
}

void RenderEngine::render(const std::vector<Location>& routePoints, const Location& currentLocation, const std::vector<Location>& trackPoints, bool sdInitialized, bool hasRoute, const Location* pointPool, int pointCount, const POI* poiPool, int poiCount, int showPOIsMode) {
  // 检查canvas是否已设置
  if (!canvas) return;
  
  // 根据视图模式选择渲染方法
  if (viewMode == MODE_3D) {
    render3D(routePoints, currentLocation, trackPoints, sdInitialized, hasRoute, pointPool, pointCount, poiPool, poiCount, showPOIsMode);
    return;
  }
  
  // 2D渲染模式
  if (isLocationLocked && currentLocation.isValid) {
    centerOnLocation(currentLocation.latitude, currentLocation.longitude);
  }

  // 清空canvas
  canvas->fillScreen(TFT_WHITE);
  // 确保canvas的颜色模式正确
  canvas->setTextColor(TFT_BLACK);
  canvas->setTextSize(1);
  canvas->setTextDatum(MC_DATUM);
  
  // 设置屏幕亮度
  // 注意：这里不设置亮度，因为亮度应该由main.cpp中的全局变量控制
  // M5Cardputer.Display.setBrightness(255);
  
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
  
  // 绘制 2D 关键点
  if (showPOIsMode == 1) {
    drawPOIs(poiPool, poiCount, true);
  } else if (showPOIsMode == 2) {
    const Location* refPoints = (pointPool != nullptr && pointCount > 0) ? pointPool : routePoints.data();
    int refCount = (pointPool != nullptr && pointCount > 0) ? pointCount : routePoints.size();
    if (refCount < 2 && trackPoints.size() >= 2) {
      refPoints = trackPoints.data();
      refCount = trackPoints.size();
    }
    drawPOIsAuto(poiPool, poiCount, currentLocation, refPoints, refCount);
  }
  
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
    
    // 绘制系统内存使用情况（居右对齐，显示在电量信息下方，与Debug标题同一行）
    char memStr[32];
    uint32_t freeMem = esp_get_free_heap_size() / 1024;
    snprintf(memStr, sizeof(memStr), "IdleMem: %uKB", (unsigned int)freeMem);
    int memX = screenWidth - 10 - canvas->textWidth(memStr);
    canvas->setCursor(memX, startY);
    canvas->print(memStr);
    
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

void RenderEngine::latLngToScreen(float lat, float lng, int& x, int& y) {
  float dx, dy;
  latLngToMeters(lat, lng, dx, dy);
  metersToScreen(dx, dy, x, y);
}

void RenderEngine::screenToLatLng(int x, int y, float& lat, float& lng) {
  int screenCenterX = screenWidth / 2;
  int screenCenterY = screenHeight / 2;
  
  // 从 metersToScreen 反推 dx, dy
  // x = screenCenterX + dx * pixelsPerMeter + panOffsetX => dx = (x - screenCenterX - panOffsetX) / pixelsPerMeter
  // y = screenCenterY - dy * pixelsPerMeter + panOffsetY => dy = (screenCenterY + panOffsetY - y) / pixelsPerMeter
  
  float dx = (float)(x - screenCenterX - panOffsetX) / pixelsPerMeter;
  float dy = (float)(screenCenterY + panOffsetY - y) / pixelsPerMeter;
  
  // 从 latLngToMeters 反推 lat, lng
  // dy = (lat1 - lat0) * EARTH_RADIUS => lat1 = lat0 + dy / EARTH_RADIUS
  // dx = (lng1 - lng0) * cosf(lat0) * EARTH_RADIUS => lng1 = lng0 + dx / (cosf(lat0) * EARTH_RADIUS)
  
  float lat0 = viewCenterLat * (float)M_PI / 180.0f;
  lat = viewCenterLat + (dy / (float)EARTH_RADIUS) * 180.0f / (float)M_PI;
  lng = viewCenterLng + (dx / (cosf(lat0) * (float)EARTH_RADIUS)) * 180.0f / (float)M_PI;
}

void RenderEngine::centerOnLocation(float lat, float lng) {
  viewCenterLat = lat;
  viewCenterLng = lng;
  panOffsetX = 0;
  panOffsetY = 0;
  floatPanX = 0.0f;
  floatPanY = 0.0f;
  targetPanOffsetX = 0.0f;
  targetPanOffsetY = 0.0f;
}

void RenderEngine::zoomAroundPoint(float lat, float lng, int newZoomLevel) {
  if (lat == 0 && lng == 0) {
    lat = viewCenterLat;
    lng = viewCenterLng;
  }
  
  float dx, dy;
  latLngToMeters(lat, lng, dx, dy);
  int oldScreenX, oldScreenY;
  metersToScreen(dx, dy, oldScreenX, oldScreenY);
  
  // 记录缩放前的 pixelsPerMeter
  float oldPixelsPerMeter = pixelsPerMeter;
  
  zoomLevel = constrain(newZoomLevel, 0, 10);
  updatePixelsPerMeter();
  targetPixelsPerMeter = pixelsPerMeter;
  
  panOffsetX += (int)(dx * (oldPixelsPerMeter - pixelsPerMeter));
  panOffsetY -= (int)(dy * (oldPixelsPerMeter - pixelsPerMeter));
  floatPanX = (float)panOffsetX;
  floatPanY = (float)panOffsetY;
  targetPanOffsetX = floatPanX;
  targetPanOffsetY = floatPanY;
  
  Serial.printf("[ZOOM] zoomAroundPoint: zoom=%d, dx=%.2f, dy=%.2f, ppm_old=%.4f, ppm_new=%.4f, panX=%d, panY=%d\n", 
                zoomLevel, dx, dy, oldPixelsPerMeter, pixelsPerMeter, panOffsetX, panOffsetY);
}

void RenderEngine::zoom2D(float factor, float anchorLat, float anchorLng) {
  if (targetPixelsPerMeter <= 0.00001f) {
    targetPixelsPerMeter = (pixelsPerMeter > 0.00001f) ? pixelsPerMeter : 0.01f;
  }

  // 几何级数等比缩放
  targetPixelsPerMeter *= factor;

  // 限制缩放范围：最小 200km 视野，最大 3m 视野
  const float MIN_PPM = 0.0012f; // 240 / 200000m
  const float MAX_PPM = 80.0f;   // 240 / 3m
  if (targetPixelsPerMeter < MIN_PPM) targetPixelsPerMeter = MIN_PPM;
  if (targetPixelsPerMeter > MAX_PPM) targetPixelsPerMeter = MAX_PPM;

  Serial.printf("[ZOOM 2D] factor=%.2f, currPPM=%.4f, targetPPM=%.4f\n",
                factor, pixelsPerMeter, targetPixelsPerMeter);
}

bool RenderEngine::update2DCameraTransition() {
  bool isTransitioning = false;

  // 1. 缩放平滑阻尼逼近 (Lerp)
  float scaleDiff = targetPixelsPerMeter - pixelsPerMeter;
  if (fabsf(scaleDiff) > 0.000005f) {
    float oldPpm = pixelsPerMeter;
    // 阻尼逼近：每帧追赶 25% 差值，带来平滑丝滑缩放动画
    pixelsPerMeter += scaleDiff * 0.25f;

    if (fabsf(targetPixelsPerMeter - pixelsPerMeter) < 0.00001f) {
      pixelsPerMeter = targetPixelsPerMeter;
    }

    // 屏幕中心对齐几何放缩：
    // 以屏幕中心为基准缩放时，原点相对于屏幕中心的偏移按 pixelsPerMeter 的比例同比例伸缩。
    // 在定位锁定跟随状态下 floatPanX/Y 为 0，乘任何比率恒为 0，定位点绝对稳定居中，绝无漂移！
    if (oldPpm > 0.000001f) {
      float ratio = pixelsPerMeter / oldPpm;
      floatPanX *= ratio;
      floatPanY *= ratio;
      targetPanOffsetX *= ratio;
      targetPanOffsetY *= ratio;
    }

    // 动态同步离散 zoomLevel 供旧逻辑读取
    if (pixelsPerMeter > 0.00001f) {
      float currentViewWidth = (float)screenWidth / pixelsPerMeter;
      int bestZoom = 12;
      for (int i = 0; i <= 12; i++) {
        if (ZOOM_VIEW_WIDTHS[i] >= currentViewWidth) {
          bestZoom = i;
          break;
        }
      }
      zoomLevel = bestZoom;
    }

    isTransitioning = true;
  }

  // 2. 平移平滑阻尼逼近 (Lerp)，带来丝滑的惯性滑行手感
  float panDx = targetPanOffsetX - floatPanX;
  float panDy = targetPanOffsetY - floatPanY;
  if (fabsf(panDx) > 0.3f || fabsf(panDy) > 0.3f) {
    floatPanX += panDx * 0.35f;
    floatPanY += panDy * 0.35f;
    panOffsetX = (int)roundf(floatPanX);
    panOffsetY = (int)roundf(floatPanY);
    isTransitioning = true;
  } else if (floatPanX != targetPanOffsetX || floatPanY != targetPanOffsetY) {
    floatPanX = targetPanOffsetX;
    floatPanY = targetPanOffsetY;
    panOffsetX = (int)roundf(floatPanX);
    panOffsetY = (int)roundf(floatPanY);
    isTransitioning = true;
  }

  return isTransitioning;
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

void RenderEngine::updatePixelsPerMeter() {
  int zoomIndex = constrain(zoomLevel, 0, 12);
  float viewWidthMeters = (float)ZOOM_VIEW_WIDTHS[zoomIndex];
  pixelsPerMeter = screenWidth / viewWidthMeters;
}

float RenderEngine::getPixelsPerMeter() {
  return pixelsPerMeter;
}

void RenderEngine::latLngToMeters(float lat, float lng, float& dx, float& dy) {
  float lat0 = viewCenterLat * (float)M_PI / 180.0f;
  float lat1 = lat * (float)M_PI / 180.0f;
  float lng0 = viewCenterLng * (float)M_PI / 180.0f;
  float lng1 = lng * (float)M_PI / 180.0f;
  
  dx = (lng1 - lng0) * cosf(lat0) * (float)EARTH_RADIUS;
  dy = (lat1 - lat0) * (float)EARTH_RADIUS;
}

void RenderEngine::metersToScreen(float dx, float dy, int& x, int& y) {
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

// 绘制航向渐变蓝色扇形视野锥（末端变淡融入背景，整体呈纯净渐变天蓝色）
void RenderEngine::drawCourseHeadingCone(int cx, int cy, float courseDeg) {
  if (courseDeg < 0.0f) return;

  const float DEG_TO_RAD_FACTOR = 0.0174532925f;
  const float R_MAX = 24.0f;          // 扇形末端最大半径
  const float R_MIN = 3.5f;           // 扇形根部最小半径（从定位圆点边缘展开）
  const float FAN_HALF_ANGLE = 28.0f; // 半张角 28 度（总张角 56 度，与主流现代地图对齐）
  const float STEP_DEG = 3.5f;        // 弧度逼近步长
  const int NUM_BANDS = 14;           // 径向渐变分层数，层间距极小带来丝滑过渡

  float startDeg = courseDeg - FAN_HALF_ANGLE;
  float endDeg = courseDeg + FAN_HALF_ANGLE;

  // 自外向内采用画家算法逐层绘制同心扇形阶梯，外浅内深形成通透纯净的径向渐变淡出效果
  for (int band = NUM_BANDS - 1; band >= 0; band--) {
    float t = (float)band / (float)(NUM_BANDS - 1); // 0.0 (最深/最内) ~ 1.0 (最淡/最外)
    float r = R_MIN + (R_MAX - R_MIN) * t;

    // 非线性衰减插值曲线：末端完全隐入纯白背景，根部呈高饱和鲜艳天蓝
    float factor = powf(t, 0.70f);
    uint8_t red   = (uint8_t)(30  + (255 - 30)  * factor);
    uint8_t green = (uint8_t)(136 + (255 - 136) * factor);
    uint8_t blue  = 255; // 蓝分量恒定饱满，杜绝灰暗杂色

    uint16_t bandColor = canvas->color565(red, green, blue);

    int prevX = -1, prevY = -1;
    for (float deg = startDeg; deg <= endDeg + 0.1f; deg += STEP_DEG) {
      float rad = deg * DEG_TO_RAD_FACTOR;
      int px = cx + (int)roundf(r * sinf(rad));
      int py = cy - (int)roundf(r * cosf(rad));

      if (prevX != -1) {
        canvas->fillTriangle(cx, cy, prevX, prevY, px, py, bandColor);
      }
      prevX = px;
      prevY = py;
    }
  }
}

void RenderEngine::drawCurrentLocation(const Location& location, const std::vector<Location>& routePoints) {
  int x, y;
  
  if (location.isValid) {
    // 如果有有效的GPS位置，使用该位置
    latLngToScreen(location.latitude, location.longitude, x, y);
    
    // 检查位置是否在屏幕范围内
    if (x >= -10 && x < screenWidth + 10 && y >= -10 && y < screenHeight + 10) {
      // 1. 绘制行进方向渐变蓝色扇形视野锥（末端渐淡）
      if (location.course >= 0.0f) {
        drawCourseHeadingCone(x, y, location.course);
      }

      // 2. 绘制现代地图风格定位标记（白色描边外环 + 鲜艳天蓝圆心）
      canvas->fillCircle(x, y, 4, TFT_WHITE);
      canvas->fillCircle(x, y, 3, canvas->color565(30, 136, 229));
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

      // 当航速大于 0.1km/h 时，在定位点旁边直接显示简约黑色航速文字（无底框、不显示度数）
      if (location.speed > 0.1f) {
        String speedStr = (location.speed < 10.0f) ? (String(location.speed, 1) + "km/h") : (String((int)roundf(location.speed)) + "km/h");
        
        canvas->setFont(&fonts::Font0);
        canvas->setTextSize(1);
        int textW = canvas->textWidth(speedStr);
        int textH = 8;
        
        int textX = x + 6;
        int textY = y - 4;
        if (textX + textW > screenWidth - 2) {
          textX = x - textW - 6;
        }
        if (textY < 2) {
          textY = y + 6;
        }
        
        canvas->setTextColor(TFT_BLACK);
        canvas->setCursor(textX, textY);
        canvas->print(speedStr);
      }
    } else {
      // 位置在屏幕外，计算距离最近的屏幕边缘并绘制蓝色三角形
      int triSize = 10; // 三角形大小
      int triHeight = 8; // 三角形高度
      
      // 计算到各屏幕边缘的距离（使用绝对值）
      int distLeft = abs(x);
      int distRight = abs(x - screenWidth);
      int distTop = abs(y);
      int distBottom = abs(y - screenHeight);
      
      // 找到最小的距离（绝对值）
      int minDist = std::min({distLeft, distRight, distTop, distBottom});
      
      // 绘制蓝色实心钝角等腰三角形
      canvas->fillTriangle(0, 0, 0, 0, 0, 0, TFT_BLUE); // 占位，下面会覆盖
      
      if (minDist == distLeft) {
        // 左侧边缘，三角形指向右（指向红圈）
        int triX = 0;
        int triY = std::max(0, std::min(screenHeight - triSize, y));
        canvas->fillTriangle(triX, triY, triX, triY + triSize, triX + triHeight, triY + triSize/2, TFT_BLUE);
      } else if (minDist == distRight) {
        // 右侧边缘，三角形指向左（指向红圈）
        int triX = screenWidth - 1;
        int triY = std::max(0, std::min(screenHeight - triSize, y));
        canvas->fillTriangle(triX, triY, triX, triY + triSize, triX - triHeight, triY + triSize/2, TFT_BLUE);
      } else if (minDist == distTop) {
        // 顶部边缘，三角形指向下（指向红圈）
        int triX = std::max(0, std::min(screenWidth - triSize, x));
        int triY = 0;
        canvas->fillTriangle(triX, triY, triX + triSize, triY, triX + triSize/2, triY + triHeight, TFT_BLUE);
      } else if (minDist == distBottom) {
        // 底部边缘，三角形指向上（指向红圈）
        int triX = std::max(0, std::min(screenWidth - triSize, x));
        int triY = screenHeight - 1;
        canvas->fillTriangle(triX, triY, triX + triSize, triY, triX + triSize/2, triY - triHeight, TFT_BLUE);
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
      } else {
        // 位置在屏幕外，计算距离最近的屏幕边缘并绘制蓝色三角形
        int triSize = 10; // 三角形大小
        int triHeight = 8; // 三角形高度
        
        // 计算到各屏幕边缘的距离（使用绝对值）
        int distLeft = abs(x);
        int distRight = abs(x - screenWidth);
        int distTop = abs(y);
        int distBottom = abs(y - screenHeight);
        
        // 找到最小的距离（绝对值）
        int minDist = std::min({distLeft, distRight, distTop, distBottom});
        
        // 绘制蓝色实心钝角等腰三角形
        if (minDist == distLeft) {
          // 左侧边缘
          int triX = 0;
          int triY = std::max(0, std::min(screenHeight - triSize, y));
          canvas->fillTriangle(triX, triY, triX, triY + triSize, triX + triHeight, triY + triSize/2, TFT_BLUE);
        } else if (minDist == distRight) {
          // 右侧边缘
          int triX = screenWidth - 1;
          int triY = std::max(0, std::min(screenHeight - triSize, y));
          canvas->fillTriangle(triX, triY, triX, triY + triSize, triX - triHeight, triY + triSize/2, TFT_BLUE);
        } else if (minDist == distTop) {
          // 顶部边缘
          int triX = std::max(0, std::min(screenWidth - triSize, x));
          int triY = 0;
          canvas->fillTriangle(triX, triY, triX + triSize, triY, triX + triSize/2, triY + triHeight, TFT_BLUE);
        } else if (minDist == distBottom) {
          // 底部边缘
          int triX = std::max(0, std::min(screenWidth - triSize, x));
          int triY = screenHeight - 1;
          canvas->fillTriangle(triX, triY, triX + triSize, triY, triX + triSize/2, triY - triHeight, TFT_BLUE);
        }
      }
    } else if (!routePoints.empty()) {
      // 如果没有有效的起点坐标，但有路线数据，则使用路线的第一个点
      Location routeStartPoint = routePoints[0];
      latLngToScreen(routeStartPoint.latitude, routeStartPoint.longitude, x, y);
      
      // 检查位置是否在屏幕范围内
      if (x >= -10 && x < screenWidth + 10 && y >= -10 && y < screenHeight + 10) {
        canvas->fillCircle(x, y, 5, TFT_RED);
        canvas->fillCircle(x, y, 2, TFT_WHITE);
      } else {
        // 位置在屏幕外，计算距离最近的屏幕边缘并绘制蓝色三角形
        int triSize = 10; // 三角形大小
        int triHeight = 4; // 三角形高度
        
        // 计算到各屏幕边缘的距离（使用绝对值）
        int distLeft = abs(x);
        int distRight = abs(x - screenWidth);
        int distTop = abs(y);
        int distBottom = abs(y - screenHeight);
        
        // 找到最小的距离（绝对值）
        int minDist = std::min({distLeft, distRight, distTop, distBottom});
        
        // 绘制蓝色实心钝角等腰三角形
        if (minDist == distLeft) {
          // 左侧边缘
          int triX = 0;
          int triY = std::max(0, std::min(screenHeight - triSize, y));
          canvas->fillTriangle(triX, triY, triX, triY + triSize, triX + triHeight, triY + triSize/2, TFT_BLUE);
        } else if (minDist == distRight) {
          // 右侧边缘
          int triX = screenWidth - 1;
          int triY = std::max(0, std::min(screenHeight - triSize, y));
          canvas->fillTriangle(triX, triY, triX, triY + triSize, triX - triHeight, triY + triSize/2, TFT_BLUE);
        } else if (minDist == distTop) {
          // 顶部边缘
          int triX = std::max(0, std::min(screenWidth - triSize, x));
          int triY = 0;
          canvas->fillTriangle(triX, triY, triX + triSize, triY, triX + triSize/2, triY + triHeight, TFT_BLUE);
        } else if (minDist == distBottom) {
          // 底部边缘
          int triX = std::max(0, std::min(screenWidth - triSize, x));
          int triY = screenHeight - 1;
          canvas->fillTriangle(triX, triY, triX + triSize, triY, triX + triSize/2, triY - triHeight, TFT_BLUE);
        }
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
  
  // 固定比例尺线段的目标基准像素长度
  const int MAX_SCALE_PIXEL_LENGTH = 50;
  
  // 直接使用当前的平滑 pixelsPerMeter 转换比例
  double ppm = (double)this->pixelsPerMeter;
  if (ppm <= 0.000001) ppm = 0.000001;
  
  // 计算固定像素长度对应的实际距离（米）
  double scaleMeters = MAX_SCALE_PIXEL_LENGTH / ppm;
  
  // 规范化比例尺距离为整洁易读的数值
  const double niceScales[] = {1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000};
  const int niceScaleCount = sizeof(niceScales) / sizeof(niceScales[0]);
  double bestScale = niceScales[0];
  double minDiff = fabs(scaleMeters - niceScales[0]);
  
  for (int i = 1; i < niceScaleCount; i++) {
    double diff = fabs(scaleMeters - niceScales[i]);
    if (diff < minDiff) {
      minDiff = diff;
      bestScale = niceScales[i];
    }
  }
  scaleMeters = bestScale;
  
  // 根据规范化后的距离，重新计算像素长度
  double scalePixelLength = scaleMeters * ppm;
  
  // 确保线段不会超出屏幕右侧边界
  int maxAllowedLength = screenWidth - x - 10; // 留出10像素的边距
  if (scalePixelLength > maxAllowedLength) {
    scalePixelLength = maxAllowedLength;
    // 重新计算对应的距离
    scaleMeters = scalePixelLength / ppm;
    // 再次规范化
    minDiff = fabs(scaleMeters - niceScales[0]);
    bestScale = niceScales[0];
    for (int i = 1; i < niceScaleCount; i++) {
      double diff = fabs(scaleMeters - niceScales[i]);
      if (diff < minDiff) {
        minDiff = diff;
        bestScale = niceScales[i];
      }
    }
    scaleMeters = bestScale;
    scalePixelLength = scaleMeters * ppm;
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

void RenderEngine::draw3DScaleBar() {
  // 绘制位置（右下角）
  const int MARGIN = 5;
  const int SCALE_LENGTH = 35;
  int centerX = screenWidth - MARGIN - 5;
  int centerY = screenHeight - MARGIN - 10;
  
  canvas->setTextColor(TFT_BLACK);
  canvas->setTextSize(1);
  
  // 计算水平比例尺（地面网格）
  double metersPerPixel = 1000.0 / scaleFactor;
  
  // 规范化比例尺距离
  double niceScales[] = {1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000};
  int niceScaleCount = 13;
  
  double targetMeters = SCALE_LENGTH * metersPerPixel;
  double bestScale = niceScales[0];
  double minDiff = abs(targetMeters - niceScales[0]);
  
  for (int i = 1; i < niceScaleCount; i++) {
    double diff = abs(targetMeters - niceScales[i]);
    if (diff < minDiff) {
      minDiff = diff;
      bestScale = niceScales[i];
    }
  }
  
  double hScaleMeters = bestScale;
  double hScalePixels = hScaleMeters / metersPerPixel;
  
  // 绘制水平比例尺（向左延伸）
  canvas->drawLine(centerX, centerY, centerX - (int)hScalePixels, centerY, TFT_BLACK);
  canvas->drawLine(centerX, centerY - 3, centerX, centerY + 3, TFT_BLACK);
  canvas->drawLine(centerX - (int)hScalePixels, centerY - 3, centerX - (int)hScalePixels, centerY + 3, TFT_BLACK);
  
  // 水平比例尺文本（显示在比例尺下方）
  int hTextX = centerX - (int)(hScalePixels / 2) - 10;
  canvas->setCursor(hTextX, centerY + 5);
  if (hScaleMeters >= 1000) {
    canvas->printf("%.0fkm", hScaleMeters / 1000.0);
  } else {
    canvas->printf("%.0fm", hScaleMeters);
  }
  
  // 绘制垂直比例尺（向上延伸）
  double vScaleMeters = 100.0;
  double vScalePixels = vScaleMeters * 0.001 * verticalExaggeration * scaleFactor;
  
  while (vScalePixels > SCALE_LENGTH * 1.5 && vScaleMeters > 10) {
    vScaleMeters /= 2;
    vScalePixels = vScaleMeters * 0.001 * verticalExaggeration * scaleFactor;
  }
  while (vScalePixels < SCALE_LENGTH * 0.5 && vScaleMeters < 10000) {
    vScaleMeters *= 2;
    vScalePixels = vScaleMeters * 0.001 * verticalExaggeration * scaleFactor;
  }
  
  if (vScalePixels > SCALE_LENGTH * 2) {
    vScalePixels = SCALE_LENGTH * 2;
  }
  
  canvas->drawLine(centerX, centerY, centerX, centerY - (int)vScalePixels, TFT_BLACK);
  canvas->drawLine(centerX - 3, centerY, centerX + 3, centerY, TFT_BLACK);
  canvas->drawLine(centerX - 3, centerY - (int)vScalePixels, centerX + 3, centerY - (int)vScalePixels, TFT_BLACK);
  
  // 垂直比例尺文本（显示在比例尺左侧，避免超出右边界）
  canvas->setCursor(centerX - 30, centerY - (int)(vScalePixels / 2) - 3);
  canvas->printf("%.0fm", vScaleMeters);
  
  // 显示垂直放大系数（显示在顶部文本左侧）
  canvas->setCursor(centerX - 20, centerY - (int)vScalePixels - 10);
  canvas->printf("x%.1f", verticalExaggeration);
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

// 3D渲染相关方法实现
void RenderEngine::setViewMode(ViewMode mode) {
  if (viewMode == mode) return;
  
  if (mode == MODE_2D) {
    releaseWorldPoints();
  } else {
    hasReferenceOrientation = false;
    pitch = 0.0;
    roll = 0.0;
    targetPitch = 0.0;
    targetRoll = 0.0;
    viewOffsetX = 0.0;
    viewOffsetY = 0.0;
    targetViewOffsetX = 0.0;
    targetViewOffsetY = 0.0;
  }
  viewMode = mode;
}

void RenderEngine::setVerticalExaggeration(float value) {
  float newValue = constrain(value, 1.0f, 8.0f);
  if (newValue != verticalExaggeration) {
    verticalExaggeration = newValue;
    invalidateWorldPoints();
  }
}

void RenderEngine::setCameraDistance(float distance) {
  cameraDistance = distance;
}

void RenderEngine::setScaleFactor(float scale) {
  scaleFactor = scale;
}

void RenderEngine::setOrientation(float pitchAngle, float rollAngle) {
  pitch = pitchAngle;
  roll = rollAngle;
}

void RenderEngine::setReferenceOrientation(float pitchAngle, float rollAngle) {
  refPitch = pitchAngle;
  refRoll = rollAngle;
  hasReferenceOrientation = true;
  Serial.printf("[3D Camera] Reference orientation set: Pitch=%.1f°, Roll=%.1f°\n",
                refPitch * 180.0f / (float)M_PI, refRoll * 180.0f / (float)M_PI);
}

void RenderEngine::updateCameraOrientation(float currentPitch, float currentRoll, float accelX, float accelY) {
  if (!hasReferenceOrientation) {
    setReferenceOrientation(currentPitch, currentRoll);
    return;
  }
  
  float deltaPitch = currentPitch - refPitch;
  float deltaRoll = currentRoll - refRoll;
  
  const float MAX_ANGLE = 90.0f * (float)M_PI / 180.0f;
  deltaPitch = constrain(deltaPitch, -MAX_ANGLE, MAX_ANGLE);
  deltaRoll = constrain(deltaRoll, -MAX_ANGLE, MAX_ANGLE);
  deltaRoll = -deltaRoll;
  
  // 死区滤波与平滑：微弱的手部生理震颤（小于1.1度）不驱动角度晃动，抑制高频抖动
  const float DEADBAND = 0.02f; // ~1.15度
  if (fabs(deltaPitch - targetPitch) > DEADBAND) {
    targetPitch = deltaPitch;
  }
  if (fabs(deltaRoll - targetRoll) > DEADBAND) {
    targetRoll = deltaRoll;
  }
  
  const float SMOOTH_FACTOR = 0.12f;
  pitch += (targetPitch - pitch) * SMOOTH_FACTOR;
  roll += (targetRoll - roll) * SMOOTH_FACTOR;
  
  // 大幅降低微小加速度对视口的晃动拉扯（VIEW_SHIFT_SCALE 由 15.0f 降为 2.0f）
  const float VIEW_SHIFT_SCALE = 2.0f;
  targetViewOffsetX = -accelY * VIEW_SHIFT_SCALE;
  targetViewOffsetY = accelX * VIEW_SHIFT_SCALE;
  
  const float OFFSET_SMOOTH_FACTOR = 0.05f;
  viewOffsetX += (targetViewOffsetX - viewOffsetX) * OFFSET_SMOOTH_FACTOR;
  viewOffsetY += (targetViewOffsetY - viewOffsetY) * OFFSET_SMOOTH_FACTOR;
}

void RenderEngine::increaseVerticalExaggeration() {
  setVerticalExaggeration(verticalExaggeration + 0.5f);
}

void RenderEngine::decreaseVerticalExaggeration() {
  setVerticalExaggeration(verticalExaggeration - 0.5f);
}

void RenderEngine::zoom3D(float factor) {
  targetScaleFactor *= factor;
  if (targetScaleFactor < 0.05f) targetScaleFactor = 0.05f;
  if (targetScaleFactor > 5000.0f) targetScaleFactor = 5000.0f;
  userScaleFactor = true;
  Serial.printf("[3D Zoom] Target ScaleFactor: %.4f\n", targetScaleFactor);
}

void RenderEngine::pan3D(int dx, int dy) {
  targetPan3DX += dx;
  targetPan3DY += dy;
}

void RenderEngine::center3DOnLocation(const Location& loc) {
  if (!loc.isValid) return;

  if (cachedLat0 == 0.0 && cachedLon0 == 0.0) {
    cachedLat0 = loc.latitude;
    cachedLon0 = loc.longitude;
    cachedAlt0 = loc.altitude;
    cosLat0 = cosf((float)cachedLat0 * PI / 180.0f);
  }

  // 计算旋转中心
  float centerX = 0.0f, centerY = 0.0f;
  if (useCenterRotation) {
    centerX = (minWorldX + maxWorldX) / 2.0f;
    centerY = (minWorldY + maxWorldY) / 2.0f;
  }

  float cosP = cosf((float)pitch);
  float sinP = sinf((float)pitch);
  float cosR = cosf((float)roll);
  float sinR = sinf((float)roll);

  float alpha = 19.47f * PI / 180.0f;
  float gamma = 20.7f * PI / 180.0f;
  float sinA = sinf(alpha);
  float cosA = cosf(alpha);
  float sinG = sinf(gamma);
  float cosG = cosf(gamma);

  const float EARTH_RADIUS_KM = 6378.137f;
  const float DEG2RAD = PI / 180.0f;

  // 转换为公里并应用中心偏移
  float dLon = (loc.longitude - cachedLon0) * DEG2RAD;
  float dLat = (loc.latitude - cachedLat0) * DEG2RAD;

  float wx = dLon * cosLat0 * EARTH_RADIUS_KM - centerX;
  float wy = -dLat * EARTH_RADIUS_KM - centerY;
  float wz = (loc.altitude - cachedAlt0) * 0.001f * verticalExaggeration;

  // 3D 旋转变换
  float rx = wx * cosR + wz * sinR;
  float rz = -wx * sinR + wz * cosR;
  float ry = wy * cosP - rz * sinP;
  rz = wy * sinP + rz * cosP;

  // 应用投影变换（使用当前缩放）
  float sx = rx * (float)scaleFactor;
  float sy = ry * (float)scaleFactor;
  float sz = rz * (float)scaleFactor;

  float x2d = (sx * cosG) - (sy * sinG);
  float y2d = -(sx * sinG * sinA) - (sy * cosG * sinA) + (sz * cosA);

  // 保持在屏幕中心的目标平移值（通过阻尼插值趋近，消除GPS微小杂讯抖动）
  targetPan3DX = -x2d;
  targetPan3DY = y2d;
}

void RenderEngine::toggleRotationCenter() {
  useCenterRotation = !useCenterRotation;
  Serial.printf("[3D] Rotation center: %s\n", useCenterRotation ? "Grid center" : "Start point");
}

void RenderEngine::reset3DView() {
  pan3DX = targetPan3DX = 0.0f;
  pan3DY = targetPan3DY = 0.0f;
  scaleFactor = targetScaleFactor = 1.0f;
  userScaleFactor = false;
  Serial.println("[3D] View reset");
}

bool RenderEngine::update3DCameraTransition() {
  bool changing = false;
  
  // 缩放平滑阻尼插值 (Lerp)
  float scaleDiff = targetScaleFactor - scaleFactor;
  if (fabs(scaleDiff) > 0.002f * targetScaleFactor) {
    scaleFactor += scaleDiff * 0.25f;
    changing = true;
  } else if (scaleFactor != targetScaleFactor) {
    scaleFactor = targetScaleFactor;
    changing = true;
  }
  
  // 平移平滑阻尼插值 (Lerp) - 吸收GPS定位杂讯与按键平移跳变
  float panXDiff = targetPan3DX - pan3DX;
  if (fabs(panXDiff) > 0.15f) {
    pan3DX += panXDiff * 0.25f;
    changing = true;
  } else if (pan3DX != targetPan3DX) {
    pan3DX = targetPan3DX;
    changing = true;
  }
  
  float panYDiff = targetPan3DY - pan3DY;
  if (fabs(panYDiff) > 0.15f) {
    pan3DY += panYDiff * 0.25f;
    changing = true;
  } else if (pan3DY != targetPan3DY) {
    pan3DY = targetPan3DY;
    changing = true;
  }
  
  return changing;
}

void RenderEngine::buildWorldPoints(const Location* pointPool, int pointCount) {
  releaseWorldPoints();
  
  if (!pointPool || pointCount == 0) return;
  
  // 记录开始分配前的堆内存
  Serial.printf("[3D] Before allocation: %u bytes free\n", esp_get_free_heap_size());
  
  // 限制 3D 视图使用的点数。3000 点对于 3D 来说已经足够，且能腾出 RAM 用于渲染。
  int targetCount = (pointCount > MAX_WORLD_POINTS) ? MAX_WORLD_POINTS : pointCount;
  
  worldPoints = new (std::nothrow) WorldPoint[targetCount];
  if (!worldPoints) {
    Serial.printf("[3D] Memory allocation failed for %d worldPoints! Free: %u\n", targetCount, esp_get_free_heap_size());
    return;
  }
  
  cachedLat0 = pointPool[0].latitude;
  cachedLon0 = pointPool[0].longitude;
  cachedAlt0 = pointPool[0].altitude;
  cosLat0 = cosf((float)cachedLat0 * PI / 180.0f);
  
  const float EARTH_RADIUS_KM = 6378.137f;
  const float DEG2RAD = PI / 180.0f;
  
  worldPointCount = 0;
  minWorldX = 1e9f;  maxWorldX = -1e9f;
  minWorldY = 1e9f;  maxWorldY = -1e9f;
  minWorldZ = 1e9f;  maxWorldZ = -1e9f;
  // 初始化仪表盘配置
  // 如果原始点数超过目标，进行均匀采样
  float sampleRate = (float)pointCount / targetCount;
  
  for (int i = 0; i < targetCount; i++) {
    int srcIdx = (int)(i * sampleRate);
    if (srcIdx >= pointCount) srcIdx = pointCount - 1;
    
    float dLon = (pointPool[srcIdx].longitude - cachedLon0) * DEG2RAD;
    float dLat = (pointPool[srcIdx].latitude - cachedLat0) * DEG2RAD;
    
    float wx = dLon * cosLat0 * EARTH_RADIUS_KM;
    float wy = -dLat * EARTH_RADIUS_KM;
    float wz = (pointPool[srcIdx].altitude - cachedAlt0) * 0.001f * (float)verticalExaggeration;
    
    worldPoints[i].x = wx;
    worldPoints[i].y = wy;
    worldPoints[i].z = wz;
    
    if (wx < minWorldX) minWorldX = wx;
    if (wx > maxWorldX) maxWorldX = wx;
    if (wy < minWorldY) minWorldY = wy;
    if (wy > maxWorldY) maxWorldY = wy;
    
    worldPointCount++;
  }
  
  // 在采样后进行 Douglas-Peucker 进一步精简（可选，因为已经均匀采样了）
  // simplifyPathDouglasPeucker(worldPoints, worldPointCount, 0.002f);
  
  Serial.printf("[3D] Built %d points. After allocation: %u bytes free\n", worldPointCount, esp_get_free_heap_size());
  
  // 在构建线段索引之前进行 Douglas-Peucker 简化
  // 使用 epsilon = 2.0米 (0.002km) 作为默认阈值
  int originalCount = worldPointCount;
  simplifyPathDouglasPeucker(worldPoints, worldPointCount, 0.002f);
  
  // 计算线段数量
  segmentCount = worldPointCount - 1;
  if (segmentCount > 0) {
    segments = new (std::nothrow) SegmentRef[segmentCount];
    if (!segments) {
      Serial.printf("[3D] Memory allocation failed for %d segments! Free: %u\n", segmentCount, esp_get_free_heap_size());
      segmentCount = 0; // Allocation failed, so no segments
      return;
    }
    
    for (int i = 0; i < segmentCount; i++) {
      segments[i].i1 = i;
      segments[i].i2 = i + 1;
      segments[i].depth = 0;
    }
  } else {
    segments = nullptr; // No segments to allocate
  }

  // 分配顶点单次投影缓存数组
  if (worldPointCount > 0) {
    projectedVertices = new (std::nothrow) ProjectedVertex[worldPointCount];
    if (!projectedVertices) {
      Serial.printf("[3D] Memory allocation failed for %d projectedVertices! Free: %u\n", worldPointCount, esp_get_free_heap_size());
    }
  } else {
    projectedVertices = nullptr;
  }
  
  cachedPointPool = pointPool;
  cachedPointCount = pointCount;
  
  if (!userScaleFactor) {
    float worldWidth = maxWorldX - minWorldX;
    float worldHeight = maxWorldY - minWorldY;
    float maxExtent = max(worldWidth, worldHeight);
    
    if (maxExtent > 0.001f) {
      float screenExtent = min(screenWidth, screenHeight) * 0.85f;
      scaleFactor = screenExtent / maxExtent;
      
      if (scaleFactor < 0.05f) scaleFactor = 0.05f; // 支持长达上百公里的超级大跨度
      if (scaleFactor > 5000.0f) scaleFactor = 5000.0f;
      targetScaleFactor = scaleFactor; // 同步目标缩放
      
      Serial.printf("[3D World] Auto scale: %.2f (extent: %.3f km)\n", scaleFactor, maxExtent);
    }
  } else {
    targetScaleFactor = scaleFactor;
  }
  
  Serial.printf("[3D World] Built %d points from %d original (DP simplified, removed %d)\n", 
                worldPointCount, pointCount, originalCount - worldPointCount);
  Serial.printf("[3D World] X: %.3f to %.3f km, Y: %.3f to %.3f km\n",
                minWorldX, maxWorldX, minWorldY, maxWorldY);
}

void RenderEngine::releaseWorldPoints() {
  if (worldPoints) {
    delete[] worldPoints;
    worldPoints = nullptr;
    Serial.println("[3D] worldPoints released");
  }
  if (segments) {
    delete[] segments;
    segments = nullptr;
    Serial.println("[3D] segments released");
  }
  if (projectedVertices) {
    delete[] projectedVertices;
    projectedVertices = nullptr;
    Serial.println("[3D] projectedVertices released");
  }
  worldPointCount = 0;
  segmentCount = 0;
  minWorldX = 0.0f; maxWorldX = 0.0f;
  minWorldY = 0.0f; maxWorldY = 0.0f;
  minWorldZ = 0.0f; maxWorldZ = 0.0f;
  cachedLat0 = 0.0; cachedLon0 = 0.0; cachedAlt0 = 0.0;
}

void RenderEngine::invalidateWorldPoints() {
  releaseWorldPoints();
  reset3DView();
}

// 3D渲染核心方法实现
void RenderEngine::render3D(const std::vector<Location>& routePoints, const Location& currentLocation, const std::vector<Location>& trackPoints, bool sdInitialized, bool hasRoute, const Location* pointPool, int pointCount, const POI* poiPool, int poiCount, int showPOIsMode) {
  if (!canvas) return;
  
  if (worldPointCount == 0 || worldPoints == nullptr) {
    if (pointPool && pointCount > 0) {
      buildWorldPoints(pointPool, pointCount);
    } else if (!routePoints.empty()) {
      buildWorldPoints(routePoints.data(), routePoints.size());
    }
  }
  
  if (worldPointCount == 0) {
    canvas->fillScreen(TFT_WHITE);
    canvas->setTextColor(TFT_BLACK);
    canvas->setCursor(10, 10);
    canvas->println("No route data");
    canvas->setCursor(10, 25);
    canvas->println("Please load a KML file");
    return;
  }
  
  // 更新相机阻尼平滑插值（缩放、平移等）
  update3DCameraTransition();

  // 如果处于定位锁定跟随状态且定位有效，自动居中3D视角到当前定位点（平滑对齐目标）
  if (isLocationLocked && currentLocation.isValid) {
    center3DOnLocation(currentLocation);
    update3DCameraTransition();
  }

  canvas->fillScreen(TFT_WHITE);
  canvas->setTextColor(TFT_BLACK);
  canvas->setTextSize(1);
  
  float cosP = cosf((float)pitch);
  float sinP = sinf((float)pitch);
  float cosR = cosf((float)roll);
  float sinR = sinf((float)roll);
  
  float alpha = 19.47f * PI / 180.0f;
  float gamma = 20.7f * PI / 180.0f;
  float sinA = sinf(alpha);
  float cosA = cosf(alpha);
  float sinG = sinf(gamma);
  float cosG = cosf(gamma);
  
  float centerX = 0.0f, centerY = 0.0f;
  if (useCenterRotation) {
    centerX = (minWorldX + maxWorldX) / 2.0f;
    centerY = (minWorldY + maxWorldY) / 2.0f;
  }
  
  // 1. 顶点单次投影变换与缓存（Single-Pass Vertex Transform Cache）
  // 彻底消除反复对同一端点进行重复 3D 旋转与投影的计算
  float minZ = 1e9f, maxZ = -1e9f;
  for (int i = 0; i < worldPointCount; i++) {
    float wx = worldPoints[i].x - centerX;
    float wy = worldPoints[i].y - centerY;
    float wz = worldPoints[i].z;
    
    float rx = wx * cosR + wz * sinR;
    float rz = -wx * sinR + wz * cosR;
    float ry = wy * cosP - rz * sinP;
    rz = wy * sinP + rz * cosP;
    
    if (rz < minZ) minZ = rz;
    if (rz > maxZ) maxZ = rz;
    
    float sx = rx * (float)scaleFactor;
    float sy = ry * (float)scaleFactor;
    float sz = rz * (float)scaleFactor;
    
    float x2d = (sx * cosG) - (sy * sinG);
    float y2d = -(sx * sinG * sinA) - (sy * cosG * sinA) + (sz * cosA);
    
    if (projectedVertices) {
      projectedVertices[i].screenX = (int16_t)(x2d + screenWidth / 2 + pan3DX);
      projectedVertices[i].screenY = (int16_t)(screenHeight / 2 - y2d + pan3DY);
      projectedVertices[i].rz = rz;
    }
  }
  float zRange = maxZ - minZ;
  if (zRange < 0.001f) zRange = 1.0f;
  
  // 2. 线段深度计算（直接复用顶点缓存的 rz，零三角计算）
  if (projectedVertices) {
    for (int i = 0; i < segmentCount; i++) {
      int i1 = segments[i].i1;
      int i2 = segments[i].i2;
      segments[i].depth = (projectedVertices[i1].rz + projectedVertices[i2].rz) * 0.5f;
    }
  }
  
  // 3. 线段深度排序：使用 O(N log N) std::sort 代替旧有的 O(N^2) 冒泡排序
  if (segments && segmentCount > 1) {
    std::sort(segments, segments + segmentCount, [](const SegmentRef& a, const SegmentRef& b) {
      return a.depth > b.depth;
    });
  }
  
  // 4. 线段绘制：直接使用顶点投影缓存的屏幕坐标
  for (int i = 0; i < segmentCount; i++) {
    int i1 = segments[i].i1;
    int i2 = segments[i].i2;
    
    int screenX1 = projectedVertices ? projectedVertices[i1].screenX : 0;
    int screenY1 = projectedVertices ? projectedVertices[i1].screenY : 0;
    int screenX2 = projectedVertices ? projectedVertices[i2].screenX : 0;
    int screenY2 = projectedVertices ? projectedVertices[i2].screenY : 0;
    
    // 快速视口粗筛：如果线段完全在屏幕外较远处，跳过底层绘制
    if ((screenX1 < -50 && screenX2 < -50) || 
        (screenX1 > screenWidth + 50 && screenX2 > screenWidth + 50) ||
        (screenY1 < -50 && screenY2 < -50) || 
        (screenY1 > screenHeight + 50 && screenY2 > screenHeight + 50)) {
      continue;
    }
    
    float depthNorm = (segments[i].depth - minZ) / zRange;
    if (depthNorm < 0.0f) depthNorm = 0.0f;
    if (depthNorm > 1.0f) depthNorm = 1.0f;
    
    float brightness = 1.0f - depthNorm * 0.7f;
    
    uint8_t r = (uint8_t)(0 * brightness);
    uint8_t g = (uint8_t)(0 * brightness);
    uint8_t b = (uint8_t)(255 * brightness);
    
    float fogFactor = depthNorm * 0.4f;
    r = (uint8_t)(r * (1.0f - fogFactor) + 255 * fogFactor);
    g = (uint8_t)(g * (1.0f - fogFactor) + 255 * fogFactor);
    b = (uint8_t)(b * (1.0f - fogFactor) + 255 * fogFactor);
    
    uint16_t color = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
    
    canvas->drawLine(screenX1, screenY1, screenX2, screenY2, color);
  }
  
  // 5. 绘制起点与终点标记（直接复用顶点投影缓存）
  if (worldPointCount > 0 && projectedVertices) {
    int startScreenX = projectedVertices[0].screenX;
    int startScreenY = projectedVertices[0].screenY;
    if (startScreenX > -10 && startScreenX < screenWidth + 10 && startScreenY > -10 && startScreenY < screenHeight + 10) {
      canvas->fillCircle(startScreenX, startScreenY, 3, TFT_GREEN);
    }
    
    int endScreenX = projectedVertices[worldPointCount - 1].screenX;
    int endScreenY = projectedVertices[worldPointCount - 1].screenY;
    if (endScreenX > -10 && endScreenX < screenWidth + 10 && endScreenY > -10 && endScreenY < screenHeight + 10) {
      canvas->fillCircle(endScreenX, endScreenY, 3, TFT_RED);
    }
  }
  
  draw3DGroundPlane(cosP, sinP, cosR, sinR, sinA, cosA, sinG, cosG);
  
  draw3DGroundProjection(cosP, sinP, cosR, sinR, sinA, cosA, sinG, cosG, minZ, zRange, centerX, centerY);
  
  draw3DVerticalLines(cosP, sinP, cosR, sinR, sinA, cosA, sinG, cosG, minZ, zRange, centerX, centerY);
  
  // 绘制 3D 关键点
  if (showPOIsMode != 0 && poiPool && poiCount > 0) {
    const float EARTH_RADIUS_KM = 6378.137f;
    const float DEG2RAD = PI / 180.0f;
    
    // 自动模式过滤
    int prevIdx = -1;
    int nextIdx = -1;
    if (showPOIsMode == 2 && currentLocation.isValid) {
        const Location* dataPoints = (pointPool != nullptr && pointCount > 0) ? pointPool : routePoints.data();
        int dataPointCount = (pointPool != nullptr && pointCount > 0) ? pointCount : routePoints.size();
        if (dataPointCount < 2 && trackPoints.size() >= 2) {
            dataPoints = trackPoints.data();
            dataPointCount = trackPoints.size();
        }
        
        if (dataPointCount >= 2) {
            // 更新缓存
            if (lastPoiPoolPtr != poiPool || lastPoiCount != poiCount || lastPointPoolPtr != dataPoints || lastPointCount != dataPointCount) {
                poiProgressCache.clear();
                for (int i = 0; i < poiCount; i++) {
                    double p, d;
                    findClosestSegment(dataPoints, dataPointCount, poiPool[i].loc, p, d);
                    poiProgressCache.push_back(p);
                }
                lastPoiPoolPtr = poiPool;
                lastPoiCount = poiCount;
                lastPointPoolPtr = dataPoints;
                lastPointCount = dataPointCount;
            }

            double userProgress, userDist;
            if (findClosestSegment(dataPoints, dataPointCount, currentLocation, userProgress, userDist)) {
                double maxPrevProg = -1.0;
                double minNextProg = 2.0;

                for (int i = 0; i < (int)poiProgressCache.size(); i++) {
                    double pProg = poiProgressCache[i];
                    if (pProg <= userProgress) {
                        if (pProg > maxPrevProg) {
                            maxPrevProg = pProg;
                            prevIdx = i;
                        }
                    } else {
                        if (pProg < minNextProg) {
                            minNextProg = pProg;
                            nextIdx = i;
                        }
                    }
                }
            }
        } else {
            // 无参考轨迹线时：按与当前位置直线距离最近筛选 1~2 个 POI
            float minDist1 = 1e9f;
            float minDist2 = 1e9f;
            for (int i = 0; i < poiCount; i++) {
                float d = calculateDistance(currentLocation, poiPool[i].loc);
                if (d < minDist1) {
                    minDist2 = minDist1;
                    nextIdx = prevIdx;
                    minDist1 = d;
                    prevIdx = i;
                } else if (d < minDist2) {
                    minDist2 = d;
                    nextIdx = i;
                }
            }
        }
    }

    for (int i = 0; i < poiCount; i++) {
        // 如果是自动模式且不属于选中的两个，跳过
        if (showPOIsMode == 2 && i != prevIdx && i != nextIdx) continue;
        
        float dLon = (poiPool[i].loc.longitude - cachedLon0) * DEG2RAD;
        float dLat = (poiPool[i].loc.latitude - cachedLat0) * DEG2RAD;
        
        float wx = dLon * cosLat0 * EARTH_RADIUS_KM - centerX;
        float wy = -dLat * EARTH_RADIUS_KM - centerY;
        float wz = (poiPool[i].loc.altitude - cachedAlt0) * 0.001f * verticalExaggeration;
        
        float rx = wx * cosR + wz * sinR;
        float rz = -wx * sinR + wz * cosR;
        float ry = wy * cosP - rz * sinP;
        rz = wy * sinP + rz * cosP;
        
        float sx = rx * (float)scaleFactor;
        float sy = ry * (float)scaleFactor;
        float sz = rz * (float)scaleFactor;
        
        float x2d = (sx * cosG) - (sy * sinG);
        float y2d = -(sx * sinG * sinA) - (sy * cosG * sinA) + (sz * cosA);
        
        int screenX = (int)(x2d + screenWidth / 2 + pan3DX);
        int screenY = (int)(screenHeight / 2 - y2d + pan3DY);
        
        if (screenX > 0 && screenX < screenWidth && screenY > 0 && screenY < screenHeight) {
            // 画绿色小旗帜（三角形）
            canvas->fillTriangle(screenX, screenY, screenX - 4, screenY - 8, screenX + 4, screenY - 8, TFT_GREEN);
            canvas->drawLine(screenX, screenY, screenX, screenY - 8, TFT_BLACK);
            
            if (poiPool[i].name != "") {
                canvas->setFont(&fonts::efontCN_12);
                canvas->setTextColor(TFT_BLACK);
                canvas->drawCenterString(poiPool[i].name, screenX, screenY - 20);
            }
        }
    }
    canvas->setFont(&fonts::Font0);
  }
  
  // 绘制 3D 当前位置
  draw3DCurrentLocation(currentLocation);
  
  draw3DUIInfo();
}

// 绘制3D地面网格
void RenderEngine::draw3DGroundPlane(float cosP, float sinP, float cosR, float sinR, float sinA, float cosA, float sinG, float cosG) {
  if (!canvas || worldPointCount == 0) return;
  
  float routeExtent = max(maxWorldX - minWorldX, maxWorldY - minWorldY);
  // 网格略微大于路径跨度（外留约10%边距），充满度约90%
  float gridExtent = routeExtent * 0.55f;
  if (gridExtent < 0.05f) gridExtent = 0.05f; // 支持最小50米的小跨度路径
  
  float gridCenterX = (minWorldX + maxWorldX) / 2.0f;
  float gridCenterY = (minWorldY + maxWorldY) / 2.0f;
  
  float rotCenterX = useCenterRotation ? gridCenterX : 0.0f;
  float rotCenterY = useCenterRotation ? gridCenterY : 0.0f;
  
  int gridLines = 15;
  float gridStep = gridExtent * 2.0f / (gridLines - 1);
  
  for (int i = 0; i < gridLines; i++) {
    float x = gridCenterX - gridExtent + i * gridStep;
    
    float wx1 = x - rotCenterX, wy1 = gridCenterY - gridExtent - rotCenterY, wz1 = 0;
    float wx2 = x - rotCenterX, wy2 = gridCenterY + gridExtent - rotCenterY, wz2 = 0;
    
    float rx1 = wx1 * cosR + wz1 * sinR;
    float rz1 = -wx1 * sinR + wz1 * cosR;
    float ry1 = wy1 * cosP - rz1 * sinP;
    rz1 = wy1 * sinP + rz1 * cosP;
    
    float rx2 = wx2 * cosR + wz2 * sinR;
    float rz2 = -wx2 * sinR + wz2 * cosR;
    float ry2 = wy2 * cosP - rz2 * sinP;
    rz2 = wy2 * sinP + rz2 * cosP;
    
    float sx1 = rx1 * (float)scaleFactor;
    float sy1 = ry1 * (float)scaleFactor;
    float sz1 = rz1 * (float)scaleFactor;
    float sx2 = rx2 * (float)scaleFactor;
    float sy2 = ry2 * (float)scaleFactor;
    float sz2 = rz2 * (float)scaleFactor;
    
    float x2d1 = (sx1 * cosG) - (sy1 * sinG);
    float y2d1 = -(sx1 * sinG * sinA) - (sy1 * cosG * sinA) + (sz1 * cosA);
    float x2d2 = (sx2 * cosG) - (sy2 * sinG);
    float y2d2 = -(sx2 * sinG * sinA) - (sy2 * cosG * sinA) + (sz2 * cosA);
    
    int screenX1 = (int)(x2d1 + screenWidth / 2 + pan3DX);
    int screenY1 = (int)(screenHeight / 2 - y2d1 + pan3DY);
    int screenX2 = (int)(x2d2 + screenWidth / 2 + pan3DX);
    int screenY2 = (int)(screenHeight / 2 - y2d2 + pan3DY);
    
    int margin = max(screenWidth, screenHeight);
    bool xOutside = (screenX1 < -margin && screenX2 < -margin) || (screenX1 > screenWidth + margin && screenX2 > screenWidth + margin);
    bool yOutside = (screenY1 < -margin && screenY2 < -margin) || (screenY1 > screenHeight + margin && screenY2 > screenHeight + margin);
    if (!xOutside && !yOutside) {
      canvas->drawLine(screenX1, screenY1, screenX2, screenY2, TFT_LIGHTGRAY);
    }
  }
  
  for (int i = 0; i < gridLines; i++) {
    float y = gridCenterY - gridExtent + i * gridStep;
    
    float wx1 = gridCenterX - gridExtent - rotCenterX, wy1 = y - rotCenterY, wz1 = 0;
    float wx2 = gridCenterX + gridExtent - rotCenterX, wy2 = y - rotCenterY, wz2 = 0;
    
    float rx1 = wx1 * cosR + wz1 * sinR;
    float rz1 = -wx1 * sinR + wz1 * cosR;
    float ry1 = wy1 * cosP - rz1 * sinP;
    rz1 = wy1 * sinP + rz1 * cosP;
    
    float rx2 = wx2 * cosR + wz2 * sinR;
    float rz2 = -wx2 * sinR + wz2 * cosR;
    float ry2 = wy2 * cosP - rz2 * sinP;
    rz2 = wy2 * sinP + rz2 * cosP;
    
    float sx1 = rx1 * (float)scaleFactor;
    float sy1 = ry1 * (float)scaleFactor;
    float sz1 = rz1 * (float)scaleFactor;
    float sx2 = rx2 * (float)scaleFactor;
    float sy2 = ry2 * (float)scaleFactor;
    float sz2 = rz2 * (float)scaleFactor;
    
    float x2d1 = (sx1 * cosG) - (sy1 * sinG);
    float y2d1 = -(sx1 * sinG * sinA) - (sy1 * cosG * sinA) + (sz1 * cosA);
    float x2d2 = (sx2 * cosG) - (sy2 * sinG);
    float y2d2 = -(sx2 * sinG * sinA) - (sy2 * cosG * sinA) + (sz2 * cosA);
    
    int screenX1 = (int)(x2d1 + screenWidth / 2 + pan3DX);
    int screenY1 = (int)(screenHeight / 2 - y2d1 + pan3DY);
    int screenX2 = (int)(x2d2 + screenWidth / 2 + pan3DX);
    int screenY2 = (int)(screenHeight / 2 - y2d2 + pan3DY);
    
    int margin = max(screenWidth, screenHeight);
    bool xOutside = (screenX1 < -margin && screenX2 < -margin) || (screenX1 > screenWidth + margin && screenX2 > screenWidth + margin);
    bool yOutside = (screenY1 < -margin && screenY2 < -margin) || (screenY1 > screenHeight + margin && screenY2 > screenHeight + margin);
    if (!xOutside && !yOutside) {
      canvas->drawLine(screenX1, screenY1, screenX2, screenY2, TFT_LIGHTGRAY);
    }
  }
}

void RenderEngine::draw3DGroundProjection(float cosP, float sinP, float cosR, float sinR, float sinA, float cosA, float sinG, float cosG, float minZ, float zRange, float centerX, float centerY) {
  if (!canvas || worldPointCount < 2) return;
  
  for (int i = 0; i < segmentCount; i++) {
    int i1 = segments[i].i1;
    int i2 = segments[i].i2;
    
    float wx1 = worldPoints[i1].x - centerX;
    float wy1 = worldPoints[i1].y - centerY;
    float wx2 = worldPoints[i2].x - centerX;
    float wy2 = worldPoints[i2].y - centerY;
    
    float rx1 = wx1 * cosR;
    float rz1 = -wx1 * sinR;
    float ry1 = wy1 * cosP - rz1 * sinP;
    rz1 = wy1 * sinP + rz1 * cosP;
    
    float rx2 = wx2 * cosR;
    float rz2 = -wx2 * sinR;
    float ry2 = wy2 * cosP - rz2 * sinP;
    rz2 = wy2 * sinP + rz2 * cosP;
    
    float sx1 = rx1 * (float)scaleFactor;
    float sy1 = ry1 * (float)scaleFactor;
    float sz1 = rz1 * (float)scaleFactor;
    float sx2 = rx2 * (float)scaleFactor;
    float sy2 = ry2 * (float)scaleFactor;
    float sz2 = rz2 * (float)scaleFactor;
    
    float x2d1 = (sx1 * cosG) - (sy1 * sinG);
    float y2d1 = -(sx1 * sinG * sinA) - (sy1 * cosG * sinA) + (sz1 * cosA);
    float x2d2 = (sx2 * cosG) - (sy2 * sinG);
    float y2d2 = -(sx2 * sinG * sinA) - (sy2 * cosG * sinA) + (sz2 * cosA);
    
    int screenX1 = (int)(x2d1 + screenWidth / 2 + pan3DX);
    int screenY1 = (int)(screenHeight / 2 - y2d1 + pan3DY);
    int screenX2 = (int)(x2d2 + screenWidth / 2 + pan3DX);
    int screenY2 = (int)(screenHeight / 2 - y2d2 + pan3DY);
    
    if (screenX1 < -50 || screenX1 > screenWidth + 50 ||
        screenY1 < -50 || screenY1 > screenHeight + 50 ||
        screenX2 < -50 || screenX2 > screenWidth + 50 ||
        screenY2 < -50 || screenY2 > screenHeight + 50) {
      continue;
    }
    
    float depthNorm = (segments[i].depth - minZ) / zRange;
    if (depthNorm < 0.0f) depthNorm = 0.0f;
    if (depthNorm > 1.0f) depthNorm = 1.0f;
    
    uint8_t intensity = (uint8_t)(180 + 50 * (1.0f - depthNorm));
    uint16_t color = ((intensity >> 3) << 11) | ((intensity >> 2) << 5) | (intensity >> 3);
    
    canvas->drawLine(screenX1, screenY1, screenX2, screenY2, color);
  }
}

void RenderEngine::draw3DVerticalLines(float cosP, float sinP, float cosR, float sinR, float sinA, float cosA, float sinG, float cosG, float minZ, float zRange, float centerX, float centerY) {
  if (!canvas || worldPointCount == 0) return;
  
  int step = max(1, worldPointCount / 50);
  
  for (int i = 0; i < worldPointCount; i += step) {
    float wx = worldPoints[i].x - centerX;
    float wy = worldPoints[i].y - centerY;
    float wz = worldPoints[i].z;
    
    if (wz < 0.01f) continue;
    
    int screenX = projectedVertices ? projectedVertices[i].screenX : 0;
    int screenY = projectedVertices ? projectedVertices[i].screenY : 0;
    
    float grx = wx * cosR;
    float grz = -wx * sinR;
    float gry = wy * cosP - grz * sinP;
    grz = wy * sinP + grz * cosP;
    
    float gsx = grx * (float)scaleFactor;
    float gsy = gry * (float)scaleFactor;
    float gsz = grz * (float)scaleFactor;
    
    float gx2d = (gsx * cosG) - (gsy * sinG);
    float gy2d = -(gsx * sinG * sinA) - (gsy * cosG * sinA) + (gsz * cosA);
    
    int groundScreenX = (int)(gx2d + screenWidth / 2 + pan3DX);
    int groundScreenY = (int)(screenHeight / 2 - gy2d + pan3DY);
    
    if (screenX > -50 && screenX < screenWidth + 50 && screenY > -50 && screenY < screenHeight + 50) {
      float rz = projectedVertices ? projectedVertices[i].rz : 0.0f;
      float depthNorm = (rz - minZ) / zRange;
      if (depthNorm < 0.0f) depthNorm = 0.0f;
      if (depthNorm > 1.0f) depthNorm = 1.0f;
      
      uint8_t intensity = (uint8_t)(150 + 100 * (1.0f - depthNorm));
      uint16_t color = ((intensity >> 3) << 11) | (((intensity >> 1)) << 5) | (intensity >> 3);
      
      canvas->drawLine(screenX, screenY, groundScreenX, groundScreenY, color);
    }
  }
}

// 绘制3D当前位置
void RenderEngine::draw3DCurrentLocation(const Location& currentLocation) {
  if (!currentLocation.isValid) return;
  if (!canvas) return;
  
  // 计算旋转中心
  float centerX = 0.0f, centerY = 0.0f;
  if (useCenterRotation) {
    centerX = (minWorldX + maxWorldX) / 2.0f;
    centerY = (minWorldY + maxWorldY) / 2.0f;
  }
  
  float cosP = cosf((float)pitch);
  float sinP = sinf((float)pitch);
  float cosR = cosf((float)roll);
  float sinR = sinf((float)roll);
  
  float alpha = 19.47f * PI / 180.0f;
  float gamma = 20.7f * PI / 180.0f;
  float sinA = sinf(alpha);
  float cosA = cosf(alpha);
  float sinG = sinf(gamma);
  float cosG = cosf(gamma);
  
  const float EARTH_RADIUS_KM = 6378.137f;
  const float DEG2RAD = PI / 180.0f;
  
  // 转换为公里并应用中心偏移
  float dLon = (currentLocation.longitude - cachedLon0) * DEG2RAD;
  float dLat = (currentLocation.latitude - cachedLat0) * DEG2RAD;
  
  float wx = dLon * cosLat0 * EARTH_RADIUS_KM - centerX;
  float wy = -dLat * EARTH_RADIUS_KM - centerY;
  float wz = (currentLocation.altitude - cachedAlt0) * 0.001f * verticalExaggeration;
  
  // 3D 旋转变换
  float rx = wx * cosR + wz * sinR;
  float rz = -wx * sinR + wz * cosR;
  float ry = wy * cosP - rz * sinP;
  rz = wy * sinP + rz * cosP;
  
  // 应用投影变换
  float sx = rx * (float)scaleFactor;
  float sy = ry * (float)scaleFactor;
  float sz = rz * (float)scaleFactor;
  
  float x2d = (sx * cosG) - (sy * sinG);
  float y2d = -(sx * sinG * sinA) - (sy * cosG * sinA) + (sz * cosA);
  
  int screenX = (int)(x2d + screenWidth / 2 + pan3DX);
  int screenY = (int)(screenHeight / 2 - y2d + pan3DY);
  
  // 绘制定位标记、视野标志与航速
  if (screenX > -10 && screenX < screenWidth + 10 && screenY > -10 && screenY < screenHeight + 10) {
      // 1. 绘制行进方向渐变蓝色扇形视野锥（末端渐淡）
      if (currentLocation.course >= 0.0f) {
        drawCourseHeadingCone(screenX, screenY, currentLocation.course);
      }

      // 2. 绘制现代地图风格定位标记（白色描边外环 + 鲜艳天蓝圆心）
      canvas->fillCircle(screenX, screenY, 4, TFT_WHITE);
      canvas->fillCircle(screenX, screenY, 3, canvas->color565(30, 136, 229));
      canvas->fillCircle(screenX, screenY, 1, TFT_WHITE);

      // 当航速大于 0.1km/h 时，显示纯黑简约航速文字
      if (currentLocation.speed > 0.1f) {
        String speedStr = (currentLocation.speed < 10.0f) ? (String(currentLocation.speed, 1) + "km/h") : (String((int)roundf(currentLocation.speed)) + "km/h");
        canvas->setFont(&fonts::Font0);
        canvas->setTextSize(1);
        int textW = canvas->textWidth(speedStr);
        int textX = screenX + 6;
        int textY = screenY - 4;
        if (textX + textW > screenWidth - 2) {
          textX = screenX - textW - 6;
        }
        if (textY < 2) {
          textY = screenY + 6;
        }
        canvas->setTextColor(TFT_BLACK);
        canvas->setCursor(textX, textY);
        canvas->print(speedStr);
      }
  }
}

// 绘制3D UI信息
void RenderEngine::draw3DUIInfo() {
  // 绘制3D比例尺（右下角，始终显示）
  draw3DScaleBar();
  
  // 只有当debug抽屉可见时才绘制调试信息
  if (debugPosition <= -100) return;
  
  canvas->setTextColor(TFT_BLACK);
  canvas->setTextSize(1);
  
  int startX = debugPosition;
  int startY = 10;
  int lineHeight = 12;  // 统一行距
  
  // 绘制标题
  canvas->setCursor(startX, startY);
  canvas->println("=== 3D VIEW ===");
  
  // 显示垂直放大系数
  canvas->setCursor(startX, startY + lineHeight);
  canvas->printf("Vert Exag: %.1f", verticalExaggeration);
  
  // 显示当前倾角
  canvas->setCursor(startX, startY + lineHeight * 2);
  canvas->printf("Pitch: %.1f deg", pitch * 180.0 / M_PI);
  
  canvas->setCursor(startX, startY + lineHeight * 3);
  canvas->printf("Roll: %.1f deg", roll * 180.0 / M_PI);
  
  // 显示缩放因子
  canvas->setCursor(startX, startY + lineHeight * 4);
  canvas->printf("Scale: %.2f", scaleFactor);
  
  // 显示旋转中心模式
  canvas->setCursor(startX, startY + lineHeight * 5);
  canvas->printf("Center: %s", useCenterRotation ? "Grid" : "Start");
  
  // 显示剩余堆内存
  canvas->setCursor(startX, startY + lineHeight * 6);
  canvas->printf("Free: %luB", (unsigned long)esp_get_free_heap_size());
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

void RenderEngine::drawPOIs(const POI* poiPool, int poiCount, bool showNames) {
  if (!poiPool || poiCount <= 0 || !canvas) return;
  
  for (int i = 0; i < poiCount; i++) {
    int px, py;
    latLngToScreen(poiPool[i].loc.latitude, poiPool[i].loc.longitude, px, py);
    {
        // 画绿色小旗帜
        canvas->fillTriangle(px, py, px - 4, py - 8, px + 4, py - 8, TFT_GREEN);
        canvas->drawLine(px, py, px, py - 8, TFT_BLACK);
        
        if (showNames && poiPool[i].name != "") {
             canvas->setFont(&fonts::efontCN_12);
             canvas->setTextColor(TFT_BLACK);
             canvas->drawCenterString(poiPool[i].name, px, py - 18);
        }
    }
  }
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

float RenderEngine::pointLineDistance(WorldPoint p, WorldPoint a, WorldPoint b) {
  float dx = b.x - a.x;
  float dy = b.y - a.y;
  if (dx == 0 && dy == 0) {
    return sqrtf((p.x - a.x) * (p.x - a.x) + (p.y - a.y) * (p.y - a.y));
  }
  
  // 点到直线的距离公式：d = |(y2-y1)x0 - (x2-x1)y0 + x2y1 - y2x1| / sqrt((y2-y1)^2 + (x2-x1)^2)
  float area = fabsf(dy * p.x - dx * p.y + b.x * a.y - b.y * a.x);
  return area / sqrtf(dx * dx + dy * dy);
}

void RenderEngine::simplifyPathDouglasPeucker(WorldPoint* points, int& count, float epsilon) {
  if (count <= 2) return;
  
  // 标记保留点
  bool* keep = new (std::nothrow) bool[count];
  if (!keep) return;
  for (int i = 0; i < count; i++) keep[i] = false;
  
  keep[0] = true;
  keep[count - 1] = true;
  
  // 非递归栈使用 std::vector
  std::vector<Range> stack;
  stack.push_back({0, count - 1});
  
  while (!stack.empty()) {
    Range range = stack.back();
    stack.pop_back();
    
    int start = range.start;
    int end = range.end;
    if (end - start <= 1) continue;
    
    float maxDist = 0;
    int index = -1;
    
    for (int i = start + 1; i < end; i++) {
        float dist = pointLineDistance(points[i], points[start], points[end]);
        if (dist > maxDist) {
            maxDist = dist;
            index = i;
        }
    }
    
    if (maxDist > epsilon && index != -1) {
      keep[index] = true;
      stack.push_back({start, index});
      stack.push_back({index, end});
    }
  }
  
  // 重新映射点数组（原地压缩）
  int newCount = 0;
  for (int i = 0; i < count; i++) {
    if (keep[i]) {
      points[newCount++] = points[i];
    }
  }
  count = newCount;
  delete[] keep;
}
void RenderEngine::drawPOIsAuto(const POI* poiPool, int poiCount, const Location& currentLocation, const Location* pointPool, int pointCount) {
  if (!poiPool || poiCount <= 0 || !currentLocation.isValid) return;

  // 如果参考轨迹线点数不足2个（如纯tracking刚开始或无预载路线），回退到按直线距离最近算法选取最近的 1~2 个 POI
  if (!pointPool || pointCount < 2) {
    int closestIdx1 = -1;
    int closestIdx2 = -1;
    float minDist1 = 1e9f;
    float minDist2 = 1e9f;
    for (int i = 0; i < poiCount; i++) {
      float d = calculateDistance(currentLocation, poiPool[i].loc);
      if (d < minDist1) {
        minDist2 = minDist1;
        closestIdx2 = closestIdx1;
        minDist1 = d;
        closestIdx1 = i;
      } else if (d < minDist2) {
        minDist2 = d;
        closestIdx2 = i;
      }
    }
    if (closestIdx1 != -1) {
      POI temp[1] = {poiPool[closestIdx1]};
      drawPOIs(temp, 1, true);
    }
    if (closestIdx2 != -1 && closestIdx2 != closestIdx1) {
      POI temp[1] = {poiPool[closestIdx2]};
      drawPOIs(temp, 1, true);
    }
    return;
  }

  // 检测并更新缓存
  if (lastPoiPoolPtr != poiPool || lastPoiCount != poiCount || lastPointPoolPtr != pointPool || lastPointCount != pointCount) {
      poiProgressCache.clear();
      for (int i = 0; i < poiCount; i++) {
          double progress, dist;
          findClosestSegment(pointPool, pointCount, poiPool[i].loc, progress, dist);
          poiProgressCache.push_back(progress);
      }
      lastPoiPoolPtr = poiPool;
      lastPoiCount = poiCount;
      lastPointPoolPtr = pointPool;
      lastPointCount = pointCount;
      Serial.println("[POI Cache] Updated POI progress cache");
  }

  double userProgress, userDist;
  if (!findClosestSegment(pointPool, pointCount, currentLocation, userProgress, userDist)) return;

  int prevIdx = -1;
  int nextIdx = -1;
  double maxPrevProg = -1.0;
  double minNextProg = 2.0;

  for (int i = 0; i < (int)poiProgressCache.size(); i++) {
      double pProg = poiProgressCache[i];
      if (pProg <= userProgress) {
          if (pProg > maxPrevProg) {
              maxPrevProg = pProg;
              prevIdx = i;
          }
      } else {
          if (pProg < minNextProg) {
              minNextProg = pProg;
              nextIdx = i;
          }
      }
  }

  // 绘制选中的两个 POI
  if (prevIdx != -1) {
      POI temp[1] = {poiPool[prevIdx]};
      drawPOIs(temp, 1, true);
  }
  if (nextIdx != -1 && nextIdx != prevIdx) {
      POI temp[1] = {poiPool[nextIdx]};
      drawPOIs(temp, 1, true);
  }
}
