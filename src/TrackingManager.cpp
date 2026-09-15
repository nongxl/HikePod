#include "TrackingManager.h"
#include <SD.h>
#include <time.h>
#include <sys/time.h>

TrackingManager::TrackingManager() : 
  isTrackingEnabled(false),
  gnssModule(nullptr),
  trackFileName(""),
  tempCoordFileName(""),
  tempCoordFile(),
  trackPoints(),
  waypoints(),
  totalDistance(0.0),
  elevationGain(0.0),
  elevationLoss(0.0),
  trackingStartTimeMs(0),
  beginTimeIso(""),
  beginTimeEpochMs(0),
  lastRecordedPoint(),
  lastRecordTime(0),
  trackPointIndex(0) {
  lastRecordedPoint.isValid = false;
}

void TrackingManager::begin() {
  ensureTracksDirectoryExists();
}

void TrackingManager::setGNSSModule(GNSSModule* gnss) {
  gnssModule = gnss;
}

bool TrackingManager::startTracking(const String& customPrefix) {
  if (isTrackingEnabled) {
    return true; // 已经在跟踪
  }
  
  ensureTracksDirectoryExists();
  
  // 生成标准KML文件名（带自定义前缀与时间戳）
  trackFileName = generateKMLFileName(customPrefix);
  tempCoordFileName = "/HikePod/.temp_track.coord";
  
  // 打开临时坐标存储文件
  if (SD.exists(tempCoordFileName)) {
    SD.remove(tempCoordFileName);
  }
  tempCoordFile = SD.open(tempCoordFileName, FILE_WRITE);
  
  // 重置状态与统计数据
  trackPoints.clear();
  waypoints.clear();
  lastRecordedPoint.isValid = false;
  lastRecordTime = 0;
  trackPointIndex = 0;
  totalDistance = 0.0;
  elevationGain = 0.0;
  elevationLoss = 0.0;
  trackingStartTimeMs = millis();
  beginTimeIso = getIsoTimeString();
  beginTimeEpochMs = getEpochTimeMs();
  
  isTrackingEnabled = true;
  Serial.printf("[Tracking] Started tracking, output KML: %s\n", trackFileName.c_str());
  return true;
}

void TrackingManager::stopTracking() {
  if (!isTrackingEnabled) {
    return;
  }
  
  // 关闭临时坐标流
  if (tempCoordFile) {
    tempCoordFile.close();
  }
  
  // 生成对齐两步路规范的最终KML文件
  generateFinalKML();
  
  // 清理临时文件
  if (SD.exists(tempCoordFileName)) {
    SD.remove(tempCoordFileName);
  }
  
  isTrackingEnabled = false;
  Serial.printf("[Tracking] Stopped tracking. Recorded %d track points, %d waypoints to %s\n",
                trackPoints.size(), waypoints.size(), trackFileName.c_str());
}

bool TrackingManager::isTracking() const {
  return isTrackingEnabled;
}

void TrackingManager::updateTracking(const Location& currentLocation) {
  if (!isTrackingEnabled || !currentLocation.isValid) {
    return;
  }
  
  // 检查是否满足距离或时间记录条件
  if (shouldRecordPoint(currentLocation)) {
    // 累加距离与爬升/下降
    if (lastRecordedPoint.isValid) {
      double dDist = calculateDistance(currentLocation, lastRecordedPoint);
      if (dDist > 0.1) {
        totalDistance += dDist;
      }
      double dAlt = currentLocation.altitude - lastRecordedPoint.altitude;
      if (dAlt > 1.0) {
        elevationGain += dAlt;
      } else if (dAlt < -1.0) {
        elevationLoss += (-dAlt);
      }
    }
    
    // 记录到内存点集（用于屏幕实时渲染）
    trackPoints.push_back(currentLocation);
    
    // 实时追加到临时坐标文件
    if (tempCoordFile) {
      tempCoordFile.printf("%.6f,%.6f,%.2f ", currentLocation.longitude, currentLocation.latitude, currentLocation.altitude);
      tempCoordFile.flush();
    }
    
    // 更新状态
    lastRecordedPoint = currentLocation;
    lastRecordTime = millis();
    trackPointIndex++;
    
    Serial.printf("[Tracking] Point #%d: (%.6f, %.6f, %.1fm), Dist: %.1fm, +%.1fm/-%.1fm\n",
                  trackPointIndex, currentLocation.latitude, currentLocation.longitude, currentLocation.altitude,
                  totalDistance, elevationGain, elevationLoss);
  }
}

bool TrackingManager::addWaypoint(const String& name, const Location& loc) {
  if (!isTrackingEnabled || !loc.isValid) {
    return false;
  }
  
  TrackWaypoint wp;
  wp.name = (name.length() > 0) ? name : String("POI");
  wp.loc = loc;
  wp.timeStr = getIsoTimeString();
  wp.timeMs = getEpochTimeMs();
  float spdKmph = (gnssModule) ? (float)gnssModule->getSpeedKmph() : loc.speed;
  wp.speed = (spdKmph > 0.1f) ? (spdKmph / 3.6f) : 0.0f;
  wp.accuracy = (gnssModule) ? (float)gnssModule->getHDOP() : 0.0f;
  
  waypoints.push_back(wp);
  Serial.printf("[Tracking] Inserted Waypoint #%d: '%s' at (%.6f, %.6f, %.1fm), Spd: %.1fm/s, Acc: %.1f\n",
                waypoints.size(), wp.name.c_str(), loc.latitude, loc.longitude, loc.altitude, wp.speed, wp.accuracy);
  return true;
}

const std::vector<TrackWaypoint>& TrackingManager::getWaypoints() const {
  return waypoints;
}

const std::vector<Location>& TrackingManager::getTrackPoints() const {
  return trackPoints;
}

void TrackingManager::clearTrack() {
  trackPoints.clear();
  waypoints.clear();
  totalDistance = 0.0;
  elevationGain = 0.0;
  elevationLoss = 0.0;
}

bool TrackingManager::shouldRecordPoint(const Location& currentLocation) const {
  if (!lastRecordedPoint.isValid) {
    return true;
  }
  
  // 移动距离 >= 5 米
  double distance = calculateDistance(currentLocation, lastRecordedPoint);
  if (distance >= 5.0) {
    return true;
  }
  
  // 或者时间间隔 >= 5 秒且距离微变 (>= 2米)
  unsigned long currentTime = millis();
  if (currentTime - lastRecordTime >= 5000 && distance >= 2.0) {
    return true;
  }
  
  return false;
}

double TrackingManager::calculateDistance(const Location& p1, const Location& p2) const {
  const double R = 6371000.0;
  double lat1 = p1.latitude * M_PI / 180.0;
  double lon1 = p1.longitude * M_PI / 180.0;
  double lat2 = p2.latitude * M_PI / 180.0;
  double lon2 = p2.longitude * M_PI / 180.0;
  
  double dlat = lat2 - lat1;
  double dlon = lon2 - lon1;
  
  double a = sin(dlat / 2) * sin(dlat / 2) +
             cos(lat1) * cos(lat2) *
             sin(dlon / 2) * sin(dlon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  
  return R * c;
}

String TrackingManager::getIsoTimeString() const {
  if (gnssModule && gnssModule->isDateValid() && gnssModule->isTimeValid()) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
             gnssModule->getYear(), gnssModule->getMonth(), gnssModule->getDay(),
             gnssModule->getHour(), gnssModule->getMinute(), gnssModule->getSecond());
    return String(buf);
  }
  
  time_t now = time(nullptr);
  struct tm* t = gmtime(&now);
  char buf[32];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02dT%02d:%02d:%02dZ",
           1900 + t->tm_year, t->tm_mon + 1, t->tm_mday,
           t->tm_hour, t->tm_min, t->tm_sec);
  return String(buf);
}

uint64_t TrackingManager::getEpochTimeMs() const {
  if (gnssModule && gnssModule->isDateValid() && gnssModule->isTimeValid()) {
    int y = gnssModule->getYear();
    int m = gnssModule->getMonth();
    int d = gnssModule->getDay();
    int hh = gnssModule->getHour();
    int mm = gnssModule->getMinute();
    int ss = gnssModule->getSecond();
    
    // 基于格里高利历法计算自 1970-01-01 00:00:00 UTC 起的真实秒数
    int y1 = y;
    int m1 = m;
    if (m1 <= 2) {
      y1 -= 1;
      m1 += 12;
    }
    long days = 365L * y1 + y1 / 4 - y1 / 100 + y1 / 400 + (153 * (m1 - 3) + 2) / 5 + d - 1;
    long daysSince1970 = days - 719468L;
    uint64_t epochSec = (uint64_t)daysSince1970 * 86400ULL + hh * 3600ULL + mm * 60ULL + ss;
    return epochSec * 1000ULL;
  }
  
  // 回退：读取系统 RTC 时钟
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return ((uint64_t)tv.tv_sec * 1000ULL) + ((uint64_t)tv.tv_usec / 1000ULL);
}

String TrackingManager::generateKMLFileName(const String& customPrefix) const {
  String fileName = "";
  if (customPrefix.length() > 0) {
    fileName = customPrefix + "_";
  } else {
    fileName = "hikepod_track_";
  }
  
  if (gnssModule && gnssModule->isDateValid() && gnssModule->isTimeValid()) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d",
             gnssModule->getLocalYear(), gnssModule->getLocalMonth(), gnssModule->getLocalDay(),
             gnssModule->getLocalHour(), gnssModule->getLocalMinute(), gnssModule->getLocalSecond());
    fileName += buf;
  } else {
    time_t now = time(nullptr);
    struct tm* t = localtime(&now);
    char buf[32];
    snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d",
             1900 + t->tm_year, t->tm_mon + 1, t->tm_mday,
             t->tm_hour, t->tm_min, t->tm_sec);
    fileName += buf;
  }
  
  fileName += ".kml";
  return String("/HikePod/") + fileName;
}

void TrackingManager::generateFinalKML() {
  File kml = SD.open(trackFileName, FILE_WRITE);
  if (!kml) {
    Serial.printf("[Tracking] Error: Failed to open %s for writing\n", trackFileName.c_str());
    return;
  }
  
  unsigned long timeUsedMs = millis() - trackingStartTimeMs;
  uint64_t endTimeEpochMs = getEpochTimeMs();
  String endTimeIso = getIsoTimeString();
  double avgSpeed = (timeUsedMs > 0) ? (totalDistance / (timeUsedMs / 1000.0)) : 0.0;
  
  // 1. 写入 XML 头与 Document 根节点（对齐两步路 TbuluKmlVersion2 规范）
  kml.print("<?xml version='1.0' encoding='UTF-8' standalone='yes' ?>\n");
  kml.print("<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\">\n");
  kml.print("  <Document id=\"TbuluKmlVersion2\">\n");
  kml.printf("    <name><![CDATA[HikePod_%llu]]></name>\n", (unsigned long long)beginTimeEpochMs);
  kml.print("    <description><![CDATA[通过 HikePod 户外导航仪记录生成]]></description>\n");
  kml.print("    <snippet><![CDATA[通过“HikePod”生成]]></snippet>\n");
  kml.print("    <author><![CDATA[HikePod]]></author>\n");
  
  // 2. 写入两步路 / 户外规范核心 ExtendedData
  kml.print("    <ExtendedData>\n");
  kml.print("      <Data name=\"TrackTags\"><value>徒步</value></Data>\n");
  kml.printf("      <Data name=\"BeginTime\"><value>%llu</value></Data>\n", (unsigned long long)beginTimeEpochMs);
  kml.printf("      <Data name=\"EndTime\"><value>%llu</value></Data>\n", (unsigned long long)endTimeEpochMs);
  kml.printf("      <Data name=\"TimeUsed\"><value>%lu</value></Data>\n", timeUsedMs);
  kml.print("      <Data name=\"PauseTime\"><value>0</value></Data>\n");
  kml.print("      <Data name=\"color\"><value>ee0000ff</value></Data>\n");
  kml.printf("      <Data name=\"Distance\"><value>%.1f</value></Data>\n", totalDistance);
  kml.printf("      <Data name=\"ElevationGain\"><value>%.2f</value></Data>\n", elevationGain);
  kml.printf("      <Data name=\"ElevationLoss\"><value>%.2f</value></Data>\n", elevationLoss);
  kml.printf("      <Data name=\"SportAvgSpeed\"><value>%.4f</value></Data>\n", avgSpeed);
  kml.print("    </ExtendedData>\n");
  
  // 3. 写入标准 Style 定义
  kml.print("    <Style id=\"LineStringStyle\">\n");
  kml.print("      <LineStyle>\n");
  kml.print("        <color>ee0000ff</color>\n");
  kml.print("        <width>6</width>\n");
  kml.print("      </LineStyle>\n");
  kml.print("    </Style>\n");
  kml.print("    <Style id=\"startPointStyle\">\n");
  kml.print("      <IconStyle><scale>1.1</scale></IconStyle>\n");
  kml.print("    </Style>\n");
  kml.print("    <Style id=\"endPointStyle\">\n");
  kml.print("      <IconStyle><scale>1.1</scale></IconStyle>\n");
  kml.print("    </Style>\n");
  kml.print("    <Style id=\"MarkerStyleText\">\n");
  kml.print("      <LabelStyle><color>ff00ffff</color><colorMode>normal</colorMode></LabelStyle>\n");
  kml.print("      <IconStyle><scale>1.1</scale></IconStyle>\n");
  kml.print("    </Style>\n");
  
  // 4. 写入标注点文件夹（Folder id="TbuluHisPointFolder"）
  kml.print("    <Folder id=\"TbuluHisPointFolder\">\n");
  kml.print("      <name>标注点</name>\n");
  
  // 起点
  if (!trackPoints.empty()) {
    const Location& startLoc = trackPoints.front();
    kml.print("      <Placemark id=\"startPoint\">\n");
    kml.print("        <name>起点</name>\n");
    kml.print("        <styleUrl>#startPointStyle</styleUrl>\n");
    kml.printf("        <TimeStamp><when>%s</when></TimeStamp>\n", beginTimeIso.c_str());
    kml.print("        <Point>\n");
    kml.printf("          <coordinates>%.6f,%.6f,%.2f </coordinates>\n", startLoc.longitude, startLoc.latitude, startLoc.altitude);
    kml.print("        </Point>\n");
    kml.print("      </Placemark>\n");
  }
  
  // 途径标注点（由用户按 i 键输入）
  for (size_t i = 0; i < waypoints.size(); i++) {
    const TrackWaypoint& wp = waypoints[i];
    kml.print("      <Placemark id=\"realPoint\">\n");
    kml.printf("        <name><![CDATA[%s]]></name>\n", wp.name.c_str());
    kml.print("        <styleUrl>#MarkerStyleText</styleUrl>\n");
    kml.printf("        <TimeStamp><when>%s</when></TimeStamp>\n", wp.timeStr.c_str());
    kml.print("        <Point>\n");
    kml.printf("          <coordinates>%.6f,%.6f,%.2f </coordinates>\n", wp.loc.longitude, wp.loc.latitude, wp.loc.altitude);
    kml.print("        </Point>\n");
    kml.print("        <ExtendedData>\n");
    kml.printf("          <Data name=\"ServerId\"><value>%u</value></Data>\n", (unsigned int)(i + 1));
    kml.print("          <Data name=\"PosType\"><value>0</value></Data>\n");
    kml.printf("          <Data name=\"Speed\"><value>%.1f</value></Data>\n", wp.speed);
    kml.printf("          <Data name=\"Accuracy\"><value>%.1f</value></Data>\n", wp.accuracy);
    kml.printf("          <Data name=\"Time\"><value>%llu</value></Data>\n", (unsigned long long)wp.timeMs);
    kml.print("        </ExtendedData>\n");
    kml.print("      </Placemark>\n");
  }
  
  // 终点
  if (trackPoints.size() > 1) {
    const Location& endLoc = trackPoints.back();
    kml.print("      <Placemark id=\"endPoint\">\n");
    kml.print("        <name>终点</name>\n");
    kml.print("        <styleUrl>#endPointStyle</styleUrl>\n");
    kml.printf("        <TimeStamp><when>%s</when></TimeStamp>\n", endTimeIso.c_str());
    kml.print("        <Point>\n");
    kml.printf("          <coordinates>%.6f,%.6f,%.2f </coordinates>\n", endLoc.longitude, endLoc.latitude, endLoc.altitude);
    kml.print("        </Point>\n");
    kml.print("      </Placemark>\n");
  }
  
  kml.print("    </Folder>\n");
  
  // 5. 写入导航线文件夹（Folder id="TbuluLineStringFolder"）
  kml.print("    <Folder id=\"TbuluLineStringFolder\">\n");
  kml.print("      <name>导航线</name>\n");
  kml.print("      <Placemark>\n");
  kml.print("        <name><![CDATA[轨迹记录]]></name>\n");
  kml.print("        <styleUrl>#LineStringStyle</styleUrl>\n");
  kml.print("        <description>\n");
  kml.print("          <div>通过“HikePod”生成</div>\n");
  kml.printf("          <div>轨迹点数:%u</div>\n", (unsigned int)trackPoints.size());
  kml.printf("          <div>本段里程:%.1f米</div>\n", totalDistance);
  kml.print("        </description>\n");
  kml.print("        <LineString>\n");
  kml.print("          <coordinates>");
  
  // 写入轨迹连续坐标串
  for (size_t i = 0; i < trackPoints.size(); i++) {
    const Location& p = trackPoints[i];
    kml.printf("%.6f,%.6f,%.2f ", p.longitude, p.latitude, p.altitude);
  }
  
  kml.print("</coordinates>\n");
  kml.print("        </LineString>\n");
  kml.print("      </Placemark>\n");
  kml.print("    </Folder>\n");
  
  // 6. 结束标签
  kml.print("  </Document>\n");
  kml.print("</kml>\n");
  
  kml.close();
  Serial.printf("[Tracking] Final KML successfully written to %s (size: %llu bytes)\n",
                trackFileName.c_str(), (unsigned long long)SD.open(trackFileName).size());
}

void TrackingManager::ensureTracksDirectoryExists() {
  if (!SD.exists("/HikePod")) {
    if (SD.mkdir("/HikePod")) {
      Serial.println("[Tracking] Created /HikePod directory");
    } else {
      Serial.println("[Tracking] Failed to create /HikePod directory");
    }
  }
}
