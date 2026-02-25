#include "TrackingManager.h"
#include <SD.h>
#include <time.h>

TrackingManager::TrackingManager() : 
  isTrackingEnabled(false),
  gnssModule(nullptr),
  trackFile(),
  trackFileName(""),
  trackPoints(),
  lastRecordedPoint(),
  lastRecordTime(0),
  trackPointIndex(0) {
  lastRecordedPoint.isValid = false;
}

void TrackingManager::begin() {
  // 确保tracks目录存在
  ensureTracksDirectoryExists();
}

void TrackingManager::setGNSSModule(GNSSModule* gnss) {
  gnssModule = gnss;
}

bool TrackingManager::startTracking() {
  if (isTrackingEnabled) {
    return true; // 已经在跟踪
  }
  
  // 生成文件名
  trackFileName = generateKMLFileName();
  
  // 初始化KML文件
  if (!initializeKMLFile()) {
    return false;
  }
  
  // 写入KML文件头
  writeKMLHeader();
  
  // 重置状态
  trackPoints.clear();
  lastRecordedPoint.isValid = false;
  lastRecordTime = 0;
  trackPointIndex = 0;
  
  isTrackingEnabled = true;
  Serial.println("[Tracking] Started tracking");
  return true;
}

void TrackingManager::stopTracking() {
  if (!isTrackingEnabled) {
    return;
  }
  
  // 写入KML文件尾
  writeKMLFooter();
  
  // 关闭文件
  if (trackFile) {
    trackFile.close();
  }
  
  isTrackingEnabled = false;
  Serial.println("[Tracking] Stopped tracking");
  Serial.printf("[Tracking] Recorded %d points to %s\n", trackPoints.size(), trackFileName.c_str());
}

bool TrackingManager::isTracking() const {
  return isTrackingEnabled;
}

void TrackingManager::updateTracking(const Location& currentLocation) {
  if (!isTrackingEnabled || !currentLocation.isValid) {
    return;
  }
  
  // 检查是否应该记录新点
  if (shouldRecordPoint(currentLocation)) {
    // 记录新点
    trackPoints.push_back(currentLocation);
    
    // 写入KML文件
    writeTrackPoint(currentLocation);
    
    // 更新最后记录的点和时间
    lastRecordedPoint = currentLocation;
    lastRecordTime = millis();
    trackPointIndex++;
    
    Serial.printf("[Tracking] Recorded point %d: Lat=%.6f, Lng=%.6f, Alt=%.2f\n", 
                  trackPointIndex, currentLocation.latitude, currentLocation.longitude, currentLocation.altitude);
  }
}

const std::vector<Location>& TrackingManager::getTrackPoints() const {
  return trackPoints;
}

void TrackingManager::clearTrack() {
  trackPoints.clear();
}

bool TrackingManager::shouldRecordPoint(const Location& currentLocation) const {
  // 如果没有上一个点，直接记录
  if (!lastRecordedPoint.isValid) {
    return true;
  }
  
  // 检查距离条件（≥3-5米）
  double distance = calculateDistance(currentLocation, lastRecordedPoint);
  if (distance >= 5.0) {
    return true;
  }
  
  // 检查时间条件（≥5秒）
  unsigned long currentTime = millis();
  if (currentTime - lastRecordTime >= 5000) {
    return true;
  }
  
  return false;
}

double TrackingManager::calculateDistance(const Location& p1, const Location& p2) const {
  // 简化的距离计算，使用Haversine公式的近似
  const double R = 6371000.0; // 地球半径（米）
  
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

String TrackingManager::generateKMLFileName() const {
  // 使用GPS本地时间生成文件名
  String fileName = "hikepod_track_";
  
  if (gnssModule && gnssModule->isDateValid() && gnssModule->isTimeValid()) {
    // 添加年份
    fileName += String(gnssModule->getLocalYear());
    if (gnssModule->getLocalMonth() < 10) {
      fileName += "0";
    }
    fileName += String(gnssModule->getLocalMonth());
    if (gnssModule->getLocalDay() < 10) {
      fileName += "0";
    }
    fileName += String(gnssModule->getLocalDay());
    fileName += "_";
    if (gnssModule->getLocalHour() < 10) {
      fileName += "0";
    }
    fileName += String(gnssModule->getLocalHour());
    if (gnssModule->getLocalMinute() < 10) {
      fileName += "0";
    }
    fileName += String(gnssModule->getLocalMinute());
    if (gnssModule->getLocalSecond() < 10) {
      fileName += "0";
    }
    fileName += String(gnssModule->getLocalSecond());
  } else {
    // 如果GPS时间不可用，使用系统时间
    time_t now = time(nullptr);
    struct tm* timeinfo = gmtime(&now);
    
    fileName += String(1900 + timeinfo->tm_year);
    if (timeinfo->tm_mon + 1 < 10) {
      fileName += "0";
    }
    fileName += String(timeinfo->tm_mon + 1);
    if (timeinfo->tm_mday < 10) {
      fileName += "0";
    }
    fileName += String(timeinfo->tm_mday);
    fileName += "_";
    if (timeinfo->tm_hour < 10) {
      fileName += "0";
    }
    fileName += String(timeinfo->tm_hour);
    if (timeinfo->tm_min < 10) {
      fileName += "0";
    }
    fileName += String(timeinfo->tm_min);
    if (timeinfo->tm_sec < 10) {
      fileName += "0";
    }
    fileName += String(timeinfo->tm_sec);
  }
  
  fileName += ".kml";
  
  return String("/HikePod/") + fileName;
}

bool TrackingManager::initializeKMLFile() {
  // 确保tracks目录存在
  ensureTracksDirectoryExists();
  
  // 打开文件用于写入
  trackFile = SD.open(trackFileName, FILE_WRITE);
  if (!trackFile) {
    Serial.printf("[Tracking] Failed to open track file: %s\n", trackFileName.c_str());
    return false;
  }
  
  return true;
}

void TrackingManager::writeKMLHeader() {
  if (!trackFile) return;
  
  trackFile.print("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
  trackFile.print("<kml xmlns=\"http://www.opengis.net/kml/2.2\" xmlns:gx=\"http://www.google.com/kml/ext/2.2\">\n");
  trackFile.print("<Document>\n");
  trackFile.print("  <name>hikepod track</name>\n");
  trackFile.print("  <description>\n");
  trackFile.print("    <div>generated by hikepod</div>\n");
  trackFile.print("  </description>\n");
  trackFile.print("  <Style id=\"trackStyle\">\n");
  trackFile.print("    <LineStyle>\n");
  trackFile.print("      <color>ff0000ff</color>\n"); // 红色
  trackFile.print("      <width>4</width>\n");
  trackFile.print("    </LineStyle>\n");
  trackFile.print("  </Style>\n");
  trackFile.print("  <Placemark>\n");
  trackFile.print("    <name>hikepod tracking</name>\n");
  trackFile.print("    <styleUrl>#trackStyle</styleUrl>\n");
  trackFile.print("    <gx:Track>\n");
}

void TrackingManager::writeTrackPoint(const Location& location) {
  if (!trackFile) return;
  
  // 写入时间戳 - 使用GPS时间
  String timeStr = "";
  
  if (gnssModule && gnssModule->isDateValid() && gnssModule->isTimeValid()) {
    // 使用GPS时间
    timeStr += String(gnssModule->getYear()) + "-";
    if (gnssModule->getMonth() < 10) {
      timeStr += "0";
    }
    timeStr += String(gnssModule->getMonth()) + "-";
    if (gnssModule->getDay() < 10) {
      timeStr += "0";
    }
    timeStr += String(gnssModule->getDay()) + "T";
    if (gnssModule->getHour() < 10) {
      timeStr += "0";
    }
    timeStr += String(gnssModule->getHour()) + ":";
    if (gnssModule->getMinute() < 10) {
      timeStr += "0";
    }
    timeStr += String(gnssModule->getMinute()) + ":";
    if (gnssModule->getSecond() < 10) {
      timeStr += "0";
    }
    timeStr += String(gnssModule->getSecond()) + "Z";
  } else {
    // 如果GPS时间不可用，使用系统时间
    time_t now = time(nullptr);
    struct tm* timeinfo = gmtime(&now);
    
    timeStr += String(1900 + timeinfo->tm_year) + "-";
    if (timeinfo->tm_mon + 1 < 10) {
      timeStr += "0";
    }
    timeStr += String(timeinfo->tm_mon + 1) + "-";
    if (timeinfo->tm_mday < 10) {
      timeStr += "0";
    }
    timeStr += String(timeinfo->tm_mday) + "T";
    if (timeinfo->tm_hour < 10) {
      timeStr += "0";
    }
    timeStr += String(timeinfo->tm_hour) + ":";
    if (timeinfo->tm_min < 10) {
      timeStr += "0";
    }
    timeStr += String(timeinfo->tm_min) + ":";
    if (timeinfo->tm_sec < 10) {
      timeStr += "0";
    }
    timeStr += String(timeinfo->tm_sec) + "Z";
  }
  
  trackFile.print("      <gx:when>");
  trackFile.print(timeStr);
  trackFile.println("</gx:when>");
  
  // 写入坐标
  trackFile.print("      <gx:coord>");
  trackFile.print(location.longitude, 7);
  trackFile.print(" ");
  trackFile.print(location.latitude, 7);
  trackFile.print(" ");
  trackFile.print(location.altitude, 2);
  trackFile.println("</gx:coord>");
  
  // 写入扩展数据
  trackFile.println("      <ExtendedData>");
  trackFile.println("        <Data name=\"speed\">");
  trackFile.println("          <value>0</value>"); // 暂时设为0，后续可从GPS获取
  trackFile.println("        </Data>");
  trackFile.println("        <Data name=\"course\">");
  trackFile.println("          <value>0</value>"); // 暂时设为0，后续可从GPS获取
  trackFile.println("        </Data>");
  trackFile.println("        <Data name=\"hdop\">");
  trackFile.println("          <value>0</value>"); // 暂时设为0，后续可从GPS获取
  trackFile.println("        </Data>");
  trackFile.println("        <Data name=\"index\">");
  trackFile.print("          <value>");
  trackFile.print(trackPointIndex);
  trackFile.println("</value>");
  trackFile.println("        </Data>");
  trackFile.println("      </ExtendedData>");
}

void TrackingManager::writeKMLFooter() {
  if (!trackFile) return;
  
  trackFile.print("    </gx:Track>\n");
  trackFile.print("  </Placemark>\n");
  trackFile.print("</Document>\n");
  trackFile.print("</kml>\n");
}

void TrackingManager::ensureTracksDirectoryExists() {
  if (!SD.exists("/HikePod")) {
    if (SD.mkdir("/HikePod")) {
      Serial.println("[Tracking] Created HikePod directory");
    } else {
      Serial.println("[Tracking] Failed to create HikePod directory");
    }
  }
}
