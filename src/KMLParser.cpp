#include "KMLParser.h"
#include <SD.h>
#include <Arduino.h>
#include <algorithm>

KMLParser::KMLParser() {
  // 初始化内存池
  pointPool = new Location[MAX_POINTS];
  currentPointCount = 0;
  Serial.println("=== KML Parser: Memory pool initialized with " + String(MAX_POINTS) + " locations");
}

KMLParser::~KMLParser() {
  // 释放内存池
  if (pointPool != nullptr) {
    delete[] pointPool;
    pointPool = nullptr;
    Serial.println("=== KML Parser: Memory pool released");
  }
}

bool KMLParser::addPointToPool(double lat, double lng, double alt) {
  if (currentPointCount >= MAX_POINTS) {
    Serial.println("=== KML Parser: Memory pool full, cannot add more points");
    return false;
  }
  
  // 直接在内存池中构造Location对象
  pointPool[currentPointCount] = Location(lat, lng, alt);
  currentPointCount++;
  
  // 每1000个点输出一次内存使用情况
  if (currentPointCount % 1000 == 0) {
    size_t usedMemory = currentPointCount * sizeof(Location);
    Serial.println("=== KML Parser: Memory pool usage: " + String(currentPointCount) + " points, " + String(usedMemory) + " bytes");
  }
  
  return true;
}

void KMLParser::reset() {
  currentPointCount = 0;
  routeName = "";
  Serial.println("=== KML Parser: Parser reset");
}

int KMLParser::getPointCount() {
  return currentPointCount;
}

void KMLParser::processPointsInChunks(PointProcessor processor, void* userData, int chunkSize) {
  if (processor == nullptr) {
    Serial.println("=== KML Parser: Invalid processor callback");
    return;
  }
  
  Serial.println("=== KML Parser: Starting to process points in chunks of " + String(chunkSize));
  
  int processed = 0;
  while (processed < currentPointCount) {
    int end = processed + chunkSize;
    if (end > currentPointCount) {
      end = currentPointCount;
    }
    
    // 处理当前批次的点
    for (int i = processed; i < end; i++) {
      processor(pointPool[i], userData);
    }
    
    processed = end;
    Serial.println("=== KML Parser: Processed " + String(processed) + "/" + String(currentPointCount) + " points");
  }
  
  Serial.println("=== KML Parser: Point processing completed");
}

const Location* KMLParser::getPointPool() {
  return pointPool;
}

bool KMLParser::parseFile(const char* filePath) {
  Serial.println("=== KML Parser: Starting to parse file " + String(filePath));
  
  // 直接解析文件，不一次性读取整个文件到内存
  if (parseFileDirect(filePath)) {
    Serial.println("=== KML Parser: Direct file parsing successful");
    return true;
  }
  
  // 如果直接解析失败，尝试使用传统方法
  String content = readFileContent(filePath);
  if (content.isEmpty()) {
    Serial.println("=== KML Parser: Failed to read file content");
    return false;
  }
  
  Serial.println("=== KML Parser: File read successfully, size: " + String(content.length()) + " bytes");
  
  // 提取轨迹名称
  int nameStart = content.indexOf("<name>");
  if (nameStart != -1) {
    int nameEnd = content.indexOf("</name>", nameStart);
    if (nameEnd != -1) {
      routeName = content.substring(nameStart + 6, nameEnd);
      Serial.println("=== KML Parser: Found route name: " + routeName);
    }
  }
  
  // 尝试查找标准LineString格式
  int lineStringStart = content.indexOf("<LineString>");
  if (lineStringStart != -1) {
    int coordinatesStart = content.indexOf("<coordinates>", lineStringStart);
    int coordinatesEnd = content.indexOf("</coordinates>", coordinatesStart);
    
    if (coordinatesStart != -1 && coordinatesEnd != -1) {
      Serial.println("=== KML Parser: Found LineString format");
      String coordinatesStr = content.substring(coordinatesStart + 12, coordinatesEnd);
      bool result = parseCoordinates(coordinatesStr);
      Serial.println("=== KML Parser: LineString parsing result: " + String(result));
      return result;
    } else {
      Serial.println("=== KML Parser: LineString found but no coordinates");
    }
  }
  
  // 尝试查找Google扩展gx:Track格式（支持多个Track片段）
  int trackStart = content.indexOf("<gx:Track>");
  int trackCount = 0;
  Serial.println("=== KML Parser: Starting to search for gx:Track segments");
  while (trackStart != -1) {
    trackCount++;
    Serial.println("=== KML Parser: Found track segment " + String(trackCount) + " at position " + String(trackStart));
    int trackEnd = content.indexOf("</gx:Track>", trackStart);
    if (trackEnd != -1) {
      Serial.println("=== KML Parser: Found gx:Track format, segment " + String(trackCount));
      String trackContent = content.substring(trackStart + 10, trackEnd);
      Serial.println("=== KML Parser: Track content length: " + String(trackContent.length()));
      bool result = parseGxTrack(trackContent);
      Serial.println("=== KML Parser: gx:Track parsing result: " + String(result) + ", added points: " + String(currentPointCount - (trackCount - 1) * 700) + ", total points: " + String(currentPointCount));
      
      // 查找下一个Track片段
      trackStart = content.indexOf("<gx:Track>", trackEnd);
      Serial.println("=== KML Parser: Next track segment position: " + String(trackStart));
    } else {
      // 即使找不到结束标签，也尝试解析从trackStart开始的所有内容
      Serial.println("=== KML Parser: gx:Track found but incomplete, trying to parse anyway");
      String trackContent = content.substring(trackStart + 10);
      bool result = parseGxTrack(trackContent);
      Serial.println("=== KML Parser: gx:Track parsing result: " + String(result));
      break;
    }
  }
  
  // 如果找到了至少一个Track片段，返回成功
  if (currentPointCount > 0) {
    Serial.println("=== KML Parser: Successfully parsed all " + String(trackCount) + " gx:Track segments, total points: " + String(currentPointCount));
    return true;
  }
  
  Serial.println("=== KML Parser: No supported KML format found");
  return false;
}

bool KMLParser::parseCoordinates(const String& coordinatesStr) {
  currentPointCount = 0;
  Serial.println("=== KML Parser: Starting to parse coordinates string");
  
  int start = 0;
  int end = 0;
  int pointCount = 0;
  
  while ((end = coordinatesStr.indexOf(' ', start)) != -1) {
    String coord = coordinatesStr.substring(start, end);
    
    // 解析经度,纬度,高度
    int comma1 = coord.indexOf(',');
    int comma2 = coord.indexOf(',', comma1 + 1);
    
    if (comma1 != -1 && comma2 != -1) {
      double lng = coord.substring(0, comma1).toDouble();
      double lat = coord.substring(comma1 + 1, comma2).toDouble();
      double alt = coord.substring(comma2 + 1).toDouble();
      
      // 使用内存池添加点
      if (addPointToPool(lat, lng, alt)) {
        pointCount++;
        
        // 每10个点输出一次日志
        if (pointCount % 10 == 0) {
          Serial.println("=== KML Parser: Parsed " + String(pointCount) + " points...");
        }
      } else {
        // 内存池已满，停止解析
        break;
      }
    }
    
    start = end + 1;
  }
  
  Serial.println("=== KML Parser: Coordinates parsing complete. Found " + String(currentPointCount) + " points");
  return currentPointCount > 0;
}

bool KMLParser::parseGxTrack(const String& trackContent) {
  Serial.println("=== KML Parser: Starting to parse gx:Track format");
  
  int coordStart = trackContent.indexOf("<gx:coord>");
  int pointCount = 0;
  int initialSize = currentPointCount;
  
  while (coordStart != -1) {
    int coordEnd = trackContent.indexOf("</gx:coord>", coordStart);
    if (coordEnd != -1) {
      String coordStr = trackContent.substring(coordStart + 10, coordEnd);
      
      // 解析经度 纬度 高度 (空格分隔)
      int space1 = coordStr.indexOf(' ');
      int space2 = coordStr.indexOf(' ', space1 + 1);
      
      if (space1 != -1 && space2 != -1) {
        // 添加详细的调试输出
        Serial.print("=== KML Parser: Parsing coord: '" + coordStr + "', length: " + String(coordStr.length()) + ", space1: " + String(space1) + ", space2: " + String(space2));
        
        // 提取并打印经度字符串
        String lngStr = coordStr.substring(0, space1);
        Serial.print(", lngStr: '" + lngStr + "', length: " + String(lngStr.length()));
        
        // 提取并打印纬度字符串
        String latStr = coordStr.substring(space1 + 1, space2);
        Serial.print(", latStr: '" + latStr + "', length: " + String(latStr.length()));
        
        // 解析值
        double lng = lngStr.toDouble();
        double lat = latStr.toDouble();
        double alt = coordStr.substring(space2 + 1).toDouble();
        
        // 添加调试输出
        Serial.println(", lng: " + String(lng) + ", lat: " + String(lat) + ", alt: " + String(alt));
        
        // 使用内存池添加点
        if (addPointToPool(lat, lng, alt)) {
          pointCount++;
          
          // 每10个点输出一次日志
          if (pointCount % 10 == 0) {
            Serial.println("=== KML Parser: Parsed " + String(pointCount) + " gx:coord points...");
          }
        } else {
          // 内存池已满，停止解析
          break;
        }
      } else {
        Serial.println("=== KML Parser: Invalid coordinate format: '" + coordStr + "'" + ", length: " + String(coordStr.length()));
      }
      
      coordStart = trackContent.indexOf("<gx:coord>", coordEnd);
    } else {
      // 即使找不到结束标签，也尝试解析已找到的坐标点
      Serial.println("=== KML Parser: gx:Track incomplete, but parsed " + String(pointCount) + " points");
      break;
    }
  }
  
  Serial.println("=== KML Parser: gx:Track parsing complete. Added " + String(pointCount) + " points, total points: " + String(currentPointCount));
  return currentPointCount > initialSize;
}

String KMLParser::readFileContent(const char* filePath) {
  Serial.println("=== KML Parser: Attempting to open file " + String(filePath));
  
  File file;
  bool fileOpened = false;
  
  // 使用 SD 库打开文件
  file = SD.open(filePath);
  if (file) {
    Serial.println("=== KML Parser: File opened successfully with SD library");
    fileOpened = true;
  } else {
    Serial.println("=== KML Parser: Failed to open file with SD library");
    return "";
  }
  
  Serial.println("=== KML Parser: File size: " + String(file.size()) + " bytes");
  
  // 使用更大的缓冲区和更高效的内存管理
  const size_t BUFFER_SIZE = 4096;
  char buffer[BUFFER_SIZE];
  String content;
  unsigned long startTime = millis();
  
  while (file.available()) {
    size_t bytesRead = file.readBytes(buffer, BUFFER_SIZE);
    content += String(buffer, bytesRead);
  }
  
  unsigned long endTime = millis();
  file.close();
  
  Serial.println("=== KML Parser: File read completed in " + String(endTime - startTime) + " ms");
  Serial.println("=== KML Parser: Read " + String(content.length()) + " bytes");
  
  return content;
}

std::vector<Location> KMLParser::getRoutePoints() {
  // 创建一个空vector
  std::vector<Location> result;
  
  // 限制返回的点数量，避免栈溢出
  const int MAX_POINTS_RETURN = 2000;
  int pointsToReturn = (currentPointCount < MAX_POINTS_RETURN) ? currentPointCount : MAX_POINTS_RETURN;
  
  // 预分配内存
  result.reserve(pointsToReturn);
  
  // 从内存池复制点到vector中
  for (int i = 0; i < pointsToReturn; i++) {
    result.push_back(pointPool[i]);
  }
  
  if (pointsToReturn < currentPointCount) {
    Serial.println("=== KML Parser: Returning " + String(pointsToReturn) + " points (limited), total available: " + String(currentPointCount));
  } else {
    Serial.println("=== KML Parser: Returning all " + String(pointsToReturn) + " points");
  }
  
  return result;
}

String KMLParser::getRouteName() {
  return routeName;
}

Location KMLParser::getStartPoint() {
  if (currentPointCount > 0) {
    return pointPool[0];
  }
  return Location(); // 返回默认的无效位置
}

bool KMLParser::parseFileDirect(const char* filePath) {
  Serial.println("=== KML Parser: Starting direct file parsing: " + String(filePath));
  
  File file = SD.open(filePath);
  if (!file) {
    Serial.println("=== KML Parser: Failed to open file for direct parsing");
    return false;
  }
  
  Serial.println("=== KML Parser: File opened successfully for direct parsing");
  
  // 重置解析器状态
  currentPointCount = 0;
  
  // 缓冲区设置
  const size_t BUFFER_SIZE = 1024;
  char buffer[BUFFER_SIZE];
  String currentLine = "";
  bool inTrack = false;
  int trackCount = 0;
  int pointCount = 0;
  
  // 流式处理计数器
  int streamChunkSize = 1000; // 每1000个点处理一次
  int pointsSinceLastProcess = 0;
  
  // 逐行读取文件
  while (file.available()) {
    size_t bytesRead = file.readBytes(buffer, BUFFER_SIZE);
    
    for (size_t i = 0; i < bytesRead; i++) {
      char c = buffer[i];
      
      if (c == '\n') {
        // 处理完整行
        currentLine.trim();
        
        if (currentLine.indexOf("<gx:Track>") != -1) {
          inTrack = true;
          trackCount++;
          Serial.println("=== KML Parser: Direct parsing - found track segment " + String(trackCount));
        } else if (currentLine.indexOf("</gx:Track>") != -1) {
          inTrack = false;
          Serial.println("=== KML Parser: Direct parsing - end of track segment " + String(trackCount));
          Serial.println("=== KML Parser: Direct parsing - added " + String(pointCount) + " points for this segment");
          pointCount = 0;
          
          // 处理当前段的所有点
          if (currentPointCount > 0) {
            Serial.println("=== KML Parser: Stream processing - segment completed, total points: " + String(currentPointCount));
          }
        } else if (inTrack && currentLine.indexOf("<gx:coord>") != -1) {
          // 解析坐标
          int coordStart = currentLine.indexOf("<gx:coord>") + 10;
          int coordEnd = currentLine.indexOf("</gx:coord>", coordStart);
          
          if (coordStart != -1 && coordEnd != -1) {
            String coordStr = currentLine.substring(coordStart, coordEnd);
            coordStr.trim();
            
            // 解析经度 纬度 高度 (空格分隔)
            int space1 = coordStr.indexOf(' ');
            int space2 = coordStr.indexOf(' ', space1 + 1);
            
            if (space1 != -1 && space2 != -1) {
              double lng = coordStr.substring(0, space1).toDouble();
              double lat = coordStr.substring(space1 + 1, space2).toDouble();
              double alt = coordStr.substring(space2 + 1).toDouble();
              
              // 使用内存池添加点
              if (addPointToPool(lat, lng, alt)) {
                pointCount++;
                pointsSinceLastProcess++;
                
                // 每100个点输出一次日志
                if (pointCount % 100 == 0) {
                  Serial.println("=== KML Parser: Direct parsing - parsed " + String(pointCount) + " points in segment " + String(trackCount));
                }
                
                // 流式处理：每积累一定数量的点后处理
                if (pointsSinceLastProcess >= streamChunkSize) {
                  Serial.println("=== KML Parser: Stream processing - chunk completed, processing " + String(pointsSinceLastProcess) + " points");
                  pointsSinceLastProcess = 0;
                  
                  // 这里可以添加实际的处理逻辑，如：
                  // 1. 计算路线统计信息
                  // 2. 生成简化的路线表示
                  // 3. 更新UI显示等
                }
              } else {
                // 内存池已满，停止解析
                Serial.println("=== KML Parser: Memory pool full, stopping parsing");
                file.close();
                Serial.println("=== KML Parser: Direct parsing completed with memory limit");
                Serial.println("=== KML Parser: Direct parsing - total track segments: " + String(trackCount));
                Serial.println("=== KML Parser: Direct parsing - total points: " + String(currentPointCount));
                return currentPointCount > 0;
              }
            }
          }
        }
        
        currentLine = "";
      } else {
        currentLine += c;
      }
    }
  }
  
  file.close();
  
  Serial.println("=== KML Parser: Direct parsing completed");
  Serial.println("=== KML Parser: Direct parsing - total track segments: " + String(trackCount));
  Serial.println("=== KML Parser: Direct parsing - total points: " + String(currentPointCount));
  
  // 处理剩余的点
  if (pointsSinceLastProcess > 0) {
    Serial.println("=== KML Parser: Stream processing - final chunk, processing " + String(pointsSinceLastProcess) + " points");
  }
  
  return currentPointCount > 0;
}
