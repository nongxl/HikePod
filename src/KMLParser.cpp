#include "KMLParser.h"
#include <SD.h>
#include <Arduino.h>
#include <algorithm>

KMLParser::KMLParser() {
  // 初始化内存池 (使用 std::nothrow 防止内存分配失败导致系统崩溃)
  pointPool = new (std::nothrow) Location[MAX_POINTS];
  currentPointCount = 0;
  downsampleRate = 1;      // 默认不采样
  rawPointCounter = 0;
  poiCount = 0;
  
  if (pointPool != nullptr) {
    Serial.println("=== KML Parser: Memory pool initialized with " + String(MAX_POINTS) + " locations");
  } else {
    Serial.println("=== KML Parser: CRITICAL - Memory pool allocation failed!");
  }
}

KMLParser::~KMLParser() {
  // 释放内存池
  if (pointPool != nullptr) {
    delete[] pointPool;
    pointPool = nullptr;
    Serial.println("=== KML Parser: Memory pool released");
  }
}

void KMLParser::setDownsampleRate(int rate) {
  if (rate < 1) rate = 1;
  downsampleRate = rate;
  Serial.println("=== KML Parser: Downsample rate set to " + String(downsampleRate));
}

bool KMLParser::addPointToPool(double lat, double lng, double alt) {
  // 增加原始点计数器
  rawPointCounter++;
  
  // 根据采样率决定是否保留该点
  if (rawPointCounter % downsampleRate != 0) {
    return true; // 跳过该点，但返回 true 继续解析
  }

  if (pointPool == nullptr) return false;
  
  if (currentPointCount >= MAX_POINTS) {
    // 只有在第一次达到上限时输出日志
    if (currentPointCount == MAX_POINTS) {
      Serial.println("=== KML Parser: Memory pool full (" + String(MAX_POINTS) + "), skipping further points");
    }
    return false;
  }
  
  // 直接在内存池中构造Location对象
  pointPool[currentPointCount] = Location(lat, lng, alt);
  currentPointCount++;
  
  return true;
}

void KMLParser::reset() {
  currentPointCount = 0;
  rawPointCounter = 0;
  routeName = "";
  poiCount = 0;
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

bool KMLParser::parseFileDirect(const char* filePath) {
  File file = SD.open(filePath);
  if (!file) {
    Serial.println("=== KML Parser: Failed to open file for streaming");
    return false;
  }
  
  size_t fileSize = file.size();
  Serial.println("=== KML Parser: Streaming file size: " + String(fileSize) + " bytes");
  
  int estimatedPoints = fileSize / 45;
  if (estimatedPoints > MAX_POINTS) {
    downsampleRate = (estimatedPoints / MAX_POINTS) + 1;
    Serial.println("=== KML Parser: Streaming auto downsample rate: " + String(downsampleRate));
  } else {
    downsampleRate = 1;
    Serial.println("=== KML Parser: Loading all points (estimated " + String(estimatedPoints) + " < " + String(MAX_POINTS) + ")");
  }
  rawPointCounter = 0;
  currentPointCount = 0;
  poiCount = 0;
  
  bool inGxTrack = false;
  bool inLineString = false;
  bool inPlacemark = false;
  bool inPoint = false;
  String currentPoiName = "";
  
  while (file.available()) {
    char c = file.read();
    if (c == '<') {
      String tag = file.readStringUntil('>');
      tag.toLowerCase();
      
      if (tag.startsWith("!--") || tag.startsWith("![cdata[")) {
        continue;
      }
      
      if (tag == "name" || tag.startsWith("name ")) {
        String nameContent = "";
        
        // 循环读取直到找到内容或遇到结束标签
        while (file.available()) {
            int c = file.peek();
            if (c == -1) break;
            
            if (c == '<') {
                // 可能是 CDATA 的开始，或是结束标签 </name>
                file.read(); // 消耗 '<'
                int nextC = file.peek();
                if (nextC == '!') {
                    // CDATA 开始: <![CDATA[
                    String header = file.readStringUntil('['); // 消耗到 '['
                    if (header.indexOf("CDATA") != -1 || header.indexOf("cdata") != -1) {
                        String cdataContent = file.readStringUntil(']');
                        nameContent += cdataContent;
                        file.readStringUntil('>'); // 消耗 ]]> 的最后一个 '>'
                    }
                } else if (nextC == '/') {
                    // 结束标签 </name>
                    file.readStringUntil('>');
                    break;
                } else {
                    // 嵌套标签或其他普通文本中的 '<'
                    // 如果不是结束标签，我们把它当普通文本（或者是 KML 格式不规范）
                    nameContent += "<";
                }
            } else if (isspace(c)) {
                file.read(); // 跳过空白
                // 如果 nameContent 不为空，可以保留一个空格，或者这里简单处理全部 trim
            } else {
                // 读取到下一个 '<' 之前的所有文本
                nameContent += file.readStringUntil('<');
                // 注意：readStringUntil  CONSUMES the stop character!
                // 所以我们现在已经在 '<' 之后了。我们需要补回逻辑。
                // 为了逻辑统一，我们这里通过递归或循环重构。
                // 既然已经消耗了 '<'，我们可以直接判断它是结束标签还是 CDATA
                int nextC = file.peek();
                if (nextC == '/') {
                    file.readStringUntil('>');
                    break;
                } else if (nextC == '!') {
                    // 实际上这里又进到了 CDATA 逻辑，我们重新 loop 即可
                    // 但我们需要让下一次循环看到这个 '!' 
                    // 可是我们已经把 '<' 吃掉了。
                    // 方案：把原本的 '<' 逻辑逻辑移动出来。
                    // 简化方案：统一使用 peek() == '<' 并在分支内消费。
                } else {
                    // 普通文本里的 <，补回
                    nameContent += "<";
                }
            }
        }
        nameContent.trim();
        // 彻底的保险清理：检查并剥离各种可能的 CDATA 标记残留
        if (nameContent.indexOf("CDATA[") != -1 || nameContent.indexOf("cdata[") != -1) {
            int startPos = nameContent.indexOf("[");
            int endPos = nameContent.lastIndexOf("]]");
            if (startPos != -1 && endPos != -1 && endPos > startPos) {
                nameContent = nameContent.substring(startPos + 1, endPos);
            } else if (startPos != -1) {
                nameContent = nameContent.substring(startPos + 1);
            }
        }
        
        // 移除末尾残留的标记
        while (nameContent.length() > 0 && (nameContent.endsWith("]") || nameContent.endsWith(">"))) {
            nameContent = nameContent.substring(0, nameContent.length() - 1);
        }
        nameContent.trim();

        if (inPlacemark) {
            if (nameContent != "") {
                currentPoiName = nameContent;
            }
        } else if (routeName == "" || routeName.length() < 2) {
            if (nameContent != "") {
                routeName = nameContent;
                Serial.println("=== KML Parser: Found route name: " + routeName);
            }
        }
      } 
      else if (tag == "placemark" || tag.startsWith("placemark ")) {
        inPlacemark = true;
        currentPoiName = "";
        currentPoiLoc = Location(0, 0, 0);
        hasPoiLoc = false;
      }
      else if (tag == "/placemark") {
        if (inPlacemark && hasPoiLoc && poiCount < MAX_POIS) {
            String trimmedName = currentPoiName;
            trimmedName.trim();
            if (trimmedName != "") {
                poiPool[poiCount].name = trimmedName;
                poiPool[poiCount].loc = currentPoiLoc;
                poiCount++;
                Serial.println("=== KML Parser: Added POI: " + trimmedName + " at " + String(currentPoiLoc.latitude, 6) + ", " + String(currentPoiLoc.longitude, 6));
            }
        }
        inPlacemark = false;
        currentPoiName = "";
      }
      else if (tag == "gx:track") {
        inGxTrack = true;
      } 
      else if (tag == "/gx:track") {
        inGxTrack = false;
      }
      else if (tag == "linestring") {
        inLineString = true;
      }
      else if (tag == "/linestring") {
        inLineString = false;
      }
      else if (tag == "gx:coord" && inGxTrack) {
        String coordStr = file.readStringUntil('<');
        int space1 = coordStr.indexOf(' ');
        int space2 = coordStr.indexOf(' ', space1 + 1);
        if (space1 != -1) {
          double lng = coordStr.substring(0, space1).toDouble();
          double lat = (space2 != -1) ? coordStr.substring(space1 + 1, space2).toDouble() : coordStr.substring(space1 + 1).toDouble();
          double alt = (space2 != -1) ? coordStr.substring(space2 + 1).toDouble() : 0.0;
          addPointToPool(lat, lng, alt);
        }
      }
      else if (tag == "point") {
        inPoint = true;
      }
      else if (tag == "/point") {
        inPoint = false;
      }
      else if (tag == "coordinates") {
        if (inPoint && inPlacemark) {
           String coordStr = file.readStringUntil('<');
           coordStr.trim();
           int comma1 = coordStr.indexOf(',');
           int comma2 = coordStr.indexOf(',', comma1 + 1);
           if (comma1 != -1) {
             double lng = coordStr.substring(0, comma1).toDouble();
             double lat = (comma2 != -1) ? coordStr.substring(comma1 + 1, comma2).toDouble() : coordStr.substring(comma1 + 1).toDouble();
             double alt = (comma2 != -1) ? coordStr.substring(comma2 + 1).toDouble() : 0.0;
             currentPoiLoc = Location(lat, lng, alt);
             hasPoiLoc = true;
           }
        } 
        else if (inLineString || inGxTrack) {
          String currentToken = "";
          bool coordEnd = false;
          double components[3];
          int compIdx = 0;
          while (file.available() && !coordEnd) {
            char cc = file.read();
            if (cc == '<') {
              String endTag = file.readStringUntil('>');
              endTag.toLowerCase();
              if (endTag == "/coordinates") {
                coordEnd = true;
              }
            } else if (isspace(cc) || cc == ',') {
              if (currentToken.length() > 0) {
                if (compIdx < 3) {
                  components[compIdx++] = currentToken.toDouble();
                }
                currentToken = "";
                if (cc == ' ' || cc == '\n' || cc == '\r') {
                  if (compIdx >= 2) {
                    double lng = components[0];
                    double lat = components[1];
                    double alt = (compIdx == 3) ? components[2] : 0.0;
                    addPointToPool(lat, lng, alt);
                  }
                  compIdx = 0;
                }
              }
            } else {
              currentToken += cc;
            }
          }
        }
      }
    }
  }
  
  file.close();
  Serial.println("=== KML Parser: Streaming finished. Points: " + String(currentPointCount));
  Serial.printf("=== KML Parser: Final heap free: %u bytes\n", esp_get_free_heap_size());
  return currentPointCount > 0;
}

bool KMLParser::parseFile(const char* filePath) {
  reset();
  Serial.println("=== KML Parser: Starting streaming parse for " + String(filePath));
  return parseFileDirect(filePath);
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

bool KMLParser::isMemoryFull() {
  return currentPointCount >= MAX_POINTS;
}

Location KMLParser::getStartPoint() {
  if (currentPointCount > 0 && pointPool != nullptr) {
    return pointPool[0];
  }
  return Location(); // 返回默认的无效位置
}

