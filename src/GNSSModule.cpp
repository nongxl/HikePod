#include "GNSSModule.h"

// 初始化静态变量
uint32_t GNSSModule::gpsChars = 0;
uint32_t GNSSModule::gpsSentences = 0;

GNSSModule::GNSSModule() : 
  gpsSerial(1),
  rxPin(0),
  txPin(0),
  baudRate(9600),
  isInitialized(false),
  isInStandby(false),
  timezoneOffsetHours(0) {
}

void GNSSModule::begin(int rxPin, int txPin, long baudRate) {
  this->rxPin = rxPin;
  this->txPin = txPin;
  this->baudRate = baudRate;
  
  gpsSerial.begin(baudRate, SERIAL_8N1, rxPin, txPin);
  isInitialized = true;
}

void GNSSModule::end() {
  if (isInitialized) {
    gpsSerial.end();
    isInitialized = false;
  }
}

bool GNSSModule::update() {
  if (!isInitialized) return false;
  
  bool updated = false;
  
  // 检查是否有数据可用
  if (gpsSerial.available() > 0) {
    // 有数据可用，说明模块可能不在待机模式
    while (gpsSerial.available() > 0) {
      if (gps.encode(gpsSerial.read())) {
        gpsSentences++;
        if (gps.location.isValid()) {
          currentLocation = Location(
            gps.location.lat(),
            gps.location.lng(),
            gps.altitude.isValid() ? gps.altitude.meters() : 0
          );
          updated = true;
        } else {
          // 当GPS失去定位时，更新currentLocation为无效状态
          currentLocation.isValid = false;
        }
      } else {
        // 即使没有成功解析句子，也要增加字符计数器
        gpsChars++;
      }
    }
    
    // 如果模块之前被认为是待机模式，但现在有数据可用，更新状态
    if (isInStandby) {
      Serial.println("GNSS module is active (data available)");
      isInStandby = false;
    }
  } else {
      // 没有数据可用，可能意味着模块真的进入了待机模式
      // 但只有当模块之前已经成功发送过数据，并且我们明确发送了待机命令时，才认为它进入了待机模式
      // 这样可以避免在模块正常工作时误判为待机模式
      static bool hasReceivedData = false;
      
      // 检查是否已经接收到过数据
      if (gpsChars > 0 || gpsSentences > 0) {
        hasReceivedData = true;
      }
      
      // 只有当我们明确设置了待机模式，并且之前接收到过数据时，才输出待机模式的信息
      // 这样可以避免在模块正常工作时（只是暂时没有数据）误判为待机模式
      if (isInStandby && hasReceivedData) {
        // 这里只是一个猜测，实际模块可能只是暂时没有数据发送
        // 我们可以通过其他方式（如发送命令并检查响应）来确认
        Serial.println("GNSS module may be in standby mode (no data available)");
      }
    }
  
  return updated;
}

Location GNSSModule::getCurrentLocation() {
  return currentLocation;
}

int GNSSModule::getSatelliteCount() {
  return gps.satellites.value();
}

bool GNSSModule::isFixed() {
  return gps.location.isValid();
}

int GNSSModule::available() {
  if (!isInitialized) return 0;
  return gpsSerial.available();
}

char GNSSModule::read() {
  if (!isInitialized) return 0;
  char c = gpsSerial.read();
  gpsChars++;
  return c;
}

bool GNSSModule::isModuleInitialized() {
  return isInitialized;
}

double GNSSModule::getSpeedKmph() {
  return gps.speed.kmph();
}

double GNSSModule::getCourseDeg() {
  return gps.course.deg();
}

double GNSSModule::getHDOP() {
  return gps.hdop.hdop();
}

int GNSSModule::getDay() {
  return gps.date.day();
}

int GNSSModule::getMonth() {
  return gps.date.month();
}

int GNSSModule::getYear() {
  return gps.date.year();
}

int GNSSModule::getHour() {
  return gps.time.hour();
}

int GNSSModule::getMinute() {
  return gps.time.minute();
}

int GNSSModule::getSecond() {
  return gps.time.second();
}

bool GNSSModule::isDateValid() {
  return gps.date.isValid();
}

bool GNSSModule::isTimeValid() {
  return gps.time.isValid();
}

uint32_t GNSSModule::getGpsChars() {
  return gpsChars;
}

uint32_t GNSSModule::getGpsSentences() {
  return gpsSentences;
}

bool GNSSModule::feed(char c) {
  if (gps.encode(c)) {
    gpsSentences++;
    if (gps.location.isValid()) {
      currentLocation = Location(
        gps.location.lat(),
        gps.location.lng(),
        gps.altitude.isValid() ? gps.altitude.meters() : 0
      );
      return true;
    } else {
      currentLocation.isValid = false;
    }
  } else {
    gpsChars++;
  }
  return false;
}

// 发送命令到GNSS模块并读取响应
void GNSSModule::sendCommand(const String& command) {
  if (isInitialized) {
    Serial.printf("Sending command: %s\n", command.c_str());
    gpsSerial.println(command);
    gpsSerial.flush();
    
    // 读取响应
    unsigned long startTime = millis();
    String response = "";
    while (millis() - startTime < 1000) { // 等待最多1秒
      if (gpsSerial.available()) {
        char c = gpsSerial.read();
        if (c == '\n') {
          if (response.length() > 0) {
            Serial.printf("Received response: %s\n", response.c_str());
            response = "";
          }
        } else if (c != '\r') {
          response += c;
        }
      }
    }
    
    // 如果还有未处理的响应
    if (response.length() > 0) {
      Serial.printf("Received response: %s\n", response.c_str());
    }
  }
}

// 进入待机模式
void GNSSModule::enterStandbyMode() {
  // 对于ATGM336H-6N@AT6668模块，使用PCAS10命令进入待机模式
  sendCommand("$PCAS10,0*1C"); // 热启动命令
  Serial.println("GNSS module entered standby mode");
}

// 退出待机模式（热启动）
void GNSSModule::exitStandbyMode() {
  // 使用PCAS10命令热启动，退出待机模式
  sendCommand("$PCAS10,0*1C"); // 热启动命令
  Serial.println("GNSS module exited standby mode (hot start)");
}

// 检查是否处于待机模式
bool GNSSModule::isInStandbyMode() {
  // 这里只是一个状态跟踪，实际模块可能不会返回待机状态
  return isInStandby;
}

// 设置待机模式状态
void GNSSModule::setStandbyMode(bool standby) {
  isInStandby = standby;
}

int GNSSModule::getTimezoneOffset() {
  return timezoneOffsetHours;
}

void GNSSModule::setTimezoneOffset(int offsetHours) {
  timezoneOffsetHours = offsetHours;
}

void GNSSModule::calculateTimezoneFromLocation() {
  if (currentLocation.isValid) {
    timezoneOffsetHours = (int)round(currentLocation.longitude / 15.0);
    Serial.printf("Timezone calculated from longitude %.2f: UTC%+d\n", 
                  currentLocation.longitude, timezoneOffsetHours);
  }
}

int GNSSModule::getLocalHour() {
  if (!gps.time.isValid()) return 0;
  
  int localHour = gps.time.hour() + timezoneOffsetHours;
  
  if (localHour < 0) localHour += 24;
  else if (localHour >= 24) localHour -= 24;
  
  return localHour;
}

int GNSSModule::getLocalMinute() {
  return gps.time.minute();
}

int GNSSModule::getLocalSecond() {
  return gps.time.second();
}

int GNSSModule::getLocalDay() {
  if (!gps.date.isValid()) return 0;
  
  int hour = gps.time.hour() + timezoneOffsetHours;
  int day = gps.date.day();
  int month = gps.date.month();
  int year = gps.date.year();
  
  if (hour < 0) {
    hour += 24;
    day--;
    if (day < 1) {
      month--;
      if (month < 1) {
        month = 12;
        year--;
      }
      int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
      if (month == 2 && (year % 400 == 0 || (year % 100 != 0 && year % 4 == 0))) {
        daysInMonth[1] = 29;
      }
      day = daysInMonth[month - 1];
    }
  } else if (hour >= 24) {
    hour -= 24;
    day++;
    int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (month == 2 && (year % 400 == 0 || (year % 100 != 0 && year % 4 == 0))) {
      daysInMonth[1] = 29;
    }
    if (day > daysInMonth[month - 1]) {
      day = 1;
      month++;
      if (month > 12) {
        month = 1;
      }
    }
  }
  
  return day;
}

int GNSSModule::getLocalMonth() {
  if (!gps.date.isValid()) return 0;
  
  int hour = gps.time.hour() + timezoneOffsetHours;
  int month = gps.date.month();
  int year = gps.date.year();
  
  if (hour < 0) {
    month--;
    if (month < 1) {
      month = 12;
    }
  } else if (hour >= 24) {
    int day = gps.date.day();
    int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (month == 2 && (year % 400 == 0 || (year % 100 != 0 && year % 4 == 0))) {
      daysInMonth[1] = 29;
    }
    if (day > daysInMonth[month - 1]) {
      month++;
      if (month > 12) {
        month = 1;
      }
    }
  }
  
  return month;
}

int GNSSModule::getLocalYear() {
  if (!gps.date.isValid()) return 0;
  
  int hour = gps.time.hour() + timezoneOffsetHours;
  int year = gps.date.year();
  
  if (hour < 0) {
    int month = gps.date.month();
    if (month == 1) {
      year--;
    }
  } else if (hour >= 24) {
    int month = gps.date.month();
    int day = gps.date.day();
    int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (month == 2 && (year % 400 == 0 || (year % 100 != 0 && year % 4 == 0))) {
      daysInMonth[1] = 29;
    }
    if (day > daysInMonth[month - 1]) {
      month++;
      if (month > 12) {
        year++;
      }
    }
  }
  
  return year;
}
