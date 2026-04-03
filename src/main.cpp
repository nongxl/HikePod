/**
 * @file main.cpp
 * @brief HikePod - Outdoor Navigation for M5Stack Cardputer
 * 
 * Note: The "GPS Info" display logic and satellite calculation routines 
 * are ported/derived from the "Cardputer-GPS-Info" project by alcor55.
 * Original project: https://github.com/alcor55/Cardputer-GPS-Info
 * 
 * @author HikePod Contributors
 * @license MIT
 */

#include <M5Cardputer.h>
#include <TinyGPSPlus.h>
#include <SD.h>
#include <FS.h>
#include <vector>
#include <algorithm>
#include <M5GFX.h>
#include <WiFi.h>
#include "WiFiManager.h"

// 创建独立的 SPI 对象
SPIClass sdSPI;

// 模块定义
#include "GNSSModule.h"
#include "KMLParser.h"
#include "RenderEngine.h"
#include "InteractionManager.h"
#include "TrackingManager.h"

// 离屏渲染精灵对象
M5Canvas canvas(&M5Cardputer.Display);

// 全局对象
GNSSModule gnssModule;
KMLParser* kmlParser = nullptr;
RenderEngine renderEngine;
InteractionManager interactionManager;
TrackingManager trackingManager;
WiFiManager wifiManager;

// 卫星数据结构体
struct SatData {
  String system;   // "GPS", "GLONASS", "Galileo", "BeiDou".
  int id;
  int elevation;   // 0-90°.
  int azimuth;     // 0-359°.
  int snr;         // 0-99.
  bool used;       // used in the fix.
  bool visible;    // visible in the last cycle.
};

// 卫星数据存储
std::vector<SatData> satellites;

// GSV序列状态
struct GSVSequenceState {
    String system;
    int totalMsgs = 0;
    int lastMsgNum = 0;
    std::vector<int> currentVisible;
};

GSVSequenceState gsvStates[5];
int gsvCount = 0;

// 函数声明
void setSystemTimeFromGPS();
void drawSatelliteDataTab();
void drawSkyPlot();
void initGPSSerial(bool should_I);
void nmeaDispatcher(const String &nmeaLine);
void parseGSV(const String &line);
void parseGSA(const String &line);
GSVSequenceState* getGSVState(const String& system);
void storeSatellite(const SatData &sat);
void serialGPSRead();
void updateScreen(bool force = false);
void drawHeader();
void drawStatus();
void handleControls(bool keyboardChanged, bool keyboardPressed, Keyboard_Class::KeysState keys);
void handleKeys(bool keyboardChanged, bool keyboardPressed, Keyboard_Class::KeysState keys);
void handleGPSInfoKeys(bool keyboardChanged, bool keyboardPressed, Keyboard_Class::KeysState keys);
void showGPSNoFixAlert();
void showKMLFullAlert();
void drawHikePodHelpMenu(bool should_I);
void drawSettingsMenu(bool should_I);
void drawConfig(bool should_I);
void drawHelp(bool should_I);
void drawInfo(bool should_I);
void drawHttpServerWindow(bool should_I);
// void initHttpServer(); // 移除旧函数
// void stopHttpServer(); // 移除旧函数

// SD卡状态
bool sdInitialized = false;
bool hasRoute = false;

// 当前位置
Location currentLocation;

// 轨迹数据
std::vector<Location> routePoints;

// 内存池信息（用于绘制完整路径）
const Location* pointPool = nullptr;
int totalPoints = 0;

// 电量历史数据，用于绘制电量消耗曲线
std::vector<int> batteryHistory;
const int MAX_BATTERY_HISTORY = 360; // 最多存储360个数据点（6小时，每分钟一次）
unsigned long lastBatteryRecordTime = 0;
const unsigned long BATTERY_RECORD_INTERVAL = 60000; // 每分钟记录一次电量

// 屏幕尺寸
const int SCREEN_WIDTH = 240;
const int SCREEN_HEIGHT = 135;

// 亮度控制
int screenBrightness = 64; // 默认降低亮度到64（0-255）
const int BRIGHTNESS_MIN = 10; // 最小亮度
const int BRIGHTNESS_MAX = 255; // 最大亮度
const int BRIGHTNESS_STEP = 16; // 亮度调节步长

// 息屏控制
unsigned long lastActivityTime = 0; // 上次活动时间
unsigned long SCREEN_TIMEOUT = 30000; // 30秒无操作息屏，可在设置中调整
bool isScreenOff = false; // 屏幕是否关闭
unsigned long screenOffTime = 0; // 屏幕关闭的时间
const unsigned long GNSS_STANDBY_DELAY = 10000; // 息屏10秒后让GNSS进入待机模式
bool lastGNSSStandbyState = false; // 跟踪上次的GNSS待机状态（用于日志输出）

// 系统时间控制
bool systemTimeSetFromGPS = false; // 系统时间是否已从GPS设置

// 定位频率控制
unsigned long lastGPSUpdateTime = 0; // 上次GPS更新时间
unsigned long GPS_UPDATE_INTERVAL_NORMAL = 2000; // 正常模式下的GPS更新间隔（2秒）
unsigned long GPS_UPDATE_INTERVAL_SCREEN_OFF = 10000; // 息屏模式下的GPS更新间隔（10秒）
const unsigned long GPS_UPDATE_INTERVAL_SEARCH = 500; // 搜星模式下的GPS更新间隔（500毫秒，更高刷新率）
bool isGNSSSearching = true; // GNSS是否正在搜索定位

// 省电菜单控制
int powerMenuSelection = 0; // 当前选中的省电选项
const int POWER_MENU_OPTIONS = 3; // 省电选项数量

// 新设置菜单控制
bool settingsMenuOpen = false; // 设置菜单是否打开
bool gpsNoFixAlertVisible = false; // GPS未定位提示信息框是否可见
bool kmlFullAlertVisible = false; // KML点数达到上限提示信息框是否可见
bool helpMenuVisible = false; // 帮助菜单是否可见
int settingsMenuSelection = 0; // 当前选中的设置选项
const int SETTINGS_MENU_OPTIONS = 6; // 设置选项数量（文件 + 亮度 + 超时 + 2个频率 + POI开关）
String currentKmlFile = ""; // 当前加载的 KML 文件名
int showPOIsMode = 2;      // 关键点显示模式 (0:OFF, 1:ON, 2:AUTO)

// WiFi KML 管理窗口显示标志
bool httpServerMenuOpen = false;
// WebServer server(80); // 已移除
// String httpStatusMsg = ""; // 已移除

// 模式定义
enum AppMode {
  MODE_HIKEPOD,    // 徒步路线模式
  MODE_GPS_INFO    // GPS信息模式
};

// 当前模式
AppMode currentMode = MODE_HIKEPOD;

// 当前视图模式
ViewMode currentViewMode = MODE_2D;

// 模式切换标志
bool modeChanged = false;
bool viewModeChanged = false;

// BMI270姿态传感器相关
bool imuInitialized = false;
double currentPitch = 0.0;
double currentRoll = 0.0;
double lastPitch = 0.0;
double lastRoll = 0.0;
const double IMU_FILTER_ALPHA = 0.1; // 低通滤波器系数

// BMI270相关函数
void initBMI270() {
  // 使用M5Unified内置的IMU支持
  imuInitialized = M5.Imu.begin();
  
  if (imuInitialized) {
    Serial.println("BMI270 IMU initialized successfully");
  } else {
    Serial.println("BMI270 IMU initialization failed");
  }
}

void readBMI270Data() {
  if (!imuInitialized) return;
  
  // 使用M5Unified内置的IMU更新
  auto imu_update = M5.Imu.update();
  if (imu_update) {
    // 获取IMU数据
    auto imu_data = M5.Imu.getImuData();
    
    // 使用加速度数据计算pitch和roll
    // M5Cardputer坐标系：
    // X轴：屏幕水平向右
    // Y轴：屏幕垂直向上  
    // Z轴：屏幕向外（面向用户）
    // 
    // Pitch: 绕X轴旋转（前后倾斜）- Y和Z加速度变化
    // Roll: 绕Y轴旋转（左右倾斜）- X和Z加速度变化
    currentPitch = atan2(imu_data.accel.y, imu_data.accel.z);
    currentRoll = atan2(-imu_data.accel.x, sqrt(imu_data.accel.y * imu_data.accel.y + imu_data.accel.z * imu_data.accel.z));
  }
}

void updateOrientation() {
  if (!imuInitialized) return;
  
  readBMI270Data();
  
  double filteredPitch = lastPitch * (1.0 - IMU_FILTER_ALPHA * 2) + currentPitch * IMU_FILTER_ALPHA * 2;
  double filteredRoll = lastRoll * (1.0 - IMU_FILTER_ALPHA * 2) + currentRoll * IMU_FILTER_ALPHA * 2;
  
  double maxChange = 0.05;
  if (fabs(filteredPitch - lastPitch) > maxChange) {
    filteredPitch = lastPitch + (filteredPitch > lastPitch ? maxChange : -maxChange);
  }
  if (fabs(filteredRoll - lastRoll) > maxChange) {
    filteredRoll = lastRoll + (filteredRoll > lastRoll ? maxChange : -maxChange);
  }
  
  lastPitch = filteredPitch;
  lastRoll = filteredRoll;
  
  if (currentViewMode == MODE_3D) {
    auto imu_data = M5.Imu.getImuData();
    double accelX = imu_data.accel.x / 9.8;
    double accelY = imu_data.accel.y / 9.8;
    renderEngine.updateCameraOrientation(filteredPitch, filteredRoll, accelX, accelY);
  } else {
    renderEngine.setOrientation(filteredPitch, filteredRoll);
  }
}

// 文件选择菜单相关变量
bool fileSelectionMenuOpen = false;
int selectedFileIndex = 0;
std::vector<String> kmlFileList;
bool menuJustOpened = false; // 用于跟踪文件选择菜单是否刚刚打开

// Cardputer_GPS_Info 相关变量

// 串口和菜单状态
bool gpsSerial = false;
bool debugSerial = false;
bool nmeaSerial = false;
bool satListSerial = false;
bool hidePlotId = true;
bool hidePlotSystem = true;
bool openMenu = false;
bool helpMenu = false;
bool infoMenu = false;
bool configsMenu = false;
int configsMenuSel = 0;
String configsTmp[3] = {"", "", ""};  // 0 Rx, 1 Tx, 2 Baud.

// GPS引脚和波特率
int gpsRxPin = 15; // Cardputer Rx pin <- GPS Tx pin.
int gpsTxPin = 13; // Cardputer Tx pin <- GPS Rx pin.
// 将GPS波特率从9600修改为115200，以匹配模块默认波特率
int gpsBaud = 115200;

// GPS状态
enum GPSState { GPS_OFF, GPS_ON, GPS_ERR };
GPSState gpsSerialState = GPS_OFF;
const unsigned long GPS_TIMEOUT = 120000; // 增加到120秒，适应GPS模块长时间运行需求
unsigned long lastValidGpsMillis = 0;


// 新增函数：列出HikePod文件夹中的.kml文件
std::vector<String> listKMLFiles() {
  std::vector<String> kmlFiles;
  
  if (sdInitialized) {
    if (SD.exists("/HikePod")) {
      File hikePodDir = SD.open("/HikePod");
      if (hikePodDir) {
        while (true) {
          File entry = hikePodDir.openNextFile();
          if (!entry) {
            break;
          }
          if (!entry.isDirectory()) {
            String fileName = entry.name();
            if (fileName.endsWith(".kml")) {
              kmlFiles.push_back(fileName);
              Serial.println("Found KML file: " + fileName);
            }
          }
          entry.close();
        }
        hikePodDir.close();
      } else {
        Serial.println("Failed to open HikePod directory");
      }
    } else {
      Serial.println("HikePod directory does not exist");
    }
  } else {
    Serial.println("SD card not initialized");
  }
  
  return kmlFiles;
}

// 新增函数：显示文件选择菜单
void drawFileSelectionMenu() {
  // 设置菜单状态为打开
  openMenu = true;
  // 白色背景
  canvas.fillScreen(TFT_WHITE);
  
  // 绘制菜单标题
  canvas.setTextColor(TFT_BLUE, TFT_WHITE);
  canvas.setFont(&fonts::efontCN_12);
  canvas.setCursor(10, 10);
  canvas.println("Select KML File");
  
  // 绘制分隔线
  canvas.drawLine(10, 25, SCREEN_WIDTH - 10, 25, TFT_BLACK);
  
  // 绘制文件列表
  canvas.setTextColor(TFT_BLACK, TFT_WHITE);
  
  int yPos = 35;
  int maxVisibleFiles = 6; // 中文字体较大，减少显示行数
  int startIndex = 0;
  
  int totalItems = (int)kmlFileList.size();
  
  if (totalItems <= maxVisibleFiles) {
    startIndex = 0;
  } else {
    int cursorOffset = maxVisibleFiles / 2;
    
    if (selectedFileIndex < cursorOffset) {
      startIndex = 0;
    } else if (selectedFileIndex >= totalItems - cursorOffset) {
      startIndex = totalItems - maxVisibleFiles;
    } else {
      startIndex = selectedFileIndex - cursorOffset;
    }
  }
  
  for (size_t i = startIndex; i < kmlFileList.size() && i < startIndex + maxVisibleFiles; i++) {
    if (i == selectedFileIndex) {
      // 绘制选中项 - 使用蓝色高亮
      canvas.fillRect(10, yPos - 2, SCREEN_WIDTH - 20, 16, TFT_BLUE);
      canvas.setTextColor(TFT_WHITE, TFT_BLUE);
    } else {
      canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    }
    
    canvas.setCursor(15, yPos);
    canvas.println(kmlFileList[i]);
    yPos += 16; // 增加行高以适应中文字体
  }
  
  // 绘制滚动条（当文件数量超过最大可见数量时）
  if (kmlFileList.size() > maxVisibleFiles) {
    int scrollbarX = SCREEN_WIDTH - 8;
    int scrollbarY = 35;
    int scrollbarHeight = maxVisibleFiles * 12;
    int scrollbarWidth = 4;
    
    // 绘制滚动条背景（浅灰色）
    canvas.fillRect(scrollbarX, scrollbarY, scrollbarWidth, scrollbarHeight, 0xC618);
    
    // 计算滚动条滑块位置和高度
    float totalItems = kmlFileList.size();
    float visibleItems = maxVisibleFiles;
    float scrollRatio = startIndex / (totalItems - visibleItems);
    float thumbHeight = (visibleItems / totalItems) * scrollbarHeight;
    float thumbY = scrollbarY + scrollRatio * (scrollbarHeight - thumbHeight);
    
    // 绘制滚动条滑块（深灰色）
    canvas.fillRect(scrollbarX, thumbY, scrollbarWidth, thumbHeight, 0x7BEF);
  }
  
  canvas.pushSprite(0, 0);
}

// 新增函数：加载选中的KML文件
void loadSelectedKMLFile(const String& fileName) {
  String filePath = "/HikePod/" + fileName;
  Serial.println("Loading KML file: " + filePath);
  
  // 在加载新 KML 前，强制清理 3D 渲染占用的内存并回到 2D 模式，确保内存池分配有足够空间
  renderEngine.releaseWorldPoints();
  if (currentViewMode == MODE_3D) {
    currentViewMode = MODE_2D;
    renderEngine.setViewMode(MODE_2D);
    renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, nullptr, 0, nullptr, 0, showPOIsMode);
    canvas.pushSprite(0, 0);
  }
  
  // 如果已经存在解析器，先释放旧的内存池
currentKmlFile = fileName; // 保存当前加载的文件名
  
  // 显示加载提示
  canvas.fillScreen(TFT_BLACK);
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  canvas.setTextSize(1);
  canvas.drawCenterString("Loading KML Path...", SCREEN_WIDTH/2, SCREEN_HEIGHT/2 - 10);
  canvas.drawCenterString(fileName.c_str(), SCREEN_WIDTH/2, SCREEN_HEIGHT/2 + 5);
  canvas.pushSprite(0, 0);
  
  // 动态分配内存以节省启动时的栈空间
  if (kmlParser) delete kmlParser;
  kmlParser = new (std::nothrow) KMLParser();
  
  if (!kmlParser || kmlParser->getPointPool() == nullptr) {
    Serial.println("Failed to allocate memory for KML parser");
    canvas.fillScreen(TFT_BLACK);
    canvas.setTextColor(TFT_RED, TFT_BLACK);
    canvas.drawCenterString("Memory Error!", SCREEN_WIDTH/2, SCREEN_HEIGHT/2);
    canvas.pushSprite(0, 0);
    delay(2000);
    hasRoute = false;
    return;
  }
  
  if (kmlParser->parseFile(filePath.c_str())) {
    int pointCount = kmlParser->getPointCount();
    if (pointCount > 0) {
      hasRoute = true;
      Serial.println("KML file loaded successfully with " + String(pointCount) + " points");
      
      // 检查是否达到点数上限
      if (kmlParser->isMemoryFull()) {
        Serial.println("KML file hit memory limit, showing alert");
        showKMLFullAlert();
      }
      
      // 获取起点坐标并传递给RenderEngine
      Location startPoint = kmlParser->getStartPoint();
      renderEngine.setStartPoint(startPoint);
      Serial.printf("KML start point: Lat=%.6f, Lng=%.6f\n", startPoint.latitude, startPoint.longitude);
      
      // 更新内存池信息
      pointPool = kmlParser->getPointPool();
      totalPoints = pointCount;
      
      // 通知RenderEngine重新构建3D世界坐标
      renderEngine.invalidateWorldPoints();
      
      // 计算边界框
      if (totalPoints > 0) {
        renderEngine.calculateBoundingBoxFromPool(pointPool, totalPoints);
        renderEngine.autoFitToRoute();
        
        // 同步缩放级别和平移到交互管理器，确保后续按键逻辑同步
        interactionManager.setZoomLevel(renderEngine.getZoomLevel());
        int px, py;
        renderEngine.getPanOffset(px, py);
        interactionManager.setPanOffset(px, py);
        
        Serial.println("Bounding box calculation and auto-fit completed, InteractionManager synced");
      }
      
      // 只获取少量点用于显示，避免栈溢出
      routePoints.clear();
      int pointsToAdd = min(pointCount, 100); // 只添加100个点
      for (int i = 0; i < pointsToAdd; i++) {
        routePoints.push_back(pointPool[i]);
      }
      Serial.println("Added " + String(routePoints.size()) + " points to routePoints for display");
    } else {
      Serial.println("KML file parsed but no route points found");
      hasRoute = false;
    }
  } else {
    Serial.println("Failed to parse KML file");
    hasRoute = false;
  }
  
  // 关闭文件选择菜单
  fileSelectionMenuOpen = false;
  
  // 重新渲染界面
  renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
  canvas.pushSprite(0, 0);
}

// 从GPS获取时间并设置系统时间
void setSystemTimeFromGPS() {
  if (systemTimeSetFromGPS) {
    return; // 已经设置过，不需要重复设置
  }
  
  // 检查GPS日期和时间是否有效
  if (gnssModule.isDateValid() && gnssModule.isTimeValid()) {
    if (currentLocation.isValid) {
      gnssModule.calculateTimezoneFromLocation();
    }
    
    struct tm timeinfo;
    timeinfo.tm_year = gnssModule.getLocalYear() - 1900;
    timeinfo.tm_mon = gnssModule.getLocalMonth() - 1;
    timeinfo.tm_mday = gnssModule.getLocalDay();
    timeinfo.tm_hour = gnssModule.getLocalHour();
    timeinfo.tm_min = gnssModule.getLocalMinute();
    timeinfo.tm_sec = gnssModule.getLocalSecond();
    timeinfo.tm_isdst = 0;
    
    time_t t = mktime(&timeinfo);
    struct timeval tv = { .tv_sec = t, .tv_usec = 0 };
    settimeofday(&tv, nullptr);
    
    systemTimeSetFromGPS = true;
    Serial.printf("[Time] System time set from GPS (UTC%+d): %04d-%02d-%02d %02d:%02d:%02d\n",
                 gnssModule.getTimezoneOffset(),
                 gnssModule.getLocalYear(), gnssModule.getLocalMonth(), gnssModule.getLocalDay(),
                 gnssModule.getLocalHour(), gnssModule.getLocalMinute(), gnssModule.getLocalSecond());
  }
}

void initHttpServer() {
  wifiManager.begin([]() {
    // 释放路径点内存以确保 WiFi 启动空间
    if (kmlParser) {
      delete kmlParser;
      kmlParser = nullptr;
      pointPool = nullptr;
      totalPoints = 0;
      renderEngine.releaseWorldPoints();
      Serial.println("Memory released for WiFi startup");
    }
  });
}

void stopHttpServer() {
  wifiManager.stop();
  httpServerMenuOpen = false;
  
  // 增加延迟和 yield 确保 WiFi 栈有足够时间清理堆内存和控制块
  Serial.println("WiFi stopped, waiting for resource cleanup...");
  for (int i = 0; i < 6; i++) {
    delay(200);
    yield();
  }
  
  // WiFi 关闭后，如果之前有加载 KML，则自动重载
  if (currentKmlFile != "" && !kmlParser) {
    Serial.println("Reloading KML after WiFi: " + currentKmlFile);
    loadSelectedKMLFile(currentKmlFile);
  } else if (currentKmlFile != "") {
    Serial.println("KML already loaded, skipping reload.");
  }
}

void setup() {
  // 首先初始化串口通信用于调试
  Serial.begin(115200);
  Serial.println("Starting HikePod setup...");
  
  // 初始化M5Cardputer（按照M5Mp3的方式，先初始化M5Cardputer）
  Serial.println("Initializing M5Cardputer...");
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);  // 启用键盘 - Cardputer ADV正确方式
  
  // 增加启动延迟以稳定电源和 I2C 总线，并清空初始可能的随机按键输入（解决幽灵按键问题）
  delay(300);
  for (int i = 0; i < 10; i++) {
    M5Cardputer.update();
    delay(10);
  }
  
  M5Cardputer.Display.setBrightness(screenBrightness);
  Serial.printf("Screen brightness set to: %d\n", screenBrightness);
  
  // 初始化离屏渲染画布
  canvas.createSprite(SCREEN_WIDTH, SCREEN_HEIGHT);
  
  // 初始化电量历史数据，添加初始数据点
  int initialBattery = M5Cardputer.Power.getBatteryLevel();
  batteryHistory.push_back(initialBattery);
  lastBatteryRecordTime = millis();
  Serial.printf("Initial battery level: %d%% (history size: %d)\n", initialBattery, batteryHistory.size());
  
  lastValidGpsMillis = millis();
  lastActivityTime = millis(); // 初始化上次活动时间
  
  // 初始化SD卡（使用独立的SPI对象）
  
  // 永久禁用LoRa模块的SPI通信，因为项目不需要LoRa功能
  // 根据管脚映射，CAP-LoRa-1262模块的NSS引脚连接到Cardputer-Adv的G5
  #define LORA_CS 5
  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH); // 设置为HIGH以永久禁用LoRa模块的SPI通信
  Serial.println("Permanently disabled LoRa module SPI communication (project doesn't require LoRa functionality)");
  
  // 额外的延迟，确保LoRa模块完全禁用
  //delay(100);
  
  // 同时禁用LoRa模块的其他相关引脚，进一步减少电源消耗
  #define LORA_RST 8  // LoRa_RST连接到G8
  #define LORA_IRQ 6  // LoRa_IRQ连接到G6
  #define LORA_BUSY 10 // LoRa_BUSY连接到G10
  
  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, LOW); // 设置为LOW以保持LoRa模块复位状态
  pinMode(LORA_IRQ, INPUT);
  pinMode(LORA_BUSY, INPUT);
  Serial.println("Disabled additional LoRa module pins to reduce power consumption");
  
  // 使用官方推荐的SD卡引脚配置
  #define SD_SCK 40
  #define SD_MISO 39
  #define SD_MOSI 14
  #define SD_CS 12
  
  // 初始化独立的SPI对象
  Serial.println("Initializing SPI with pins - SCK: " + String(SD_SCK) + ", MISO: " + String(SD_MISO) + ", MOSI: " + String(SD_MOSI) + ", CS: " + String(SD_CS));
  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  //delay(50); // 添加适当的延迟
  
  // 尝试初始化SD卡（使用标准SD库和独立SPI对象）
  Serial.println("Attempting SD card initialization with independent SPI object...");
  sdInitialized = false;
  hasRoute = false;
  
  if (SD.begin(SD_CS, sdSPI)) {
    Serial.println("SD card initialized successfully with independent SPI object");
    sdInitialized = true;
    
    // 检查SD卡类型
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
      Serial.println("No SD card attached");
      sdInitialized = false;
    } else {
      Serial.print("SD Card Type: ");
      if (cardType == CARD_MMC) {
        Serial.println("MMC");
      } else if (cardType == CARD_SD) {
        Serial.println("SDSC");
      } else if (cardType == CARD_SDHC) {
        Serial.println("SDHC");
      } else {
        Serial.println("UNKNOWN");
      }
      
      // 检查SD卡根目录
      Serial.println("Listing root directory contents:");
      File root = SD.open("/");
      if (root) {
        while (true) {
          File entry = root.openNextFile();
          if (!entry) {
            break;
          }
          Serial.print(entry.name());
          if (entry.isDirectory()) {
            Serial.println("/");
          } else {
            Serial.print(" ");
            Serial.println(entry.size());
          }
          entry.close();
        }
        root.close();
      } else {
        Serial.println("Failed to open root directory");
      }
      
      // 检查HikePod目录是否存在
      if (SD.exists("/HikePod")) {
        Serial.println("HikePod directory exists");
        // 列出HikePod目录内容
        Serial.println("Listing HikePod directory contents:");
        File hikePodDir = SD.open("/HikePod");
        if (hikePodDir) {
          while (true) {
            File entry = hikePodDir.openNextFile();
            if (!entry) {
              break;
            }
            Serial.print(entry.name());
            if (entry.isDirectory()) {
              Serial.println("/");
            } else {
              Serial.print(" ");
              Serial.println(entry.size());
            }
            entry.close();
          }
          hikePodDir.close();
        } else {
          Serial.println("Failed to open HikePod directory");
        }
      } else {
        Serial.println("HikePod directory does not exist");
      }
      
      // 不再自动加载KML文件，只在用户选择后加载
    }
  } else {
    Serial.println("ERROR: SD Mount Failed with independent SPI object!");
    sdInitialized = false;
    
    // 尝试使用默认SPI对象作为后备
    Serial.println("Attempting fallback to default SPI object...");
    SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
    //delay(500);
    if (SD.begin(SD_CS)) {
      Serial.println("SD card initialized successfully with default SPI object");
      sdInitialized = true;
      
      // 检查SD卡类型
      uint8_t cardType = SD.cardType();
      if (cardType == CARD_NONE) {
        Serial.println("No SD card attached");
        sdInitialized = false;
      } else {
        Serial.print("SD Card Type: ");
        if (cardType == CARD_MMC) {
          Serial.println("MMC");
        } else if (cardType == CARD_SD) {
          Serial.println("SDSC");
        } else if (cardType == CARD_SDHC) {
          Serial.println("SDHC");
        } else {
          Serial.println("UNKNOWN");
        }
        
        // 检查SD卡根目录
        Serial.println("Listing root directory contents:");
        File root = SD.open("/");
        if (root) {
          while (true) {
            File entry = root.openNextFile();
            if (!entry) {
              break;
            }
            Serial.print(entry.name());
            if (entry.isDirectory()) {
              Serial.println("/");
            } else {
              Serial.print(" ");
              Serial.println(entry.size());
            }
            entry.close();
          }
          root.close();
        } else {
          Serial.println("Failed to open root directory");
        }
        
        // 检查HikePod目录是否存在
        if (SD.exists("/HikePod")) {
          Serial.println("HikePod directory exists");
          // 列出HikePod目录内容
          Serial.println("Listing HikePod directory contents:");
          File hikePodDir = SD.open("/HikePod");
          if (hikePodDir) {
            while (true) {
              File entry = hikePodDir.openNextFile();
              if (!entry) {
                break;
              }
              Serial.print(entry.name());
              if (entry.isDirectory()) {
                Serial.println("/");
              } else {
                Serial.print(" ");
                Serial.println(entry.size());
              }
              entry.close();
            }
            hikePodDir.close();
          } else {
            Serial.println("Failed to open HikePod directory");
          }
        } else {
          Serial.println("HikePod directory does not exist");
        }
        
        // 不再自动加载KML文件，只在用户选择后加载
      }
    } else {
      Serial.println("ERROR: SD Mount Failed with default SPI object too!");
      sdInitialized = false;
    }
  }
  
  if(!sdInitialized) {
    Serial.println("SD card initialization failed");
  }
  
  // 初始化空路线
  routePoints.clear();
  hasRoute = false;
  Serial.println("Initialized with empty route - KML file will be loaded via 'c' key");
  
  // 初始化内存池信息
  pointPool = nullptr;
  totalPoints = 0;
  
  Serial.println("HikePod GPS Info Mode Started");
  
  // 初始化渲染引擎
  renderEngine.begin(SCREEN_WIDTH, SCREEN_HEIGHT);
  renderEngine.setCanvas(&canvas);  // 设置canvas用于离屏渲染
  renderEngine.setGNSSModule(&gnssModule);  // 设置GNSS模块引用
  
  Serial.println("=== HikePod: efont (Solution A) balanced for Chinese support");

  // 初始化BMI270姿态传感器
  Serial.println("Initializing BMI270 IMU...");
  initBMI270();
  
  // 初始化交互管理器
    interactionManager.begin();
    
    // 初始化跟踪管理器
    trackingManager.begin();
    trackingManager.setGNSSModule(&gnssModule);
    
    // 初始化GPS串口状态
    gpsSerialState = GPS_OFF;
    
    // GPS模块将在用户按下s键时通过initGPSSerial函数初始化
    // 这样可以避免状态不一致的问题
    
    // 初始化屏幕
  if (currentMode == MODE_GPS_INFO) {
    canvas.fillScreen(TFT_BLACK);
    drawHeader();
    drawStatus();
    drawSatelliteDataTab();
    drawSkyPlot();
    canvas.pushSprite(0, 0);  // 一次性刷新到屏幕
  }
  
  // 默认开启GPS
  Serial.println("Initializing GPS module...");
  initGPSSerial(true);
  gpsSerial = true;
  gpsSerialState = GPS_ON;
  Serial.println("GPS module initialized and started");
}

void loop() {
  // 更新Cardputer状态（键盘、按钮、传感器等）
  M5Cardputer.update();
  
  // 处理 HTTP 服务器请求
  wifiManager.handle();
  
  // 更新BMI270姿态传感器数据
  updateOrientation();
  
  // 检查键盘状态变化
  bool keyboardChanged = M5Cardputer.Keyboard.isChange();
  bool keyboardPressed = M5Cardputer.Keyboard.isPressed();
  Keyboard_Class::KeysState keys;
  
  // 每次都获取最新的keys状态，而不是只在keyboardChanged && keyboardPressed为true时才获取
  // 这样可以确保InteractionManager能够检测到所有按键输入，包括平移操作按键
  keys = M5Cardputer.Keyboard.keysState();
  
  // 处理键盘输入控制逻辑
  handleControls(keyboardChanged, keyboardPressed, keys);

  // 检查用户活动
  if (keyboardChanged || keyboardPressed) {
    lastActivityTime = millis();
    // 如果屏幕是关闭的，按任意键恢复亮屏
    if (isScreenOff) {
      M5Cardputer.Display.setBrightness(screenBrightness);
      isScreenOff = false;
      Serial.println("Screen turned on");
    }
  }
  
  // 检查是否需要息屏
  if (SCREEN_TIMEOUT > 0 && !isScreenOff && millis() - lastActivityTime > SCREEN_TIMEOUT) {
    M5Cardputer.Display.setBrightness(0); // 关闭屏幕
    isScreenOff = true;
    screenOffTime = millis(); // 记录屏幕关闭时间
    Serial.println("Screen turned off due to inactivity");
  }
  
  // 息屏10秒后让GNSS模块进入待机模式
  // 但是如果GNSS正在搜索卫星或正在tracking，即使屏幕关闭了，也不应该让GNSS模块进入待机模式
  if (isScreenOff && millis() - screenOffTime > GNSS_STANDBY_DELAY && !isGNSSSearching && !trackingManager.isTracking()) {
    if (!gnssModule.isInStandbyMode()) {
      gnssModule.enterStandbyMode();
      gnssModule.setStandbyMode(true);
    }
  }
  
  // 屏幕打开时，退出GNSS待机模式
  if (!isScreenOff && gnssModule.isInStandbyMode()) {
    gnssModule.exitStandbyMode();
    gnssModule.setStandbyMode(false);
  }
  
  // 定期记录电量数据，用于绘制电量消耗曲线
  unsigned long currentTime = millis();
  if (currentTime - lastBatteryRecordTime > BATTERY_RECORD_INTERVAL) {
    int currentBattery = M5Cardputer.Power.getBatteryLevel();
    batteryHistory.push_back(currentBattery);
    
    // 限制历史数据点数量
    if (batteryHistory.size() > MAX_BATTERY_HISTORY) {
      batteryHistory.erase(batteryHistory.begin());
    }
    
    lastBatteryRecordTime = currentTime;
    Serial.printf("Battery level recorded: %d%% (history size: %d)\n", currentBattery, batteryHistory.size());
  }
  
  // 根据当前模式执行不同功能
  if (currentMode == MODE_HIKEPOD && !httpServerMenuOpen) {
    // 将交互控制逻辑完全交由 handleControls 处理
    interactionManager.update(keyboardChanged, keyboardPressed, keys);
    
    // 检查是否需要重绘
    bool needRender = false;
    
    static bool hasUserPanned = false;  // 跟踪用户是否已手动操作过地图
    static bool initialPositionSet = false;  // 跟踪是否已设置初始位置
    static Location prevLocation; // 用于跟踪GPS位置变化
    static bool prevLocationInitialized = false;  // 跟踪prevLocation是否已初始化
    
    if (!prevLocationInitialized) {
      prevLocation.latitude = 0;
      prevLocation.longitude = 0;
      prevLocation.altitude = 0;
      prevLocation.isValid = false;
      prevLocationInitialized = true;
    }
    
    // 根据GNSS搜索状态、屏幕状态和待机模式控制GPS更新频率
    unsigned long currentTime = millis();
    unsigned long gpsInterval;
    
    // 检查GNSS模块是否处于待机模式
    bool currentGNSSStandbyState = gnssModule.isInStandbyMode();
    if (currentGNSSStandbyState) {
      // 待机模式下，不执行GPS更新
      // 只有当状态变化时才输出日志
      if (currentGNSSStandbyState != lastGNSSStandbyState) {
        Serial.println("GNSS in standby mode, skipping update");
        lastGNSSStandbyState = currentGNSSStandbyState;
      }
    } else {
      // 只有当状态变化时才输出日志
      if (currentGNSSStandbyState != lastGNSSStandbyState) {
        Serial.println("GNSS exited standby mode, resuming update");
        lastGNSSStandbyState = currentGNSSStandbyState;
      }
      if (isGNSSSearching) {
        gpsInterval = GPS_UPDATE_INTERVAL_SEARCH; // 搜索模式使用更高频率
      } else {
        gpsInterval = isScreenOff ? GPS_UPDATE_INTERVAL_SCREEN_OFF : GPS_UPDATE_INTERVAL_NORMAL;
      }
      
      if (currentTime - lastGPSUpdateTime > gpsInterval) {
        // 读取GNSS数据
        gnssModule.update();
        // 无论是否有新数据，都获取当前位置，以确保currentLocation始终是最新的
        currentLocation = gnssModule.getCurrentLocation();
        lastGPSUpdateTime = currentTime;
        
        // 如果获取到GPS时间，设置系统时间
        if (!systemTimeSetFromGPS) {
          setSystemTimeFromGPS();
        }
        
        // 如果GPS位置有效，确保时区已计算
        if (currentLocation.isValid) {
          gnssModule.calculateTimezoneFromLocation();
        }
        
        // 如果正在跟踪，更新轨迹记录
        if (trackingManager.isTracking()) {
          trackingManager.updateTracking(currentLocation);
        }
        
        // 检查是否获取到定位，如果获取到则结束搜索模式
        if (isGNSSSearching && currentLocation.isValid) {
          isGNSSSearching = false;
          Serial.println("GNSS acquired fix, switching to normal update interval");
        }
      }
    }
    
    // 检查用户是否进行了手动操作（缩放或平移）
    bool userInteracted = interactionManager.isZoomChanged() || interactionManager.isPanChanged();
    
    // 检查空格键是否被按下
    if (interactionManager.isSpaceKeyPressed()) {
      // 切换定位点锁定状态
      bool currentLocked = renderEngine.isLocationLockedState();
      renderEngine.setLocationLocked(!currentLocked);
      
      if (!currentLocked) {
        Serial.println("[Location] Location locked");
      } else {
        Serial.println("[Location] Location unlocked");
      }
      
      // 重置空格键标志
      interactionManager.resetSpaceKeyPressed();
    }
    
    // 检查t键是否被按下，用于启动/停止tracking模式
    if (interactionManager.isTKeyPressed()) {
      if (trackingManager.isTracking()) {
        // 停止tracking
        trackingManager.stopTracking();
        renderEngine.setTrackingState(false);
        Serial.println("[Tracking] Tracking stopped");
      } else {
        // 检查GPS定位是否有效
        if (!currentLocation.isValid) {
          // GPS未定位，显示提示信息框
          showGPSNoFixAlert();
        } else {
          // 启动tracking
          if (trackingManager.startTracking()) {
            renderEngine.setTrackingState(true);
            Serial.println("[Tracking] Tracking started");
            
            // 确保GNSS模块退出待机模式
            if (gnssModule.isInStandbyMode()) {
              gnssModule.exitStandbyMode();
              gnssModule.setStandbyMode(false);
              Serial.println("[Tracking] Exited GNSS standby mode for tracking");
            }
          } else {
            Serial.println("[Tracking] Failed to start tracking");
          }
        }
      }
      
      // 重置t键标志
      interactionManager.resetTKeyPressed();
    }
    
    // 更新海拔图的用户操作状态
    bool debugAction = (renderEngine.getDebugVisible() || renderEngine.getDebugPosition() > -100);
    // 只将实际的键盘输入视为用户操作，而不是将动画视为用户操作
    // 这样可以确保在动画结束后，海拔图能够自动恢复显示
    bool actualUserInput = keyboardChanged && keyboardPressed;
    renderEngine.updateUserAction(actualUserInput || debugAction);
    
    if (userInteracted) {
      hasUserPanned = true;
      static bool userInteractedLogged = false;
      if (!userInteractedLogged) {
        Serial.println("[PAN DEBUG] User interacted, set hasUserPanned to true");
        userInteractedLogged = true;
      }
      
      // 获取缩放级别和平移偏移
      int zoom = interactionManager.getZoomLevel();
      int panX, panY;
      interactionManager.getPanOffset(panX, panY);
      
      // 检查是否是缩放操作
      if (interactionManager.isZoomChanged()) {
        // 缩放操作：根据锁定状态选择缩放中心
        float centerLat, centerLng;
        
        if (renderEngine.isLocationLockedState() && currentLocation.isValid) {
          // 如果定位已锁定，以当前位置为缩放中心
          centerLat = currentLocation.latitude;
          centerLng = currentLocation.longitude;
        } else {
          // 否则以屏幕中心为缩放中心
          renderEngine.screenToLatLng(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2, centerLat, centerLng);
        }
        
        // 使用zoomAroundPoint方法进行缩放
        renderEngine.zoomAroundPoint(centerLat, centerLng, zoom);
        
        // 同步平移偏移量和缩放级别到InteractionManager
        renderEngine.getPanOffset(panX, panY);
        interactionManager.setPanOffset(panX, panY);
        // 同步缩放级别
        int currentZoom = renderEngine.getZoomLevel();
        interactionManager.setZoomLevel(currentZoom);
        
        static bool zoomAppliedLogged = false;
        if (!zoomAppliedLogged) {
          Serial.printf("[ZOOM DEBUG] Zoomed to level %d around point (%.6f, %.6f)\n", zoom, centerLat, centerLng);
          zoomAppliedLogged = true;
        }
      } else {
        // 平移操作：直接设置平移偏移
        renderEngine.setPanOffset(panX, panY);
        
        static bool panAppliedLogged = false;
        if (!panAppliedLogged) {
          Serial.printf("[PAN DEBUG] Applied pan: X=%d, Y=%d\n", panX, panY);
          panAppliedLogged = true;
        }
      }
      
      needRender = true;
      // 清除变化标志，避免重复处理
      // 注意：这里我们不直接清除标志，而是依赖InteractionManager内部处理
    }
    
    // 如果还没有设置初始位置，不管有没有GPS，都尝试居中
    if (!initialPositionSet) {
      if (currentLocation.isValid) {
        // 如果有有效的GPS位置，则居中到GPS位置
        renderEngine.centerOnLocation(currentLocation.latitude, currentLocation.longitude);
        hasUserPanned = false; // Reset panning flag since we're starting fresh
        initialPositionSet = true;
        // 同步偏移量到InteractionManager
        int panX, panY;
        renderEngine.getPanOffset(panX, panY);
        interactionManager.setPanOffset(panX, panY);
        static bool initialGpsLogged = false;
        if (!initialGpsLogged) {
          Serial.println("[PAN DEBUG] Initial position set to GPS location");
          Serial.printf("[PAN DEBUG] Synchronized pan offset: X=%d, Y=%d\n", panX, panY);
          initialGpsLogged = true;
        }
      } else {
        // 如果没有有效的GPS位置，但有路线数据，则居中到路线的起点
        if (!routePoints.empty()) {
          Location startPoint = routePoints[0];  // 取路线起点作为中心
          renderEngine.centerOnLocation(startPoint.latitude, startPoint.longitude);
          initialPositionSet = true; // 设置初始位置标志为true
          // 同步偏移量到InteractionManager
          int panX, panY;
          renderEngine.getPanOffset(panX, panY);
          interactionManager.setPanOffset(panX, panY);
          static bool initialRouteLogged = false;
          if (!initialRouteLogged) {
            Serial.println("[PAN DEBUG] Initial position set to route start point");
            Serial.printf("[PAN DEBUG] Synchronized pan offset: X=%d, Y=%d\n", panX, panY);
            initialRouteLogged = true;
          }
        }
      }
    }
    // 如果用户没有手动操作过地图，且当前有有效GPS位置，则居中到当前位置
    else if (!hasUserPanned && currentLocation.isValid) {
      renderEngine.centerOnLocation(currentLocation.latitude, currentLocation.longitude);
      // 不要重置hasUserPanned标志，因为这是自动居中，不是用户手动操作
      static bool autoCenterLogged = false;
      if (!autoCenterLogged) {
        Serial.println("[PAN DEBUG] Auto-centered to GPS location (user not panned)");
        autoCenterLogged = true;
      }
    }
    // 如果用户已经操作过地图，但GPS位置发生了大幅变化（>100米），则重新居中
    else if (hasUserPanned && currentLocation.isValid && prevLocation.isValid) {
      // 计算距离变化（简化计算）
      double latDiff = fabs(currentLocation.latitude - prevLocation.latitude);
      double lngDiff = fabs(currentLocation.longitude - prevLocation.longitude);
      // 大约1度≈111公里，所以0.001度≈111米
      if (latDiff > 0.0009 || lngDiff > 0.0009) {  // 大约100米的变化
        renderEngine.centerOnLocation(currentLocation.latitude, currentLocation.longitude);
        hasUserPanned = false; // Reset panning flag since we moved significantly
        static bool recenterLogged = false;
        if (!recenterLogged) {
          Serial.println("[PAN DEBUG] Re-centered due to GPS position change, reset hasUserPanned");
          recenterLogged = true;
        }
      }
    }
    else if (hasUserPanned) {
      // 如果用户已经手动操作过地图，则不执行自动居中
      static bool skipAutoCenterLogged = false;
      if (!skipAutoCenterLogged) {
        Serial.println("[PAN DEBUG] User has panned, skipping auto-center");
        skipAutoCenterLogged = true;
      }
    }
    
    // 更新之前的位置
    if (currentLocation.isValid) {
      prevLocation = currentLocation;
    }
    
    // 处理文件选择菜单
    if (fileSelectionMenuOpen) {
      // 确保openMenu为true，这样键盘输入会被正确处理
      openMenu = true;
      
      // 禁用其他菜单的处理，确保文件选择菜单的输入优先
      settingsMenuOpen = false;
      
      // 静态标志，用于跟踪是否已经输出了菜单打开的日志
      static bool menuOpenLogOutput = false;
      
      // 检查是否需要输出菜单打开的日志
      if (!menuOpenLogOutput) {
        Serial.println("File selection menu is open, waiting for user input");
        menuOpenLogOutput = true;
      }
      
      if (keyboardChanged && keyboardPressed) {
        Serial.println("Keyboard input received in file selection menu");
        
        // 处理导航键
        bool hasNavigationKey = false;
        for(auto key : keys.word) {
          Serial.printf("Key pressed: %c (ASCII: %d)\n", key, key);
          if (key == ';') { // 上箭头
            if (selectedFileIndex > 0) {
              selectedFileIndex--;
              drawFileSelectionMenu();
              Serial.printf("Selected file index: %d, file: %s\n", selectedFileIndex, kmlFileList[selectedFileIndex].c_str());
            }
            hasNavigationKey = true;
          } else if (key == '.') { // 下箭头
            if (selectedFileIndex < (int)kmlFileList.size() - 1) {
              selectedFileIndex++;
              drawFileSelectionMenu();
              Serial.printf("Selected file index: %d, file: %s\n", selectedFileIndex, kmlFileList[selectedFileIndex].c_str());
            }
            hasNavigationKey = true;
          } else if (key == 8) { // 退格键作为取消（ASCII码8）
            // 取消
            Serial.println("Cancel key pressed");
            fileSelectionMenuOpen = false;
            openMenu = false;
            renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
            canvas.pushSprite(0, 0);
            Serial.println("File selection menu closed by cancel");
            hasNavigationKey = true;
            // 重置菜单打开日志标志
            menuOpenLogOutput = false;
          }
        }
        
        // 处理回车键 - 只有当没有导航键被按下时才处理
        if (keys.enter && !hasNavigationKey) {
          if (menuJustOpened) {
            // 忽略第一次Enter键按下事件，这是从设置菜单传递过来的
            Serial.println("Ignoring first Enter key press in file selection menu");
            menuJustOpened = false;
          } else {
            Serial.println("Enter key pressed, selecting file");
            // 选择文件
            if (selectedFileIndex < (int)kmlFileList.size()) {
              Serial.printf("User selected file: %s\n", kmlFileList[selectedFileIndex].c_str());
              loadSelectedKMLFile(kmlFileList[selectedFileIndex]);
              // 关闭文件选择菜单
              fileSelectionMenuOpen = false;
              openMenu = false;
              // 重新渲染界面
              renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
              canvas.pushSprite(0, 0);
              Serial.println("File selection menu closed after selecting file");
              // 重置菜单打开日志标志
              menuOpenLogOutput = false;
            }
          }
        }
      }
      return; // 文件选择菜单打开时，直接返回，不执行后续处理
    } else {
      // 当文件选择菜单关闭时，重置菜单打开日志标志
      static bool menuOpenLogOutput = false;
      menuOpenLogOutput = false;
    }
    
    // 控制刷新率
    static unsigned long lastRenderTime = 0;
    const unsigned long RENDER_INTERVAL = 100; // 提高刷新率到10FPS以改善用户体验
    if (needRender || currentTime - lastRenderTime > RENDER_INTERVAL) {
      if (!openMenu && !gpsNoFixAlertVisible && !helpMenuVisible) { // 只有在没有菜单打开且没有提示信息框时才渲染
        // 渲染界面，传递内存池信息以绘制完整路径和已记录的轨迹
        renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
        canvas.pushSprite(0, 0);  // 推送至屏幕
        lastRenderTime = currentTime;
      }
    }
  } else if (currentMode == MODE_GPS_INFO && !httpServerMenuOpen) {
    // 处理GPS Info模式特定的键盘输入
    // 如果配置菜单打开，使用handleKeys处理菜单输入；否则使用handleGPSInfoKeys处理常规输入
    if (configsMenu) {
      handleKeys(keyboardChanged, keyboardPressed, keys);  // 处理配置菜单的键盘输入
    } else {
      handleGPSInfoKeys(keyboardChanged, keyboardPressed, keys);  // 处理常规GPS Info模式的键盘输入，包括菜单关闭
    }
    
    // 控制屏幕刷新率 - 当配置菜单打开时，不执行常规屏幕更新以避免干扰菜单显示
    static unsigned long lastUpdateTime = 0;
    const unsigned long UPDATE_INTERVAL = 100; // 提高刷新率以改善用户体验
    unsigned long currentTime = millis();
    if (currentTime - lastUpdateTime > UPDATE_INTERVAL && !configsMenu) {
      // 根据GNSS搜索状态和屏幕状态控制GPS更新频率
      unsigned long gpsInterval;
      
      if (isGNSSSearching) {
        gpsInterval = GPS_UPDATE_INTERVAL_SEARCH; // 搜索模式使用更高频率
      } else {
        gpsInterval = isScreenOff ? GPS_UPDATE_INTERVAL_SCREEN_OFF : GPS_UPDATE_INTERVAL_NORMAL;
      }
      
      if (currentTime - lastGPSUpdateTime > gpsInterval) {
        // 读取GPS数据（serialGPSRead内部会通过feed()同时更新TinyGPSPlus和NMEA解析器）
        serialGPSRead();
        
        // 更新currentLocation（从gnssModule获取最新位置）
        currentLocation = gnssModule.getCurrentLocation();
        lastGPSUpdateTime = currentTime;
        
        // 检查是否获取到定位，如果获取到则结束搜索模式
        if (isGNSSSearching && currentLocation.isValid) {
          isGNSSSearching = false;
          Serial.println("GNSS acquired fix, switching to normal update interval");
        }
      }
      
      // 更新屏幕显示 - 只有在配置菜单关闭时才更新
      if (!isScreenOff) {
        updateScreen(false);
      }
      
      lastUpdateTime = currentTime;
    }
  }
  
  // 处理模式切换
  if (modeChanged) {
    canvas.fillScreen(TFT_BLACK);
    if (currentMode == MODE_HIKEPOD) {
      // 渲染界面，传递内存池信息以绘制完整路径和已记录的轨迹
      renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
      canvas.pushSprite(0, 0);  // 推送至屏幕
      renderEngine.reset3DView(); // 切换模式时重置 3D 视图到默认姿态与缩放
    } else {
      // 切换到 GPS Info 模式，强制完全刷新一次
      updateScreen(true);
    }
    modeChanged = false;
  }
  
  // 确保键盘响应，即使在复杂操作之间也让出控制权
  yield();
}


void drawHttpServerWindow(bool should_I) {
  if (should_I) {
    if (!httpServerMenuOpen) {
      initHttpServer();
      httpServerMenuOpen = true;
    }
    openMenu = true;

    // 绘制窗口
    canvas.fillRect(10, 10, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 20, TFT_WHITE);
    canvas.drawRect(12, 12, SCREEN_WIDTH - 24, SCREEN_HEIGHT - 24, TFT_BLACK);
    
    canvas.setTextColor(TFT_BLUE, TFT_WHITE);
    canvas.setTextSize(1);
    canvas.setCursor(20, 20);
    canvas.print("WiFi KML Manager");
    
    canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    canvas.setCursor(25, 35);
    String deviceId = String((uint32_t)ESP.getEfuseMac(), HEX).substring(0, 4);
    canvas.printf("SSID: HikePod_%s", deviceId.c_str());
    
    canvas.setCursor(25, 45);
    canvas.print("Password: (None)");
    
    canvas.setCursor(25, 65);
    canvas.setTextColor(TFT_RED, TFT_WHITE);
    canvas.setFont(&fonts::efontCN_12);
    canvas.println(wifiManager.getStatusMsg());
    
    canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    canvas.setCursor(25, 80);
    canvas.println("Connect to WiFi & Visit URL to ");
    canvas.println("     upload/manage KMLs");
    
    canvas.setCursor(20, 120);
    canvas.println("Press 'w' to close & exit");
    
    canvas.pushSprite(0, 0);
  } else {
    stopHttpServer();
    httpServerMenuOpen = false;
    openMenu = false;
    // 渲染地图
    renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
    canvas.pushSprite(0, 0);
  }
}

void showGPSNoFixAlert() {
  gpsNoFixAlertVisible = true;
  
  // 绘制提示信息框
  canvas.fillRect(40, 50, SCREEN_WIDTH - 80, 40, TFT_WHITE);
  canvas.drawRect(40, 50, SCREEN_WIDTH - 80, 40, TFT_BLACK);
  canvas.setTextColor(TFT_BLACK, TFT_WHITE);
  canvas.setTextSize(1);
  
  // 显示提示信息
  canvas.setCursor(50, 65);
  canvas.println("GPS no fix !");
  canvas.setCursor(SCREEN_WIDTH - 65, 80);
  canvas.setTextColor(TFT_WHITE , TFT_BLACK);
  canvas.println("ok");
  
  canvas.pushSprite(0, 0);
}

void showKMLFullAlert() {
  kmlFullAlertVisible = true;
  
  // 绘制提示信息框 (参考 GPS no fix 样式)
  canvas.fillRect(30, 45, SCREEN_WIDTH - 60, 50, TFT_WHITE);
  canvas.drawRect(30, 45, SCREEN_WIDTH - 60, 50, TFT_BLACK);
  canvas.setTextColor(TFT_BLACK, TFT_WHITE);
  canvas.setTextSize(1);
  
  // 显示提示信息
  canvas.setCursor(40, 55);
  canvas.println("KML Point Limit!");
  canvas.setCursor(40, 68);
  canvas.println("Some points skipped.");
  
  canvas.setCursor(SCREEN_WIDTH - 55, 80);
  canvas.setTextColor(TFT_WHITE , TFT_BLACK);
  canvas.println("ok");
  
  canvas.pushSprite(0, 0);
}

void drawHikePodHelpMenu(bool should_I) {
  if (should_I == true) {
    openMenu = true;
    // 绘制帮助菜单，风格与c设置菜单统一
    canvas.fillRect(10, 10, SCREEN_WIDTH-20, SCREEN_HEIGHT-20, TFT_WHITE);
    canvas.drawRect(12, 12, SCREEN_WIDTH-24, SCREEN_HEIGHT-24, TFT_BLACK);
    canvas.setTextColor(TFT_BLACK , TFT_WHITE );
    canvas.setTextSize(1);
    
    // 显示帮助标题
    canvas.setTextColor(TFT_BLUE, TFT_WHITE);
    canvas.setCursor(20, 20);
    canvas.println("HikePod Help");
    
    // 列出支持的按键
    const char* helpText[] = {
      "[h] Help menu (this)",
      "[c] Open settings menu",
      "[v] Switch 2D/3D view",
      "[Space] Lock/unlock(2D)/Rot center(3D)",
      "[t] Start/stop tracking",
      "[TAB] Switch to GPS Info",
      "[w] WiFi KML transfer",
      "[ESC] Toggle debug info",
      "[+/-] Zoom in/out",
      "[[/]] Adjust vert scale(3D)"
    };
    
    int count = sizeof(helpText) / sizeof(helpText[0]);
    int y = 30;
    for (int i = 0; i < count; i++) {
      String line = helpText[i];
      int endKey = line.indexOf(']') + 1;
      if (endKey > 0) {
        String key = line.substring(0, endKey);
        String desc = line.substring(endKey);
        canvas.setTextColor(TFT_BLACK, TFT_WHITE);
        canvas.setCursor(20, y);
        canvas.print(key);
        canvas.setTextColor(TFT_BLACK, TFT_WHITE);
        canvas.print(desc);
      } else {
        canvas.setTextColor(TFT_BLACK, TFT_WHITE);
        canvas.setCursor(20, y);
        canvas.print(line);
      }
      y += 10;
    }
    
    canvas.pushSprite(0, 0);
  } else {
    openMenu = false;
    helpMenuVisible = false;
    // 重新渲染界面
    renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
    canvas.pushSprite(0, 0);
  }
}

void drawSettingsMenu(bool should_I) {
  if (should_I == true) {
    openMenu = true;
    settingsMenuOpen = true;
    // 白色背景
    canvas.fillRect(10, 10, SCREEN_WIDTH-20, SCREEN_HEIGHT-20, TFT_WHITE);
    canvas.drawRect(12, 12, SCREEN_WIDTH-24, SCREEN_HEIGHT-24, TFT_BLACK);
    canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    canvas.setFont(&fonts::efontCN_12);
    int y = 19;
    
    // 显示标题
    canvas.setTextColor(TFT_BLUE, TFT_WHITE);
    canvas.setCursor(20, y);
    canvas.print("HikePod Settings");
    y += 12;
    
    // 显示选择KML文件选项
    canvas.setTextColor(settingsMenuSelection == 0 ? TFT_BLUE : TFT_BLACK, TFT_WHITE);
    canvas.setCursor(20, y);
    canvas.print("> Select KML File ");
    y += 12;
    
    // 显示亮度调节选项
    canvas.setTextColor(settingsMenuSelection == 1 ? TFT_BLUE : TFT_BLACK, TFT_WHITE);
    canvas.setCursor(20, y);
    canvas.print("> Brightness（10-255） : ");
    canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    canvas.print(String(screenBrightness));
    y += 12;

    // 显示屏幕超时时间选项
    canvas.setTextColor(settingsMenuSelection == 2 ? TFT_BLUE : TFT_BLACK, TFT_WHITE);
    canvas.setCursor(20, y);
    canvas.print("> Screen Timeout : ");
    canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    if (SCREEN_TIMEOUT == 0) {
      canvas.print("Never");
    } else if (SCREEN_TIMEOUT < 60000) {
      canvas.print(String(SCREEN_TIMEOUT / 1000) + "s");
    } else {
      int mins = SCREEN_TIMEOUT / 60000;
      int secs = (SCREEN_TIMEOUT % 60000) / 1000;
      if (secs == 0) {
        canvas.print(String(mins) + "m");
      } else {
        canvas.print(String(mins) + "m " + String(secs) + "s");
      }
    }
    y += 12;
    
    // 显示正常GPS更新频率选项
    canvas.setTextColor(settingsMenuSelection == 3 ? TFT_BLUE : TFT_BLACK, TFT_WHITE);
    canvas.setCursor(20, y);
    canvas.print("> GPS Int : ");
    canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    canvas.print(String(GPS_UPDATE_INTERVAL_NORMAL / 1000) + "s");
    y += 12;
    
    // 显示息屏GPS更新频率选项
    canvas.setTextColor(settingsMenuSelection == 4 ? TFT_BLUE : TFT_BLACK, TFT_WHITE);
    canvas.setCursor(20, y);
    canvas.print("> ScreenOff GPS : ");
    canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    canvas.print(String(GPS_UPDATE_INTERVAL_SCREEN_OFF / 1000) + "s");
    y += 12;

    // 显示关键点开关
    canvas.setTextColor(settingsMenuSelection == 5 ? TFT_BLUE : TFT_BLACK, TFT_WHITE);
    canvas.setCursor(20, y);
    canvas.print("> Show POIs : ");
    canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    if (showPOIsMode == 0) canvas.print("OFF");
    else if (showPOIsMode == 1) canvas.print("ON");
    else canvas.print("AUTO");
    y += 6;
    
    // 添加电量消耗曲线
    if (batteryHistory.size() >= 1) {
      y += 4;
      
      // 绘制电量消耗曲线标题
      canvas.setTextColor(TFT_BLUE, TFT_WHITE);
      canvas.setCursor(20, y);
      canvas.print("Battery Consumption :");
      y += 4;
      
      // 电量曲线配置
      const int CHART_HEIGHT = 30; // 减少高度，确保不超出菜单下边框
      const int CHART_WIDTH = SCREEN_WIDTH - 60; // 减少宽度，为左侧文字留出空间
      const int CHART_X = 40; // 向右移动20像素，避免左侧文字超出屏幕
      const int CHART_Y = y;
      
      // 绘制电量曲线（无边框）
      
      // 计算电量范围
      int minBattery = 100;
      int maxBattery = 0;
      for (int bat : batteryHistory) {
        if (bat < minBattery) minBattery = bat;
        if (bat > maxBattery) maxBattery = bat;
      }
      
      // 添加一些边距，确保纵轴能够适应电量值的大范围变化
      int batteryRange = maxBattery - minBattery;
      if (batteryRange < 10) {
        minBattery = max(0, minBattery - 5);
        maxBattery = min(100, maxBattery + 5);
        batteryRange = 10;
      } else {
        // 对于大范围变化，添加更多边距以确保所有点都在屏幕内
        minBattery = max(0, static_cast<int>(minBattery - batteryRange * 0.15));
        maxBattery = min(100, static_cast<int>(maxBattery + batteryRange * 0.15));
        batteryRange = maxBattery - minBattery;
      }
      
      // 确保电池范围至少为10，避免除以零或计算错误
      if (batteryRange < 10) {
        batteryRange = 10;
        if (minBattery == maxBattery) {
          minBattery = max(0, minBattery - 5);
          maxBattery = min(100, maxBattery + 5);
        }
      }
      
      // 绘制电量范围（最高值和最低值）
      // 已移除，改为在曲线最左侧显示开始值，最右侧显示当前值
      
      // 绘制电量折线
      int lastX = -1;
      int lastY = -1;
      int lastBattery = -1;
      
      for (size_t i = 0; i < batteryHistory.size(); i++) {
        int battery = batteryHistory[i];
        
        // 计算X坐标：使用MAX_BATTERY_HISTORY作为横轴长度，从左到右绘制
        int x = CHART_X + 5 + (int)((double)i / (MAX_BATTERY_HISTORY - 1) * (CHART_WIDTH - 10));
        
        // 计算Y坐标（电量越高，Y值越小）
        int batY = CHART_Y + CHART_HEIGHT - 5 - (int)((battery - minBattery) / (double)batteryRange * (CHART_HEIGHT - 10));
        
        // 如果只有一个数据点，绘制一个点
        if (batteryHistory.size() == 1) {
          // 不绘制圆点，仅用曲线
        } else {
          // 绘制折线，根据电量值选择颜色
          if (lastX != -1 && lastY != -1) {
            // 使用两个端点的平均电量来决定线条颜色
            int avgBattery = (lastBattery + battery) / 2;
            uint16_t lineColor = (avgBattery >= 20) ? TFT_BLUE : TFT_RED;
            canvas.drawLine(lastX, lastY, x, batY, lineColor);
          }
        }
        
        lastX = x;
        lastY = batY;
        lastBattery = battery;
      }
      
      // 在曲线最左侧显示开始值
      if (batteryHistory.size() >= 1) {
        int startBattery = batteryHistory[0];
        int startX = CHART_X + 5;
        int startY = CHART_Y + CHART_HEIGHT - 5 - (int)((startBattery - minBattery) / (double)batteryRange * (CHART_HEIGHT - 10));
        canvas.setTextSize(0);
        canvas.setTextColor(TFT_BLACK);
        canvas.setCursor(startX - 30, startY - 4);
        canvas.print(String(startBattery) + "%");
      }
      
      // 在曲线最右侧显示当前值
      if (batteryHistory.size() >= 1) {
        int currentBattery = batteryHistory[batteryHistory.size() - 1];
        int currentX = CHART_X + 5 + (int)((double)(batteryHistory.size() - 1) / (MAX_BATTERY_HISTORY - 1) * (CHART_WIDTH - 10));
        int currentY = CHART_Y + CHART_HEIGHT - 5 - (int)((currentBattery - minBattery) / (double)batteryRange * (CHART_HEIGHT - 10));
        canvas.setTextSize(0);
        canvas.setTextColor(TFT_BLACK);
        canvas.setCursor(currentX + 5, currentY - 4);
        canvas.print(String(currentBattery) + "%");
      }
    }
    
    canvas.pushSprite(0, 0);  // 推送至屏幕
  } else {
    openMenu = false;
    settingsMenuOpen = false;
    // 渲染界面，传递内存池信息以绘制完整路径和已记录的轨迹
    renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
    canvas.pushSprite(0, 0);  // 推送至屏幕
  }
}

void drawPowerSavingInfo(bool should_I) {
  // 保留原函数，暂时不使用
  if (should_I == true) {
    drawSettingsMenu(true);
  } else {
    drawSettingsMenu(false);
  }
}

void renderGPSInfo() {
  // 读取GPS数据
  serialGPSRead();
  
  // 更新屏幕显示
  updateScreen(false);
}

// Cardputer_GPS_Info 核心功能函数

/*    Open or close the GPS UART serial console.
*/
void initGPSSerial(bool should_I) {
  static bool gpsInitialized = false;
  
  if (should_I == true && !gpsInitialized) {
    gnssModule.begin(gpsRxPin, gpsTxPin, gpsBaud); // Start GPS UART.
    gpsInitialized = true;
    // 重新启动搜索模式，使用高刷新率搜星
    isGNSSSearching = true;
    Serial.println("GPS UART initialized, starting search mode");
    // 给GPS模块一些时间初始化
    //delay(1000);
  }
  else if (should_I == false && gpsInitialized) {
    gnssModule.end();
    gpsInitialized = false;
    Serial.println("GPS UART closed");
  }
}

/*    Read the GPS seria and compose the NMEA sentence.
*/
void serialGPSRead() {
  static String nmeaLine = "";
  bool gotValidChar = false;
  GPSState prevState = gpsSerialState;
  
  while (gnssModule.available()) {
    char c = gnssModule.read();
    if (c != '\r' && c != '\n') gotValidChar = true;
    
    // 同时将字符传递给TinyGPSPlus解析器，避免数据竞争
    gnssModule.feed(c);
    
    if (c == '\n') {
      nmeaDispatcher(nmeaLine);
      nmeaLine = "";
    } else if (c != '\r')
      nmeaLine += c;
  }
  
  if (gotValidChar) {
    lastValidGpsMillis = millis();
    gpsSerialState = GPS_ON;
  } else if (gpsSerial && millis() - lastValidGpsMillis > GPS_TIMEOUT) {
    // 只有当gpsSerial为true（即用户希望GPS是开启的）且超时未收到数据时，才设置为错误状态
    // 但不再自动关闭GPS，保持GPS开启状态
    gpsSerialState = GPS_ERR;
    // 移除自动关闭GPS的代码，让GPS保持开启
  }
  
  if (gpsSerialState != prevState)
    drawStatus();
}

/*    Read the NMEA sentence and dispatch to parsers.
*/
void nmeaDispatcher(const String &nmeaLine) {
  if (nmeaSerial)
    Serial.println(nmeaLine);
  // Trim line endings.
  String line = nmeaLine;
  line.trim();
  // Define NMEA handlers.
  struct NMEAHandler { 
    const char* prefix; 
    void (*parser)(const String&); 
  };
  static NMEAHandler handlers[] = {
    {"$GPGSV", parseGSV},
    {"$GLGSV", parseGSV},
    {"$GAGSV", parseGSV},
    {"$BDGSV", parseGSV},
    {"$GNGSV", parseGSV},
    {"$GPGSA", parseGSA},
    {"$GLGSA", parseGSA},
    {"$GAGSA", parseGSA},
    {"$BDGSA", parseGSA},
    {"$GNGSA", parseGSA}
  };
  // Dispatch to the correct parser.
  for (auto &h : handlers) {
    if (line.startsWith(h.prefix)) {
      h.parser(line);
      break;
    }
  }
}

/*    Parse NMEA 0183 GSA sentence. (GNSS DOP and Active Satellites).
 *     Mode (2D/3D), IDs of used satellites, PDOP/HDOP/VDOP.
*/
void parseGSA(const String &line) {
  int fieldNum = 0, lastIndex = 0;
  for (int i = 0; i <= line.length(); i++) {
    if (i == line.length() || line[i] == ',' || line[i] == '*') {
      String val = line.substring(lastIndex, i);
      lastIndex = i + 1;
      fieldNum++;
      if (fieldNum >= 4 && fieldNum <= 15 && val.length() > 0) {
        int id = val.toInt();
        for (auto &sat : satellites) {
          if (sat.id == id) sat.used = true;
        }
      }
    }
  }
}

/*    Parse NMEA 0183 GSV sentence. (GNSS Satellites in View).
 *     Info on all visible satellites (ID, elevation, azimuth, SNR).
*/
void parseGSV(const String &line) {
  String system;
  if (line.startsWith("$GPGSV")) system = "GPS";
  else if (line.startsWith("$GLGSV")) system = "GLONASS";
  else if (line.startsWith("$GAGSV")) system = "Galileo";
  else if (line.startsWith("$BDGSV")) system = "BeiDou";
  else if (line.startsWith("$GNGSV")) system = "Mixed";
  else return;
  GSVSequenceState* state = getGSVState(system);
  if (!state) return;
  std::vector<String> fields;
  int lastIndex = 0;
  for (int i = 0; i <= line.length(); i++) {
    if (i == line.length() || line[i] == ',' || line[i] == '*') {
      fields.push_back(line.substring(lastIndex, i));
      lastIndex = i + 1;
    }
  }
  if (fields.size() < 4) return;
  int totalMsgs = fields[1].toInt(); 
  int msgNum    = fields[2].toInt();
  if (msgNum == 1 || state->totalMsgs != totalMsgs) {
    state->currentVisible.clear();
    state->totalMsgs = totalMsgs;
  }
  // Pars sats.
  for (size_t i = 4; i + 3 < fields.size(); i += 4) {
    SatData sat;
    sat.system = system;
    sat.id = fields[i].toInt();
    // Inverted BeiDou.
    if (system == "BeiDou") {
      sat.azimuth   = fields[i + 1].toInt();
      sat.elevation = fields[i + 2].toInt();
    } else {
      sat.elevation = fields[i + 1].toInt();
      sat.azimuth   = fields[i + 2].toInt();
    }
    sat.snr = fields[i + 3].toInt();
    sat.used = false;
    storeSatellite(sat);
    state->currentVisible.push_back(sat.id);
  }
  state->lastMsgNum = msgNum;
  if (msgNum == totalMsgs) {
    for (auto &s : satellites) {
      if (s.system == system) {
        s.visible = (std::find(state->currentVisible.begin(),state->currentVisible.end(),s.id) != state->currentVisible.end());
      }
    }
  }
}

/*    Stores GNSS satellite sequence states.
*/
GSVSequenceState* getGSVState(const String& system) {
  for (int i = 0; i < gsvCount; i++) {
    if (gsvStates[i].system == system)
      return &gsvStates[i];
  }
  if (gsvCount < 5) {
    gsvStates[gsvCount].system = system;
    return &gsvStates[gsvCount++];
  }
  return nullptr;
}

/*    Store satellite in a list.
*/
void storeSatellite(const SatData &sat) {
  for (auto &s : satellites) {
    if (s.system == sat.system && s.id == sat.id) {
      s.elevation = sat.elevation;
      s.azimuth   = sat.azimuth;
      s.snr       = sat.snr;
      return;
    }
  }
  satellites.push_back(sat);
}

/*    Manage display elements drawing.
*/
void updateScreen(bool force) {
  static uint32_t lastDisplay = 0;
  if (force || millis() - lastDisplay > 1000) // 1sec update or force it.
  {
    lastDisplay = millis();
    if (openMenu) return;
    if (force) {
      canvas.fillScreen(TFT_BLACK);
      drawHeader();
      drawStatus();
    }
    // Satellites datas.
    drawSatelliteDataTab();
    // Satellites plot.
    drawSkyPlot();
    // 一次性刷新到屏幕，避免闪烁
    canvas.pushSprite(0, 0);
  }
}

/*    Draw the satellites sky plot graph.
*/
void drawSkyPlot() {
  int x = 143;
  int y = 27;
  int w = 96;
  int h = w-1;
  int half_side = h * 0.5;
  int cx = x + half_side;
  int cy = y + half_side;
  int r = half_side;
  canvas.drawRect(x-1, y-1, w+2, h+2, TFT_DARKGREY);
  canvas.fillRect(x, y, w, h, TFT_BLACK);
  // Ref circles.
  canvas.drawCircle(cx, cy, r, TFT_WHITE);
  canvas.drawCircle(cx, cy, r * 0.66, TFT_DARKGREY);
  canvas.drawCircle(cx, cy, r * 0.33, TFT_DARKGREY);
  canvas.drawLine(cx - r, cy, cx + r, cy, TFT_DARKGREY);
  canvas.drawLine(cx, cy - r, cx, cy + r, TFT_DARKGREY);
  // Cardinals label.
  canvas.setTextSize(1);
  canvas.setTextDatum(TL_DATUM);
  canvas.setTextColor(TFT_LIGHTGREY);
  canvas.setCursor(cx - 3, cy - r + 4);  canvas.print("N");
  canvas.setCursor(cx - 3, cy + r - 10); canvas.print("S");
  canvas.setCursor(cx + r - 10, cy - 3); canvas.print("E");
  canvas.setCursor(cx - r + 4, cy - 3);  canvas.print("W");
  
  // Satellites - only draw if there is a valid GPS fix
  if (currentLocation.isValid) {
    for (auto &sat : satellites) {
      float elev = constrain(sat.elevation, 0.0, 90.0);
      float az   = fmod(sat.azimuth + 360.0, 360.0);
      float rad = (90.0 - elev) / 90.0 * r;
      float radAz = radians(az);
      float sx = cx + rad * sin(radAz);
      float sy = cy - rad * cos(radAz);
      uint16_t color = TFT_RED;
      if (sat.used)
        color = TFT_GREEN;
      else if
        (sat.visible) color = TFT_YELLOW;
      canvas.fillCircle(sx, sy, 2, color); // Satellite dot.
      if (!hidePlotId) // Satellite id.
      {
        canvas.setTextSize(0);
        canvas.setTextColor(TFT_BLACK);
        canvas.setCursor(sx + 4, sy - 4);
        canvas.printf("%d", sat.id);
        canvas.setTextColor(color);
        canvas.setCursor(sx + 5, sy - 3);
        canvas.printf("%d", sat.id);
      }
      if (!hidePlotSystem) // Satellite system.
      {
        const char* sys;
        if (sat.system == "GPS") sys = "Gp";
        else if (sat.system == "GLONASS") sys = "Gl";
        else if (sat.system == "Galileo") sys = "Ga";
        else if (sat.system == "BeiDou") sys = "Bd";
        else sys = "?";
        canvas.setTextSize(0);
        canvas.setTextColor(TFT_BLACK);
        canvas.setCursor(sx + 4, sy - 4);
        canvas.printf("%s", sys);
        canvas.setTextColor(color);
        canvas.setCursor(sx + 5, sy - 3);
        canvas.printf("%s", sys);
      }
    }
  }
}

/*    Draw the satellites main data table.
*/
void drawSatelliteDataTab(){
  int x = 1;
  int y = 26;
  // Column 1.
  // Labels.
  const char* c1labels[] = { "Lat", "Lng", "Alt", "Spd", "Crs", "Date", "Time", "HDOP" };
  // Values.
  char c1values[8][20];
  if (currentLocation.isValid) {sprintf(c1values[0], "%.6f", currentLocation.latitude);} else {sprintf(c1values[0], "NoFix");}
  if (currentLocation.isValid) {sprintf(c1values[1], "%.6f", currentLocation.longitude);} else {sprintf(c1values[1], "NoFix");}
  if (currentLocation.isValid) {sprintf(c1values[2], "%.2f", currentLocation.altitude);} else {sprintf(c1values[2], "0");}
  if (currentLocation.isValid) {sprintf(c1values[3], "%.1f", gnssModule.getSpeedKmph());} else {sprintf(c1values[3], "0");}
  if (currentLocation.isValid) {sprintf(c1values[4], "%.1f", gnssModule.getCourseDeg());} else {sprintf(c1values[4], "0");}
  if (currentLocation.isValid && gnssModule.isDateValid()) {sprintf(c1values[5], "%02d/%02d/%02d", gnssModule.getLocalDay(), gnssModule.getLocalMonth(), gnssModule.getLocalYear() % 100);} else {sprintf(c1values[5], "0");}
  if (currentLocation.isValid && gnssModule.isTimeValid()) {sprintf(c1values[6], "%02d:%02d:%02d", gnssModule.getLocalHour(), gnssModule.getLocalMinute(), gnssModule.getLocalSecond());} else {sprintf(c1values[6], "0");}
  if (currentLocation.isValid) {sprintf(c1values[7], "%.1f", gnssModule.getHDOP());} else {sprintf(c1values[7], "0");}
  // Draw.
  for (int i = 0; i < 8; i++) {
    canvas.fillRect(x, y, 90, 13, TFT_BLACK);
    canvas.drawRect(x, y, 90, 13, TFT_DARKGREY);
    canvas.setTextColor(TFT_WHITE);
    canvas.setTextDatum(TL_DATUM);
    canvas.setCursor(x + 4, y + 3);
    canvas.setTextSize(1);
    canvas.printf("%s: %s", c1labels[i], c1values[i]);
    y += 12;
  }
  // Column 2.
  y = 26;
  x += 89;
  // Labels.
  const char* c2labels[] = { "Seen", "Visb", "Used", "InFx", "GPS", "Gln", "Gal", "BDo" };
  // Values.
  char c2values[8][12];
  if (currentLocation.isValid) {
    int totalAll = satellites.size(); // Ever seen.
    int totalUsed = 0;                // Ever used in fix.
    int totalVisible = 0;             // Now visible.
    int gpsVisible = 0;
    int glonassVisible = 0;
    int galileoVisible = 0;
    int beidouVisible = 0;
    for (auto &sat : satellites) {
      if (sat.used) totalUsed++;
      if (sat.visible) {
        totalVisible++;
        if (sat.system == "GPS") gpsVisible++;
        else if (sat.system == "GLONASS") glonassVisible++;
        else if (sat.system == "Galileo") galileoVisible++;
        else if (sat.system == "BeiDou") beidouVisible++;
      }
    }
    sprintf(c2values[0], "%d", totalAll);
    sprintf(c2values[1], "%d", totalVisible);
    sprintf(c2values[2], "%d", totalUsed);
    sprintf(c2values[3], "%d", gnssModule.getSatelliteCount()); // 使用参与定位的卫星数
    sprintf(c2values[4], "%d", gpsVisible);
    sprintf(c2values[5], "%d", glonassVisible);
    sprintf(c2values[6], "%d", galileoVisible);
    sprintf(c2values[7], "%d", beidouVisible);
  } else {
    // No fix, display 0 for all satellite-related values
    for (int i = 0; i < 8; i++) {
      sprintf(c2values[i], "0");
    }
  }
  // Draw.
  for (int i = 0; i < 8; i++) {
    canvas.fillRect(x, y, 53, 13, TFT_BLACK);
    canvas.drawRect(x, y, 53, 13, TFT_DARKGREY);
    canvas.setTextColor(TFT_WHITE);
    canvas.setTextDatum(TL_DATUM);
    canvas.setCursor(x + 4, y + 3);
    canvas.setTextSize(1);
    canvas.printf("%s: %s", c2labels[i], c2values[i]);
    y += 12;
  }
}

/*    Draw the satellites main data table.
*/
void drawHeader(){
  int x = 1;
  int y = 1;
  int w = SCREEN_WIDTH;
  int h = 13;
  canvas.fillRect(x, y, w, h, TFT_GREEN);
  canvas.setTextColor(TFT_BLACK);
  canvas.setTextDatum(TL_DATUM);
  canvas.setCursor(x + 4, y + 3);
  canvas.printf("%-1s", "      -= Cardputer GPS Info =-");
  // Key map.
  canvas.fillRect(x, y+h-1, w, h, TFT_GREEN);
  canvas.setTextColor(TFT_BLACK); // 改为黑色以在绿色背景上更清晰
  canvas.setTextDatum(TL_DATUM);
  canvas.setCursor(x + 4, y+h + 3);
  canvas.printf("%-1s", "[s]On/Off [c]Config [h]Help [Tab]Mode");
}

/*    Draw app features status.
*/
void drawStatus(){
  int x = 1;
  int y = 122;
  int w = SCREEN_WIDTH;
  int h = 13;
  char statusChar[64];
  statusChar[0] = '\0';
  const char* gpsStr = "Off";
  if (gpsSerialState == GPS_ON) gpsStr = "On";
  else if (gpsSerialState == GPS_ERR) gpsStr = "Err";
  snprintf(statusChar + strlen(statusChar), sizeof(statusChar) - strlen(statusChar),"GP:%s ", gpsStr);
  snprintf(statusChar + strlen(statusChar),sizeof(statusChar) - strlen(statusChar),"Rx:%d Tx:%d ", gpsRxPin, gpsTxPin);
  snprintf(statusChar + strlen(statusChar),sizeof(statusChar) - strlen(statusChar),"Bd:%d", gpsBaud);
  canvas.fillRect(x, y, w, h, TFT_BLACK);
  canvas.drawRect(x, y, w, h, TFT_DARKGREY);
  canvas.setTextColor(TFT_WHITE);
  canvas.setTextDatum(TL_DATUM);
  canvas.setCursor(x + 4, y + 3);
  canvas.setTextSize(1);
  canvas.printf("%-1s", statusChar);
}

/*    Handle keyboard data inputs for GPS Info mode.
*/
void handleGPSInfoKeys(bool keyboardChanged, bool keyboardPressed, Keyboard_Class::KeysState keys) {
  // 处理GPS Info模式特定的键盘输入
  if(keyboardChanged) {
    if(keyboardPressed) {
      
      // 处理其他GPS Info模式的按键
      for(auto key : keys.word) {
        // 如果需要处理特定于GPS Info模式的按键，可以在这里添加
        // 目前只处理通用功能键
        if (key == 's') {
          gpsSerial = !gpsSerial;
          initGPSSerial(gpsSerial);
          gpsSerialState = gpsSerial ? GPS_ON : GPS_OFF;
          drawStatus();
        }
        else if (key == 'c') {
          configsMenu = !configsMenu; // Invert status.
          drawConfig(configsMenu);
        }
        else if (key == 'h') {
          helpMenu = !helpMenu; // Invert status.
          drawHelp(helpMenu);
        }
        else if (key == 'i') {
          infoMenu = !infoMenu; // Invert status.
          drawInfo(infoMenu);
        }
        else if (key == 'p') {
          hidePlotId = !hidePlotId; // Invert status.
        }
        else if (key == 'o') {
          hidePlotSystem = !hidePlotSystem; // Invert status.
        }
      }
      
      // 检查删除键
      if (keys.del) {
        // 处理删除键的逻辑
      }
      
      // 检查回车键
      if (keys.enter) {
        // 处理回车键的逻辑
      }
    }
  }
}

/*    Draw configuration popup.
*/
void drawConfig(bool should_I) {
  if (should_I == true) {
    openMenu = true;  // 设置openMenu为true，防止updateScreen刷新屏幕
    if (gpsSerial) { // If active, stop it.
      gpsSerial = false;
      initGPSSerial(false);
    }
    canvas.fillRect(10, 10, SCREEN_WIDTH-20, SCREEN_HEIGHT-20, TFT_BLACK);
    canvas.drawRect(12, 12, SCREEN_WIDTH-24, SCREEN_HEIGHT-24, TFT_GREEN);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.setTextSize(1);
    canvas.setCursor(25, 25);
    canvas.printf("Configurations:\n");
    canvas.setCursor(25, 35);
    canvas.println("Nav: [Up/Dow]. Val: [0-9].");
    canvas.setCursor(25, 45);
    canvas.println("Exit: [c]. Save: [ok].");
    canvas.setCursor(25, 70);
    canvas.printf("Cardp. RX pin (act:%d): %s %s", gpsRxPin, configsTmp[0].c_str(), configsMenuSel == 0 ? "<" : " ");
    canvas.setCursor(25, 80);
    canvas.printf("Cardp. TX pin (act:%d): %s %s", gpsTxPin, configsTmp[1].c_str(), configsMenuSel == 1 ? "<" : " ");
    canvas.setCursor(25, 90);
    canvas.printf("Cardp. Baud (act:%d): %s %s", gpsBaud, configsTmp[2].c_str(), configsMenuSel == 2 ? "<" : " ");
    canvas.pushSprite(0, 0);  // 推送至屏幕
  }
  else {
    openMenu = false;  // 关闭菜单时重置openMenu
    
    // 根据当前模式选择正确的更新方式
    if (currentMode == MODE_GPS_INFO) {
      updateScreen(true); // Forced update.
    } else if (currentMode == MODE_HIKEPOD) {
      // 3D视图模式，需要重新渲染
      if (currentViewMode == MODE_3D) {
        renderEngine.render3D(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
      } else {
        renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
      }
      // 确保亮度设置生效
      M5Cardputer.Display.setBrightness(screenBrightness);
      // 推送到屏幕
      canvas.pushSprite(0, 0);
    }
  }
}

/*    Draw info popup.
*/
void drawInfo(bool should_I) {
  if (should_I == true) {
    openMenu = true;
    const char* helpText[] = {
      "Cardputer GPS Info",
      "ADV",
      "",
      "Press Tab to switch",
      "back to HikePod mode"
    };
    int count = sizeof(helpText) / sizeof(helpText[0]);
    canvas.fillRect(18, 18, 204, 99, TFT_BLACK);
    canvas.drawRect(20, 20, 200, 95, TFT_GREEN);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.setTextSize(1);
    canvas.setCursor(25, 24);
    int y = 24;
    for (int i = 0; i < count; i++) {
      canvas.setCursor(25, y);
      canvas.println(helpText[i]);
      y += 10;
    }
    canvas.pushSprite(0, 0);  // 推送至屏幕
  } else {
    openMenu = false;
    updateScreen(true); // Forced update.
  }
}

/*    Draw help popup.
*/
void drawHelp(bool should_I) {
  if (should_I == true) {
    openMenu = true;
    const char* helpText[] = {
      "[s] Start/Stop the GPS (serial).",
      "[c] Configuration menu.",
      "[h] Help menu (this).",
      "[i] Info menu.",
      "[p] Show/hide ID on skyplot.",
      "[o] Show/hide System on skyplot.",
      "[Tab] Switch to HikePod mode"
    };
    int count = sizeof(helpText) / sizeof(helpText[0]);
    canvas.fillRect(10, 10, SCREEN_WIDTH-20, SCREEN_HEIGHT-20, TFT_BLACK);
    canvas.drawRect(12, 12, SCREEN_WIDTH-24, SCREEN_HEIGHT-24, TFT_GREEN);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.setTextSize(1);
    int y = 19;
    for (int i = 0; i < count; i++) {
      String line = helpText[i];
      int endKey = line.indexOf(']') + 1;
      if (endKey > 0) {
        String key = line.substring(0, endKey);
        String desc = line.substring(endKey);
        canvas.setTextColor(TFT_GREEN, TFT_BLACK);
        canvas.setCursor(20, y);
        canvas.print(key);
        canvas.setTextColor(TFT_WHITE, TFT_BLACK);
        canvas.print(desc);
      } else {
        canvas.setTextColor(TFT_WHITE, TFT_BLACK);
        canvas.setCursor(25, y);
        canvas.print(line);
      }
      y += 10;
    }
    canvas.pushSprite(0, 0);  // 推送至屏幕
  } else {
    openMenu = false;
    updateScreen(true); // Forced update.
  }
}

/*    Handle app functions.
*/
void handleControls(bool keyboardChanged, bool keyboardPressed, Keyboard_Class::KeysState keys) {
  // Check if keyboard has changed state
  if(keyboardChanged) {
    if(keyboardPressed) {
      
      // 如果 WiFi KML Manager 窗口打开，除了 'w' 键以外不响应其他按键
      if (httpServerMenuOpen) {
        bool wPressed = false;
        for (auto key : keys.word) {
          if (key == 'w') {
            wPressed = true;
            break;
          }
        }
        if (!wPressed) return; // 如果没按 'w'，直接忽略所有其他按键
      }

      // 处理提示信息框的关闭
      if ((gpsNoFixAlertVisible || kmlFullAlertVisible) && keys.enter) {
        gpsNoFixAlertVisible = false;
        kmlFullAlertVisible = false;
        // 重新渲染界面
        renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
        canvas.pushSprite(0, 0);
        return;
      }
      
      // 检查Tab键切换模式 (放在前面，确保模式切换优先)
      if (keys.tab) {
        static unsigned long lastTabPress = 0;
        const unsigned long TAB_DEBOUNCE_DELAY = 200;
        
        unsigned long currentTime = millis();
        if (currentTime - lastTabPress > TAB_DEBOUNCE_DELAY) {
          // 切换模式
          currentMode = (currentMode == MODE_HIKEPOD) ? MODE_GPS_INFO : MODE_HIKEPOD;
          modeChanged = true;
          lastTabPress = currentTime;
          Serial.printf("Switched to %s mode\n", currentMode == MODE_HIKEPOD ? "HikePod" : "GPS Info");
          // 清除菜单状态
          openMenu = false;
          helpMenu = false;
          infoMenu = false;
          configsMenu = false;
          fileSelectionMenuOpen = false;
        }
      }
      
      // 处理“`”键切换调试信息显示/隐藏
      for(auto key : keys.word) {
        if (key == '`') {
          static unsigned long lastBacktickPress = 0;
          const unsigned long BACKTICK_DEBOUNCE_DELAY = 200;
          
          unsigned long currentTime = millis();
          if (currentTime - lastBacktickPress > BACKTICK_DEBOUNCE_DELAY) {
            renderEngine.toggleDebugVisibility();
            lastBacktickPress = currentTime;
            Serial.println("Toggled debug info visibility");
          }
        } else if (key == 'h') {
          // 切换帮助菜单
          helpMenuVisible = !helpMenuVisible;
          if (helpMenuVisible) {
            drawHikePodHelpMenu(true);
          } else {
            drawHikePodHelpMenu(false);
          }
        } else if (key == 'v') {
          // 切换视图模式
          static unsigned long lastVPress = 0;
          const unsigned long V_DEBOUNCE_DELAY = 200;
          
          unsigned long currentTime = millis();
          if (currentTime - lastVPress > V_DEBOUNCE_DELAY) {
            currentViewMode = (currentViewMode == MODE_2D) ? MODE_3D : MODE_2D;
            renderEngine.setViewMode(currentViewMode);
            viewModeChanged = true;
            lastVPress = currentTime;
            Serial.printf("Switched to %s view mode\n", currentViewMode == MODE_2D ? "2D" : "3D");
            // 重新渲染界面
            renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
            // 确保亮度设置生效
            M5Cardputer.Display.setBrightness(screenBrightness);
            canvas.pushSprite(0, 0);
          }
        } else if (key == ']') {
          // 增加垂直放大系数
          static unsigned long lastBracketPress = 0;
          const unsigned long BRACKET_DEBOUNCE_DELAY = 200;
          
          unsigned long currentTime = millis();
          if (currentTime - lastBracketPress > BRACKET_DEBOUNCE_DELAY) {
            renderEngine.increaseVerticalExaggeration();
            lastBracketPress = currentTime;
            Serial.println("Increased vertical exaggeration");
            // 重新渲染界面
            renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
            canvas.pushSprite(0, 0);
          }
        } else if (key == '[') {
          // 减少垂直放大系数
          static unsigned long lastBracketPress2 = 0;
          const unsigned long BRACKET_DEBOUNCE_DELAY2 = 200;
          
          unsigned long currentTime = millis();
          if (currentTime - lastBracketPress2 > BRACKET_DEBOUNCE_DELAY2) {
            renderEngine.decreaseVerticalExaggeration();
            lastBracketPress2 = currentTime;
            Serial.println("Decreased vertical exaggeration");
            // 重新渲染界面
            renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
            canvas.pushSprite(0, 0);
          }
        } else if (key == '=' || key == '+') {
          // 3D视图缩放 - 放大
          if (currentViewMode == MODE_3D) {
            static unsigned long lastEqualPress = 0;
            const unsigned long EQUAL_DEBOUNCE_DELAY = 200;
            
            unsigned long currentTime = millis();
            if (currentTime - lastEqualPress > EQUAL_DEBOUNCE_DELAY) {
              renderEngine.zoom3D(1.2);  // 放大20%
              lastEqualPress = currentTime;
              Serial.println("3D Zoom in");
              renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
              canvas.pushSprite(0, 0);
            }
          }
        } else if (key == '-' || key == '_') {
          // 3D视图缩放 - 缩小
          if (currentViewMode == MODE_3D) {
            static unsigned long lastMinusPress = 0;
            const unsigned long MINUS_DEBOUNCE_DELAY = 200;
            
            unsigned long currentTime = millis();
            if (currentTime - lastMinusPress > MINUS_DEBOUNCE_DELAY) {
              renderEngine.zoom3D(0.8);  // 缩小20%
              lastMinusPress = currentTime;
              Serial.println("3D Zoom out");
              renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
              canvas.pushSprite(0, 0);
            }
          }
        } else if (key == ' ') {
          // 空格键切换旋转中心
          if (currentViewMode == MODE_3D) {
            static unsigned long lastSpacePress = 0;
            const unsigned long SPACE_DEBOUNCE_DELAY = 200;
            
            unsigned long currentTime = millis();
            if (currentTime - lastSpacePress > SPACE_DEBOUNCE_DELAY) {
              renderEngine.toggleRotationCenter();
              lastSpacePress = currentTime;
              renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
              canvas.pushSprite(0, 0);
            }
          }
        } else if (key == ';' || key == '.' || key == ',' || key == '/') {
          // 方向键用于3D视图平移（仅当没有菜单打开时）
          if (currentViewMode == MODE_3D && !settingsMenuOpen && !fileSelectionMenuOpen) {
            const int PAN_STEP = 10;
            if (key == ';') { // 上箭头
              renderEngine.pan3D(0, PAN_STEP);
            } else if (key == '.') { // 下箭头
              renderEngine.pan3D(0, -PAN_STEP);
            } else if (key == '/') { // 右箭头
              renderEngine.pan3D(-PAN_STEP, 0);
            } else if (key == ',') { // 左箭头
              renderEngine.pan3D(PAN_STEP, 0);
            } else if (key == '=') { // '+' 键用于 3D 缩放
              renderEngine.zoom3D(1.1f);
            } else if (key == '-') { // '-' 键用于 3D 缩放
              renderEngine.zoom3D(0.9f);
            }
            renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
            canvas.pushSprite(0, 0);
          }
        } else if (key == 'c' && currentMode == MODE_HIKEPOD) {
          // 处理“c”键打开设置菜单 (仅在 HikePod 模式下)
          static unsigned long lastCPress = 0;
          const unsigned long C_DEBOUNCE_DELAY = 200;
          
          unsigned long currentTime = millis();
          if (currentTime - lastCPress > C_DEBOUNCE_DELAY) {
            settingsMenuOpen = !settingsMenuOpen;
            if (settingsMenuOpen) {
              settingsMenuSelection = 0;
              drawSettingsMenu(true);
              Serial.println("Opened settings menu via handleControls");
            } else {
              drawSettingsMenu(false);
              Serial.println("Closed settings menu via handleControls");
            }
            lastCPress = currentTime;
          }
        } else if (key == 'w') {
          // WiFi 热点/HTTP 管理 (添加防抖并修复重复触发 Bug)
          static unsigned long lastWPress = 0;
          const unsigned long W_DEBOUNCE_DELAY = 200;
          
          unsigned long currentTime = millis();
          if (currentTime - lastWPress > W_DEBOUNCE_DELAY) {
            httpServerMenuOpen = !httpServerMenuOpen;
            if (httpServerMenuOpen) {
              initHttpServer();
              drawHttpServerWindow(true);
            } else {
              drawHttpServerWindow(false);
            }
            lastWPress = currentTime;
            Serial.printf("WiFi KML Manager toggled: %s\n", httpServerMenuOpen ? "ON" : "OFF");
          }
        }
      }
      
      // 处理设置菜单交互
      if (openMenu && settingsMenuOpen) {
        for(auto key : keys.word) {
          if (key == ';') { // 上箭头
            settingsMenuSelection = (settingsMenuSelection - 1 + SETTINGS_MENU_OPTIONS) % SETTINGS_MENU_OPTIONS;
            drawSettingsMenu(true);
          } else if (key == '.') { // 下箭头
            settingsMenuSelection = (settingsMenuSelection + 1) % SETTINGS_MENU_OPTIONS;
            drawSettingsMenu(true);
          } else if (key == ',') { // 左箭头/减少当前选项值
            switch(settingsMenuSelection) {
                case 1: // Brightness
                  if (screenBrightness >= BRIGHTNESS_MIN + BRIGHTNESS_STEP) {
                    screenBrightness -= BRIGHTNESS_STEP;
                  } else {
                    screenBrightness = BRIGHTNESS_MIN;
                  }
                  M5Cardputer.Display.setBrightness(screenBrightness);
                  break;
                case 2: // Screen Timeout
                  if (SCREEN_TIMEOUT > 0) {
                    SCREEN_TIMEOUT = (SCREEN_TIMEOUT <= 30000) ? 0UL : SCREEN_TIMEOUT - 30000;
                  }
                  break;
                case 3: // 正常GPS更新频率
                  if (GPS_UPDATE_INTERVAL_NORMAL > 500UL) {
                    GPS_UPDATE_INTERVAL_NORMAL = max(GPS_UPDATE_INTERVAL_NORMAL - 1000, 500UL);
                  }
                  break;
                case 4: // 息屏GPS更新频率
                  if (GPS_UPDATE_INTERVAL_SCREEN_OFF > 1000UL) {
                    GPS_UPDATE_INTERVAL_SCREEN_OFF = max(GPS_UPDATE_INTERVAL_SCREEN_OFF - 2000, 1000UL);
                  }
                  break;
                case 5: // Show POIs
                  showPOIsMode = (showPOIsMode + 1) % 3;
                  break;
            }
            drawSettingsMenu(true);
          } else if (key == '/') { // 右箭头/增加当前选项值
            switch(settingsMenuSelection) {
                case 1: // Brightness
                  if (screenBrightness <= BRIGHTNESS_MAX - BRIGHTNESS_STEP) {
                    screenBrightness += BRIGHTNESS_STEP;
                  } else {
                    screenBrightness = BRIGHTNESS_MAX;
                  }
                  M5Cardputer.Display.setBrightness(screenBrightness);
                  break;
                case 2: // Screen Timeout
                  SCREEN_TIMEOUT = min(SCREEN_TIMEOUT + 30000, 600000UL);
                  break;
                case 3: // 正常GPS更新频率
                  GPS_UPDATE_INTERVAL_NORMAL = min(GPS_UPDATE_INTERVAL_NORMAL + 1000, 10000UL);
                  break;
                case 4: // 息屏GPS更新频率
                  GPS_UPDATE_INTERVAL_SCREEN_OFF = min(GPS_UPDATE_INTERVAL_SCREEN_OFF + 2000, 30000UL);
                  break;
                case 5: // Show POIs
                  showPOIsMode = (showPOIsMode + 1) % 3;
                  break;
            }
            drawSettingsMenu(true);
          }
        }
        
        // 处理回车键选中
        if (keys.enter) {
          if (settingsMenuSelection == 0) { // 选择KML文件
            // 关闭设置菜单
            drawSettingsMenu(false);
            // 列出KML文件
            kmlFileList = listKMLFiles();
            
            if (kmlFileList.size() > 0) {
              selectedFileIndex = 0;
              fileSelectionMenuOpen = true;
              keyboardChanged = false;
              keyboardPressed = false;
              delay(100);
              drawFileSelectionMenu();
              Serial.println("Opened file selection menu");
              menuJustOpened = true;
            } else {
              Serial.println("No KML files found in HikePod directory");
              canvas.fillScreen(TFT_BLACK);
              canvas.setTextColor(TFT_RED, TFT_BLACK);
              canvas.setCursor(10, 50);
              canvas.println("No KML files found");
              canvas.pushSprite(0, 0);
              delay(1000);
              renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
              canvas.pushSprite(0, 0);
            }
          } else if (settingsMenuSelection == 5) { // Show POIs
            showPOIsMode = (showPOIsMode + 1) % 3;
            drawSettingsMenu(true);
          }
        }
      }
    }
  }
}

/*    Handle keyboard data inputs.
*/
void handleKeys(bool keyboardChanged, bool keyboardPressed, Keyboard_Class::KeysState keys) {
  if (configsMenu) {
    if (keyboardChanged) {
      if (keyboardPressed) {
        
        // Check if 'c' key is pressed to close the config menu
        for (auto c : keys.word) {
          if (c == 'c') {
            configsMenu = false;
            openMenu = false; // 确保菜单状态被重置
            updateScreen(true);
            return;
          }
        }
        
        // 处理数字输入
        for (auto c : keys.word) {
          // Arrow selection vertical up.
          if (c == ';' || c == '.') {
            configsMenuSel = (configsMenuSel + 1) % 3;
          }
          // Numbers 0-9.
          else if (c >= 48 && c <= 57) {
            configsTmp[configsMenuSel] += c;
          }
        }
        
        // Delete.
        if (keys.del && configsTmp[configsMenuSel].length() > 0) {
          configsTmp[configsMenuSel].remove(configsTmp[configsMenuSel].length() - 1);
        }
        
        // Store.
        if (keys.enter) {
          if (configsTmp[0].length() > 0)
            gpsRxPin = configsTmp[0].toInt();
          if (configsTmp[1].length() > 0)
            gpsTxPin = configsTmp[1].toInt();
          if (configsTmp[2].length() > 0)
            gpsBaud = configsTmp[2].toInt();
          configsTmp[0] = configsTmp[1] = configsTmp[2] = "";
          configsMenu = false;
          openMenu = false; // 确保菜单状态被重置
          updateScreen(true);
          return;
        }
        
        drawConfig(true);
      }
    }
  }
}
