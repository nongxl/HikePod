/**
 * @file main.cpp
 * @brief HikePod - Outdoor Navigation for M5Stack Cardputer
 * 
 * Note: The "GPS Info" display logic and satellite calculation routines 
 * are ported/derived from the "Cardputer-GPS-Info" project by alcor55.
 * Original project: https://github.com/alcor55/Cardputer-GPS-Info
 * 
 * @author HikePod Contributors
 * @license PolyForm Noncommercial License 1.0.0
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
#include "USB.h"
#include "USBMSC.h"
#include "IME/IME.h"
#include "I18n.h"


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

// 语言适配字体助手
inline void setUiFont() {
  if (I18n::getInstance().isChinese()) {
    canvas.setFont(&fonts::efontCN_12);
  } else {
    canvas.setFont(&fonts::Font0);
  }
}

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
void showNotTrackingAlert();
void drawHikePodHelpMenu(bool should_I);
void drawSettingsMenu(bool should_I);
void drawConfig(bool should_I);
void drawHelp(bool should_I);
void drawInfo(bool should_I);
void drawHttpServerWindow(bool should_I);
void enterUsbMscMode();
void drawWaypointInputDialog(bool should_I);
void drawTextInputDialog(bool should_I);
void showStatusToast(const String& msg);
void showNotTrackingAlert();
bool initSDCard();
// void initHttpServer(); // 移除旧函数
// void stopHttpServer(); // 移除旧函数

// SD卡状态
bool sdInitialized = false;
bool hasRoute = false;

// 地图浏览与跟随状态
static bool hasUserPanned = false;  // 跟踪用户是否已手动平移操作过地图

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
bool notTrackingAlertVisible = false; // 未在记录中提示信息框是否可见
bool helpMenuVisible = false; // 帮助菜单是否可见
int settingsMenuSelection = 0; // 当前选中的设置选项
int settingsMenuScrollOffset = 0; // 设置菜单滚动视口起始项索引
const int SETTINGS_VISIBLE_ITEMS = 4; // 设置菜单可视项数量
const int SETTINGS_MENU_OPTIONS = 7; // 设置选项数量（文件 + 亮度 + 超时 + 2个频率 + POI开关 + 语言）
String currentKmlFile = ""; // 当前加载的 KML 文件名
int showPOIsMode = 2;      // 关键点显示模式 (0:OFF, 1:ON, 2:AUTO)

// WiFi KML 管理窗口显示标志
bool httpServerMenuOpen = false;
// WebServer server(80); // 已移除
// String httpStatusMsg = ""; // 已移除

// USB MSC 文件传输模式标志与对象
bool usbMscModeOpen = false;
USBMSC msc;
static uint32_t usbReadCount = 0;
static uint32_t usbWriteCount = 0;
static uint64_t mscCardSizeMB = 0;

// 通用文本输入法弹窗状态
enum InputDialogType {
  INPUT_NONE = 0,
  INPUT_WAYPOINT_POI,        // 按 i 插入途经点
  INPUT_TRACKING_FILENAME,   // 按 t 开始记录自定义轨迹名
  INPUT_RENAME_KML           // 列表按 r 重命名 KML
};
InputDialogType currentInputType = INPUT_NONE;
bool inputDialogOpen = false;
#define waypointInputOpen inputDialogOpen
String inputDialogText = "";
#define waypointInputText inputDialogText
String inputDialogTitle = "";
String inputDialogOriginalFile = ""; // 重命名时的原文件名
unsigned long statusToastTime = 0;
String statusToastText = "";

void openTextInputDialog(InputDialogType type, const String& title, const String& defaultText = "", const String& origFile = "");
void showStatusToast(const String& msg) {
  statusToastText = msg;
  statusToastTime = millis();
}

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
unsigned long lastFileMenuActionTime = 0; // 文件选择菜单操作时间戳，防止 Enter 穿透

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

// GPS引脚和波特率 (v1.1 将在 setup 中动态分配以避免键盘冲突)
int gpsRxPin = -1; 
int gpsTxPin = -1; 
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
  canvas.setTextDatum(TL_DATUM);
  canvas.setCursor(10, 10);
  canvas.println(I18n::t(T_FILE_SELECT_TITLE));
  
  // 绘制分隔线
  canvas.drawLine(10, 25, SCREEN_WIDTH - 10, 25, TFT_BLACK);
  
  // 绘制文件列表
  int yPos = 32;
  int rowHeight = 20; // 增加行高以适应中文字体并留出间距
  int maxVisibleFiles = 5; // 配合行高减少显示行数，防止超出屏幕
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
      // 绘制选中项 - 使用蓝色高亮，高度增加到 20 以覆盖整行
      canvas.fillRect(5, yPos - 4, SCREEN_WIDTH - 25, rowHeight, TFT_BLUE);
      canvas.setTextColor(TFT_WHITE, TFT_BLUE);
    } else {
      canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    }
    
    canvas.setCursor(10, yPos);
    canvas.println(kmlFileList[i]);
    yPos += rowHeight; 
  }
  
  // 绘制滚动条
  if (kmlFileList.size() > maxVisibleFiles) {
    int scrollbarX = SCREEN_WIDTH - 8;
    int scrollbarY = 32;
    int scrollbarHeight = maxVisibleFiles * rowHeight;
    int scrollbarWidth = 4;
    
    // 绘制滚动条背景
    canvas.fillRect(scrollbarX, scrollbarY, scrollbarWidth, scrollbarHeight, 0xC618);
    
    // 计算滚动条滑块
    float total = kmlFileList.size();
    float visible = maxVisibleFiles;
    float scrollRatio = (float)startIndex / (total - visible);
    float thumbHeight = (visible / total) * scrollbarHeight;
    float thumbY = scrollbarY + scrollRatio * (scrollbarHeight - thumbHeight);
    
    canvas.fillRect(scrollbarX, thumbY, scrollbarWidth, thumbHeight, 0x7BEF);
  }
  
  // 底部操作提示
  setUiFont();
  canvas.setTextColor(TFT_BLUE, TFT_WHITE);
  canvas.setCursor(10, SCREEN_HEIGHT - (I18n::getInstance().isChinese() ? 13 : 11));
  canvas.print(I18n::t(T_FILE_SELECT_HINT));

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
  if (kmlParser) {
    delete kmlParser;
    kmlParser = nullptr;
    pointPool = nullptr;
  }
  Serial.printf("[KML] Before alloc: Free Heap=%d, Max Alloc Block=%d\n", (int)ESP.getFreeHeap(), (int)ESP.getMaxAllocHeap());
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

bool initSDCard() {
  #define SD_SCK 40
  #define SD_MISO 39
  #define SD_MOSI 14
  #define SD_CS 12
  
  Serial.println("Initializing SPI for SD card...");
  sdSPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);

  // 优先尝试 20MHz 高速 SPI，失败则阶梯降级到 10MHz、4MHz (参考 cardputer_Camera 提速策略)
  bool sdMounted = false;
  if (SD.begin(SD_CS, sdSPI, 20000000)) {
    sdMounted = true;
    Serial.println("SD card mounted at 20MHz SPI");
  } else if (SD.begin(SD_CS, sdSPI, 10000000)) {
    sdMounted = true;
    Serial.println("SD card mounted at 10MHz SPI");
  } else if (SD.begin(SD_CS, sdSPI, 4000000)) {
    sdMounted = true;
    Serial.println("SD card mounted at 4MHz SPI");
  }

  if (sdMounted) {
    if (SD.cardType() != CARD_NONE) {
      sdInitialized = true;
      Serial.println("SD card initialized successfully with independent SPI");
      return true;
    }
  }
  
  // 备用 SPI 初始化
  Serial.println("Attempting fallback to default SPI...");
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI);
  if (SD.begin(SD_CS, SPI, 20000000) || SD.begin(SD_CS, SPI, 10000000) || SD.begin(SD_CS)) {
    if (SD.cardType() != CARD_NONE) {
      sdInitialized = true;
      Serial.println("SD card initialized successfully with default SPI fallback");
      return true;
    }
  }
  
  sdInitialized = false;
  Serial.println("SD card initialization failed");
  return false;
}

static int32_t onUsbMscRead(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize) {
  if (!sdInitialized) return -1;
  uint32_t sector_count = bufsize / 512;
  for (uint32_t i = 0; i < sector_count; i++) {
    if (!SD.readRAW((uint8_t*)buffer + i * 512, lba + i)) {
      return -1;
    }
  }
  return bufsize;
}

static int32_t onUsbMscWrite(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize) {
  if (!sdInitialized) return -1;
  uint32_t sector_count = bufsize / 512;
  for (uint32_t i = 0; i < sector_count; i++) {
    if (!SD.writeRAW(buffer + i * 512, lba + i)) {
      return -1;
    }
  }
  return bufsize;
}

void enterUsbMscMode() {
  if (!sdInitialized) {
    showStatusToast(I18n::t(T_TOAST_SD_NOT_READY));
    return;
  }

  usbMscModeOpen = true;

  // 1. 如果当前正在记录轨迹，先安全停止并保存
  if (trackingManager.isTracking()) {
    trackingManager.stopTracking();
    renderEngine.setTrackingState(false);
  }

  // 2. 释放可能占用的路径点内存以释放系统资源
  if (kmlParser) {
    delete kmlParser;
    kmlParser = nullptr;
    pointPool = nullptr;
    totalPoints = 0;
    renderEngine.releaseWorldPoints();
  }

  // 3. 绘制 USB 传输界面 (统一 1 像素黑色单线边框，左对齐标题)
  canvas.fillRect(10, 10, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 20, TFT_WHITE);
  canvas.drawRect(10, 10, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 20, TFT_BLACK);

  setUiFont();
  canvas.setTextColor(TFT_BLUE, TFT_WHITE);
  canvas.setTextSize(1);
  canvas.setCursor(20, 18);
  canvas.print(I18n::t(T_USB_TITLE));

  uint64_t cardSizeMB = SD.cardSize() / (1024 * 1024);
  canvas.setTextColor(TFT_BLACK, TFT_WHITE);
  canvas.setCursor(20, 36);
  canvas.printf("%s%llu MB", I18n::t(T_USB_SD_CARD), cardSizeMB);

  canvas.setCursor(20, 52);
  canvas.setTextColor(TFT_DARKGREEN, TFT_WHITE);
  canvas.print(I18n::t(T_USB_STATUS_MOUNTED));

  canvas.setTextColor(TFT_BLACK, TFT_WHITE);
  canvas.setCursor(20, 70);
  canvas.print(I18n::t(T_USB_HINT_1));
  canvas.setCursor(20, 86);
  canvas.setTextColor(TFT_RED, TFT_WHITE);
  canvas.print(I18n::t(T_USB_HINT_2));

  canvas.setCursor(20, 106);
  canvas.setTextColor(TFT_BLUE, TFT_WHITE);
  canvas.print(I18n::t(T_USB_EXIT_HINT));

  canvas.pushSprite(0, 0);

  // 4. 等待 'u' 键释放，防止重复判定
  while (M5Cardputer.Keyboard.isKeyPressed('u')) {
    M5Cardputer.update();
    delay(10);
  }

  // 5. 启动 USB MSC (完全参考 cardputer_Camera 稳定规范)
  msc.onRead(onUsbMscRead);
  msc.onWrite(onUsbMscWrite);
  msc.mediaPresent(true);

  uint64_t csize = SD.cardSize();
  uint32_t sectors = (csize > 0) ? (csize / 512) : 1;
  msc.begin(sectors, 512);
  USB.begin();
  Serial.printf("[USB] Mass Storage started: %u sectors\n", sectors);

  // 6. 专有阻塞等待循环，专供 PC 端访问 SD 卡，避免任何后台任务与息屏休眠
  bool exitUsb = false;
  while (!exitUsb) {
    M5Cardputer.update();
    if (M5Cardputer.Keyboard.isKeyPressed('u') || M5Cardputer.Keyboard.isKeyPressed('`')) {
      exitUsb = true;
    }
    delay(50);
  }

  // 等待按键释放防抖
  while (M5Cardputer.Keyboard.isKeyPressed('u') || M5Cardputer.Keyboard.isKeyPressed('`')) {
    M5Cardputer.update();
    delay(10);
  }

  // 7. 停止 USB MSC 并重新挂载 SD 卡
  msc.mediaPresent(false);
  msc.end();

  SD.end();
  delay(200);
  initSDCard();

  if (currentKmlFile != "" && sdInitialized) {
    loadSelectedKMLFile(currentKmlFile);
  }

  usbMscModeOpen = false;
  lastActivityTime = millis(); // 刷新活动时间，防止退出后立即息屏
  Serial.println("[USB] Mass Storage stopped. SD card remounted.");

  // 重新渲染主界面
  renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(),
                      sdInitialized, hasRoute, pointPool, totalPoints,
                      kmlParser ? kmlParser->getPOIPool() : nullptr,
                      kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
  canvas.pushSprite(0, 0);
}


void setup() {
  // 首先初始化串口通信用于调试
  Serial.begin(115200);
  Serial.println("Starting HikePod setup...");

  // 初始化 i18n 国际化模块 (从 NVS 读取语言设置，默认英文)
  I18n::getInstance().begin();
  Serial.printf("Language initialized: %s\n", I18n::getInstance().isChinese() ? "Chinese" : "English");
  
  // 初始化M5Cardputer（按照M5Mp3的方式，先初始化M5Cardputer）
  Serial.println("Initializing M5Cardputer...");
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);  // 启用键盘 - Cardputer ADV正确方式

  // 为 Cardputer v1.1 自动适配 GPS 引脚 (防止与键盘矩阵 GPIO 13/15 冲突)
  if (M5.getBoard() == m5::board_t::board_M5Cardputer) {
      gpsRxPin = 2; // Cardputer v1.1 Grove G2
      gpsTxPin = 1; // Cardputer v1.1 Grove G1
      Serial.println("Cardputer v1.1 detected: Using Grove pins (G2/G1) for GPS to avoid keyboard conflict (G15/G13)");
  } else {
      gpsRxPin = 15; // Cardputer ADV 内部 GNSS Rx
      gpsTxPin = 13; // Cardputer ADV 内部 GNSS Tx
      Serial.println("Cardputer ADV detected: Using internal GNSS pins (G15/G13)");
  }
  
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
  // 警告：Cardputer v1.1 的矩阵键盘直接使用 GPIO (包含 5, 6, 8)，必须只在 ADV 版本执行此操作
  if (M5.getBoard() == m5::board_t::board_M5CardputerADV) {
    // 根据管脚映射，CAP-LoRa-1262模块的NSS引脚连接到Cardputer-Adv的G5
    #define LORA_CS 5
    pinMode(LORA_CS, OUTPUT);
    digitalWrite(LORA_CS, HIGH); // 设置为HIGH以永久禁用LoRa模块的SPI通信
    Serial.println("Permanently disabled LoRa module SPI communication (Cardputer ADV)");
    
    // 同时禁用LoRa模块的其他相关引脚，进一步减少电源消耗
    #define LORA_RST 8  // LoRa_RST连接到G8
    #define LORA_IRQ 6  // LoRa_IRQ连接到G6
    #define LORA_BUSY 10 // LoRa_BUSY连接到G10
    
    pinMode(LORA_RST, OUTPUT);
    digitalWrite(LORA_RST, LOW); // 设置为LOW以保持LoRa模块复位状态
    pinMode(LORA_IRQ, INPUT);
    pinMode(LORA_BUSY, INPUT);
    Serial.println("Disabled additional LoRa module pins to reduce power consumption (Cardputer ADV)");
  }
  
  // 优先初始化SD卡（使用独立的SPI对象）
  Serial.println("Attempting SD card initialization...");
  sdInitialized = false;
  hasRoute = false;
  
  if (initSDCard()) {
    Serial.println("SD card mounted successfully.");
    if (!SD.exists("/HikePod")) {
      SD.mkdir("/HikePod");
      Serial.println("Created /HikePod directory");
    }
  } else {
    Serial.println("SD card initialization failed");
  }

  // 初始化中文输入法 (词库直接内存映射自 Flash)
  IME::getInstance().begin();
  IME::getInstance().setActive(false);
  
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
  
  // 检查键盘状态变化（工业级时间去抖与连发过滤器：彻底解决微动按键触点弹跳连击与误触）
  bool rawEnter = M5Cardputer.Keyboard.isKeyPressed(0x28);
  bool rawDel   = M5Cardputer.Keyboard.isKeyPressed(0x2a);
  bool rawTab   = M5Cardputer.Keyboard.isKeyPressed(0x2b);

  const char check_chars[] = "hvcwsiop[]=+-_ t;.,/`abcdefghijklmnopqrstuvwxyz0123456789";
  const char shift_chars[] = "~!@#$%^&*()_{}:\"<>?|";

  // 按键去抖与长按状态结构
  struct KeyDebounceState {
    bool isDown = false;
    uint32_t pressTime = 0;
    uint32_t lastTriggerTime = 0;
    uint32_t releaseTime = 0;
  };
  static KeyDebounceState charStates[128];
  static KeyDebounceState enterState, delState, tabState;

  uint32_t now = millis();
  const uint32_t DEBOUNCE_MS = 35;       // 机械触点去抖时间窗：消除 5~30ms 触点回弹产生的假断开和假重按
  const uint32_t REPEAT_DELAY_MS = 450;  // 初始长按延迟：单击敲击在 450ms 内绝对只触发一次
  const uint32_t REPEAT_RATE_MS = 110;   // 连击触发周期：持续按住超过 450ms 后平滑连发

  auto processKey = [&](bool isPhysicallyDown, KeyDebounceState& state, bool allowRepeat) -> bool {
    if (isPhysicallyDown) {
      if (!state.isDown) {
        // 新按下：必须距上次释放超过去抖时间窗，防止按键抖动
        if (now - state.releaseTime >= DEBOUNCE_MS) {
          state.isDown = true;
          state.pressTime = now;
          state.lastTriggerTime = now;
          return true; // 触发按键事件
        }
      } else if (allowRepeat) {
        // 持续按住连发检测
        if (now - state.pressTime >= REPEAT_DELAY_MS) {
          if (now - state.lastTriggerTime >= REPEAT_RATE_MS) {
            state.lastTriggerTime = now;
            return true; // 连发触发
          }
        }
      }
    } else {
      if (state.isDown) {
        state.isDown = false;
        state.releaseTime = now;
      }
    }
    return false;
  };

  Keyboard_Class::KeysState keys;
  keys.reset();
  bool keyboardChanged = false;

  // Enter 绝不连发，防止按键穿透
  if (processKey(rawEnter, enterState, false)) {
    keys.enter = true;
    keyboardChanged = true;
  }
  // Del 允许连发，长按方便快速删除
  if (processKey(rawDel, delState, true)) {
    keys.del = true;
    keyboardChanged = true;
  }
  // Tab 不连发
  if (processKey(rawTab, tabState, false)) {
    keys.tab = true;
    keyboardChanged = true;
  }

  // 扫描字符键
  for (size_t i = 0; i < sizeof(check_chars) - 1; i++) {
    char c = check_chars[i];
    if (c >= 0 && c < 128) {
      bool isDown = M5Cardputer.Keyboard.isKeyPressed(c);
      // 方向键与空格键允许长按连按，普通字母敲击单次触发
      bool allowRepeat = (c == ';' || c == '.' || c == ',' || c == '/' || c == ' ');
      if (processKey(isDown, charStates[(uint8_t)c], allowRepeat)) {
        keys.word.push_back(c);
        keyboardChanged = true;
      }
    }
  }

  // 扫描 Shift 字符
  for (size_t i = 0; i < sizeof(shift_chars) - 1; i++) {
    char c = shift_chars[i];
    if (c >= 0 && c < 128) {
      bool isDown = M5Cardputer.Keyboard.isKeyPressed(c);
      if (processKey(isDown, charStates[(uint8_t)c], false)) {
        keys.word.push_back(c);
        keyboardChanged = true;
      }
    }
  }
  
  // 保持与项目后续逻辑兼容
  bool keyboardPressed = (keys.word.size() > 0 || keys.enter || keys.del || keys.tab);
  
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
  if (currentMode == MODE_HIKEPOD && !httpServerMenuOpen && !usbMscModeOpen && !waypointInputOpen) {
    // 将交互控制逻辑完全交由 handleControls 处理
    interactionManager.update(keyboardChanged, keyboardPressed, keys);
    
    // 检查是否需要重绘
    bool needRender = false;
    
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
    
    bool inStandbyMode = gnssModule.isInStandbyMode();
    if (inStandbyMode) {
      // 待机模式下保持1秒更新，保证记录精度
      gpsInterval = 1000;
    } else {
      // 正常工作模式：检查GNSS定位状态
      if (isGNSSSearching) {
        // 搜星阶段（未定位）：500ms检查一次
        gpsInterval = 500;
      } else {
        // 已定位阶段：1000ms（1秒）更新一次
        gpsInterval = 1000;
      }
    }
    
    // 检查是否到达更新时间
    static unsigned long lastGpsProcessTime = 0;
    if (currentTime - lastGpsProcessTime >= gpsInterval) {
      lastGpsProcessTime = currentTime;
      
      // 更新定位点锁定的动画状态（如果处于锁定状态）
      if (renderEngine.isLocationLockedState()) {
        needRender = true;
      }
      
      // 读取GNSS数据
      if (!inStandbyMode) {
        gnssModule.update();
        currentLocation = gnssModule.getCurrentLocation();
        
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
    
    // 检查空格键是否被按下（一键切换/恢复定位居中跟随状态）
    if (interactionManager.isSpaceKeyPressed()) {
      bool currentLocked = renderEngine.isLocationLockedState();
      bool newLocked = !currentLocked;
      renderEngine.setLocationLocked(newLocked);
      hasUserPanned = !newLocked;
      
      if (newLocked) {
        if (currentLocation.isValid) {
          if (currentViewMode == MODE_2D) {
            renderEngine.centerOnLocation(currentLocation.latitude, currentLocation.longitude);
            interactionManager.setPanOffset(0, 0);
          } else if (currentViewMode == MODE_3D) {
            renderEngine.center3DOnLocation(currentLocation);
          }
          showStatusToast(I18n::t(T_TOAST_FOLLOW_ON));
          Serial.println("[Location] Follow locked ON");
        } else {
          if (!routePoints.empty()) {
            Location startPoint = routePoints[0];
            if (currentViewMode == MODE_2D) {
              renderEngine.centerOnLocation(startPoint.latitude, startPoint.longitude);
              interactionManager.setPanOffset(0, 0);
            }
          }
          showStatusToast(I18n::t(T_ALERT_GPS_NO_FIX_1));
          Serial.println("[Location] Follow ON (Waiting GPS)");
        }
      } else {
        showStatusToast(I18n::t(T_TOAST_FOLLOW_OFF));
        Serial.println("[Location] Follow OFF (Free Pan Mode)");
      }
      
      needRender = true;
      interactionManager.resetSpaceKeyPressed();
    }
    
    // 检查t键是否被按下，用于启动/停止tracking模式
    if (interactionManager.isTKeyPressed()) {
      if (trackingManager.isTracking()) {
        // 停止tracking
        trackingManager.stopTracking();
        renderEngine.setTrackingState(false);
        showStatusToast(I18n::t(T_TOAST_TRACK_SAVED));
        Serial.println("[Tracking] Tracking stopped");
      } else {
        // 检查GPS定位是否有效
        if (!currentLocation.isValid) {
          // GPS未定位，显示提示信息框
          showGPSNoFixAlert();
        } else {
          // 呼出输入法弹窗，输入自定义轨迹名称（确认后自动拼接时间戳）
          openTextInputDialog(INPUT_TRACKING_FILENAME, I18n::t(T_DIALOG_TRACK_TITLE), "Track");
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
        
        // 如果定位处于锁定跟随状态，缩放后依然居中在当前定位
        if (renderEngine.isLocationLockedState() && currentLocation.isValid) {
          renderEngine.centerOnLocation(currentLocation.latitude, currentLocation.longitude);
          interactionManager.setPanOffset(0, 0);
        } else {
          // 同步平移偏移量和缩放级别到InteractionManager
          renderEngine.getPanOffset(panX, panY);
          interactionManager.setPanOffset(panX, panY);
        }
        // 同步缩放级别
        int currentZoom = renderEngine.getZoomLevel();
        interactionManager.setZoomLevel(currentZoom);
        
        static bool zoomAppliedLogged = false;
        if (!zoomAppliedLogged) {
          Serial.printf("[ZOOM DEBUG] Zoomed to level %d around point (%.6f, %.6f)\n", zoom, centerLat, centerLng);
          zoomAppliedLogged = true;
        }
      } else if (interactionManager.isPanChanged()) {
        // 平移操作：用户手动方向键平移，进入自由浏览模式
        hasUserPanned = true;
        renderEngine.setLocationLocked(false);
        renderEngine.setPanOffset(panX, panY);
        
        static bool panAppliedLogged = false;
        if (!panAppliedLogged) {
          Serial.printf("[PAN DEBUG] Applied pan: X=%d, Y=%d\n", panX, panY);
          panAppliedLogged = true;
        }
      }
      
      needRender = true;
    }
    
    // 如果还没有设置初始位置，不管是GPS还是路线，都尝试居中
    if (!initialPositionSet) {
      if (currentLocation.isValid) {
        if (currentViewMode == MODE_2D) {
          renderEngine.centerOnLocation(currentLocation.latitude, currentLocation.longitude);
          interactionManager.setPanOffset(0, 0);
        } else if (currentViewMode == MODE_3D) {
          renderEngine.center3DOnLocation(currentLocation);
        }
        hasUserPanned = false;
        initialPositionSet = true;
        Serial.println("[PAN DEBUG] Initial position set to GPS location");
      } else if (!routePoints.empty()) {
        Location startPoint = routePoints[0];
        if (currentViewMode == MODE_2D) {
          renderEngine.centerOnLocation(startPoint.latitude, startPoint.longitude);
          interactionManager.setPanOffset(0, 0);
        }
        initialPositionSet = true;
        Serial.println("[PAN DEBUG] Initial position set to route start point");
      }
    }
    // 徒步移动过程中实时跟随：只要处于锁定跟随状态且定位有效，无论是2D还是3D模式，始终保持定位焦点在屏幕中心！
    else if (renderEngine.isLocationLockedState() && currentLocation.isValid) {
      if (currentViewMode == MODE_2D) {
        renderEngine.centerOnLocation(currentLocation.latitude, currentLocation.longitude);
        interactionManager.setPanOffset(0, 0);
      } else if (currentViewMode == MODE_3D) {
        renderEngine.center3DOnLocation(currentLocation);
      }
    }
    
    // 更新之前的位置
    if (currentLocation.isValid) {
      prevLocation = currentLocation;
    }
    
    // 处理文件选择菜单
    if (fileSelectionMenuOpen) {
      if (inputDialogOpen) {
        return; // 输入法弹窗打开时优先处理，不拦截文件选择按键
      }
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
        
        // 处理导航键与重命名键
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
          } else if (key == 'r') { // 重命名选中文件
            if (selectedFileIndex >= 0 && selectedFileIndex < (int)kmlFileList.size()) {
              String orig = kmlFileList[selectedFileIndex];
              String defName = orig;
              if (defName.endsWith(".kml") || defName.endsWith(".KML")) {
                defName = defName.substring(0, defName.length() - 4);
              }
              openTextInputDialog(INPUT_RENAME_KML, I18n::t(T_DIALOG_RENAME_TITLE), defName, orig);
              hasNavigationKey = true;
            }
          } else if (key == 8 || key == '`') { // 退格键或 Esc 键作为取消退出（ASCII码8 或 `）
            // 取消
            Serial.println("Cancel/Esc key pressed in file selection menu");
            fileSelectionMenuOpen = false;
            openMenu = false;
            lastFileMenuActionTime = millis();
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
          if (menuJustOpened || millis() - lastFileMenuActionTime < 400) {
            // 忽略刚打开或刚从重命名对话框返回时的Enter键按下事件，彻底防止穿透加载文件
            Serial.println("Ignoring Enter key press in file selection menu (debounce / just opened)");
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
              lastFileMenuActionTime = millis();
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
      if (!openMenu && !gpsNoFixAlertVisible && !kmlFullAlertVisible && !notTrackingAlertVisible && !helpMenuVisible) { // 只有在没有菜单打开且没有提示信息框时才渲染
        // 渲染界面，传递内存池信息以绘制完整路径和已记录的轨迹
        renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
        if (millis() - statusToastTime < 2500 && statusToastText.length() > 0) {
          canvas.setFont(&fonts::efontCN_12);
          int w = canvas.textWidth(statusToastText.c_str());
          int x = (SCREEN_WIDTH - w) / 2;
          int y = SCREEN_HEIGHT - 18;
          canvas.setTextColor(TFT_BLACK);
          canvas.setCursor(x, y);
          canvas.print(statusToastText);
        }
        canvas.pushSprite(0, 0);  // 推送至屏幕
        lastRenderTime = currentTime;
      }
    }
  } else if (currentMode == MODE_GPS_INFO && !httpServerMenuOpen && !usbMscModeOpen) {
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
    canvas.drawRect(10, 10, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 20, TFT_BLACK);
    
    setUiFont();
    canvas.setTextColor(TFT_BLUE, TFT_WHITE);
    canvas.setTextSize(1);
    canvas.setCursor(20, 18);
    canvas.print(I18n::t(T_WIFI_TITLE));
    
    canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    canvas.setCursor(25, 35);
    String deviceId = String((uint32_t)ESP.getEfuseMac(), HEX).substring(0, 4);
    canvas.printf("SSID: HikePod_%s", deviceId.c_str());
    
    canvas.setCursor(25, 48);
    canvas.print("Password: (None)");
    
    canvas.setCursor(25, 65);
    canvas.setTextColor(TFT_RED, TFT_WHITE);
    canvas.setFont(&fonts::efontCN_12);
    canvas.println(wifiManager.getStatusMsg());
    
    canvas.setTextColor(TFT_BLACK, TFT_WHITE);
    canvas.setCursor(25, 82);
    canvas.println(I18n::getInstance().isChinese() ? "连接 WiFi 并访问 IP 地址以管理 KML" : "Connect to WiFi & Visit IP to manage KML");
    
    canvas.setCursor(20, 106);
    canvas.println(I18n::t(T_WIFI_HINT_EXIT));
    
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


void openTextInputDialog(InputDialogType type, const String& title, const String& defaultText, const String& origFile) {
  currentInputType = type;
  inputDialogTitle = title;
  inputDialogText = defaultText;
  inputDialogOriginalFile = origFile;
  openMenu = true;
  inputDialogOpen = true;

  IME& ime = IME::getInstance();
  ime.setActive(true);
  ime.reset();
  
  drawTextInputDialog(true);
}

void drawTextInputDialog(bool should_I) {
  if (should_I) {
    openMenu = true;
    inputDialogOpen = true;

    // 绘制统一风格窗口：(10, 10, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 20)，1 像素黑色单线边框
    canvas.fillRect(10, 10, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 20, TFT_WHITE);
    canvas.drawRect(10, 10, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 20, TFT_BLACK);

    // 标题居左对齐 (20, 18)
    setUiFont();
    canvas.setTextColor(TFT_BLUE, TFT_WHITE);
    canvas.setTextSize(1);
    canvas.setCursor(20, 18);
    canvas.print(inputDialogTitle.length() > 0 ? inputDialogTitle.c_str() : "Input");

    // 右上角输入法模式指示器 (拼 / EN)
    IME& ime = IME::getInstance();
    canvas.setFont(&fonts::Font0);
    if (ime.active()) {
      canvas.setTextColor(TFT_DARKGREEN, TFT_WHITE);
      canvas.setCursor(185, 20);
      canvas.print("[ZH/Pin]");
    } else {
      canvas.setTextColor(TFT_DARKGRAY, TFT_WHITE);
      canvas.setCursor(195, 20);
      canvas.print("[EN]");
    }

    // 输入框背景 (浅灰矩形)
    canvas.fillRect(20, 34, SCREEN_WIDTH - 40, 24, 0xF7BE);
    canvas.drawRect(20, 34, SCREEN_WIDTH - 40, 24, TFT_BLACK);

    // 绘制已输入的文字（使用支持中文的 efontCN_12）
    canvas.setFont(&fonts::efontCN_12);
    canvas.setCursor(24, 40);
    if (inputDialogText.length() > 0) {
      canvas.setTextColor(TFT_BLACK, 0xF7BE);
      canvas.print(inputDialogText);
    } else {
      canvas.setTextColor(TFT_DARKGRAY, 0xF7BE);
      if (currentInputType == INPUT_WAYPOINT_POI) {
        canvas.print("Type name or 1-8...");
      } else {
        canvas.print("Type file name...");
      }
    }

    // 绘制拼音拼写状态 vs 快捷预设提示
    if (ime.active() && ime.composing()) {
      // 拼写栏：> pinyin
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(TFT_BLUE, TFT_WHITE);
      canvas.setCursor(20, 64);
      canvas.printf("> %s", ime.composition().c_str());

      // 候选字列表
      canvas.setFont(&fonts::efontCN_12);
      const auto& cands = ime.candidates();
      int startX = 20;
      for (size_t i = 0; i < cands.size(); i++) {
        canvas.setTextColor(TFT_BLUE, TFT_WHITE);
        canvas.setCursor(startX, 80);
        canvas.printf("%d", (int)(i + 1));
        canvas.setTextColor(TFT_BLACK, TFT_WHITE);
        canvas.setCursor(startX + 8, 80);
        canvas.print(cands[i].c_str());
        startX += 22;
        if (startX > SCREEN_WIDTH - 30) break;
      }
      
      canvas.setFont(&fonts::Font0);
      canvas.setTextColor(TFT_DARKGRAY, TFT_WHITE);
      canvas.setCursor(20, 102);
      canvas.print("[1-9] Pick | [Spc] 1st | [;/.] Page");
    } else {
      if (currentInputType == INPUT_WAYPOINT_POI) {
        setUiFont();
        canvas.setTextColor(TFT_DARKGREEN, TFT_WHITE);
        canvas.setCursor(20, 64);
        canvas.print(I18n::t(T_DIALOG_POI_QUICK_1));
        canvas.setCursor(20, 78);
        canvas.print(I18n::t(T_DIALOG_POI_QUICK_2));

        setUiFont();
        canvas.setTextColor(TFT_BLUE, TFT_WHITE);
        canvas.setCursor(20, 102);
        canvas.print(I18n::t(T_DIALOG_SAVE_HINT));
      } else if (currentInputType == INPUT_TRACKING_FILENAME) {
        setUiFont();
        canvas.setTextColor(TFT_DARKGREEN, TFT_WHITE);
        canvas.setCursor(20, 64);
        canvas.print(I18n::t(T_DIALOG_TRACK_HINT_1));
        canvas.setCursor(20, 78);
        canvas.print(I18n::t(T_DIALOG_TRACK_HINT_2));

        setUiFont();
        canvas.setTextColor(TFT_BLUE, TFT_WHITE);
        canvas.setCursor(20, 102);
        canvas.print(I18n::t(T_DIALOG_START_HINT));
      } else { // INPUT_RENAME_KML
        setUiFont();
        canvas.setTextColor(TFT_DARKGREEN, TFT_WHITE);
        canvas.setCursor(20, 64);
        canvas.print(I18n::t(T_DIALOG_RENAME_HINT_1));
        canvas.setCursor(20, 78);
        canvas.print(I18n::t(T_DIALOG_RENAME_HINT_2));

        setUiFont();
        canvas.setTextColor(TFT_BLUE, TFT_WHITE);
        canvas.setCursor(20, 102);
        canvas.print(I18n::t(T_DIALOG_RENAME_HINT_3));
      }
    }

    canvas.pushSprite(0, 0);
  } else {
    inputDialogOpen = false;
    currentInputType = INPUT_NONE;
    openMenu = false;
    IME::getInstance().setActive(false);
    // 如果是从文件列表打开的重命名，退出时重绘文件列表
    if (fileSelectionMenuOpen) {
      menuJustOpened = true;
      lastFileMenuActionTime = millis();
      drawFileSelectionMenu();
    } else {
      renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(),
                          sdInitialized, hasRoute, pointPool, totalPoints,
                          kmlParser ? kmlParser->getPOIPool() : nullptr,
                          kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
      canvas.pushSprite(0, 0);
    }
  }
}

void drawWaypointInputDialog(bool should_I) {
  if (should_I) {
    openTextInputDialog(INPUT_WAYPOINT_POI, "Insert Waypoint (POI)", "");
  } else {
    drawTextInputDialog(false);
  }
}

void showGPSNoFixAlert() {
  gpsNoFixAlertVisible = true;
  
  // 绘制提示信息框
  canvas.fillRect(40, 48, SCREEN_WIDTH - 80, 44, TFT_WHITE);
  canvas.drawRect(40, 48, SCREEN_WIDTH - 80, 44, TFT_BLACK);
  setUiFont();
  canvas.setTextColor(TFT_BLACK, TFT_WHITE);
  canvas.setTextSize(1);
  
  // 显示提示信息
  canvas.setCursor(48, 62);
  canvas.println(I18n::t(T_ALERT_GPS_NO_FIX_1));
  canvas.setCursor(SCREEN_WIDTH - 65, 78);
  canvas.setTextColor(TFT_WHITE , TFT_BLACK);
  canvas.println("ok");
  
  canvas.pushSprite(0, 0);
}

void showKMLFullAlert() {
  kmlFullAlertVisible = true;
  
  // 绘制提示信息框 (参考 GPS no fix 样式)
  canvas.fillRect(30, 42, SCREEN_WIDTH - 60, 52, TFT_WHITE);
  canvas.drawRect(30, 42, SCREEN_WIDTH - 60, 52, TFT_BLACK);
  setUiFont();
  canvas.setTextColor(TFT_BLACK, TFT_WHITE);
  canvas.setTextSize(1);
  
  // 显示提示信息
  canvas.setCursor(38, 52);
  canvas.println(I18n::t(T_ALERT_KML_LIMIT_1));
  canvas.setCursor(38, 66);
  canvas.println(I18n::t(T_ALERT_KML_LIMIT_2));
  
  canvas.setCursor(SCREEN_WIDTH - 55, 80);
  canvas.setTextColor(TFT_WHITE , TFT_BLACK);
  canvas.println("ok");
  
  canvas.pushSprite(0, 0);
}

void showNotTrackingAlert() {
  notTrackingAlertVisible = true;
  
  // 绘制提示信息框 (参考 GPS no fix 样式)
  canvas.fillRect(30, 42, SCREEN_WIDTH - 60, 52, TFT_WHITE);
  canvas.drawRect(30, 42, SCREEN_WIDTH - 60, 52, TFT_BLACK);
  setUiFont();
  canvas.setTextColor(TFT_BLACK, TFT_WHITE);
  canvas.setTextSize(1);
  
  // 显示提示信息
  canvas.setCursor(38, 52);
  canvas.println(I18n::t(T_ALERT_NOT_TRACKING_1));
  canvas.setCursor(38, 66);
  canvas.println(I18n::t(T_ALERT_NOT_TRACKING_2));
  
  canvas.setCursor(SCREEN_WIDTH - 55, 80);
  canvas.setTextColor(TFT_WHITE, TFT_BLACK);
  canvas.println("ok");
  
  canvas.pushSprite(0, 0);
}

void drawHikePodHelpMenu(bool should_I) {
  if (should_I == true) {
    openMenu = true;
    helpMenuVisible = true;
    
    // 绘制帮助菜单边框，风格与c设置菜单统一
    canvas.fillRect(10, 10, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 20, TFT_WHITE);
    canvas.drawRect(10, 10, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 20, TFT_BLACK);
    
    // 显示帮助标题（统一在边框内 (20, 18) 居左对齐）
    setUiFont();
    canvas.setTextColor(TFT_BLUE, TFT_WHITE);
    canvas.setTextSize(1);
    canvas.setCursor(20, 18);
    canvas.print(I18n::t(T_HELP_TITLE));
    
    struct HelpItem {
      const char* key;
      const char* desc;
    };
    
    const HelpItem leftCol[] = {
      {"[h]", I18n::t(T_HELP_THIS)},
      {"[c]", I18n::t(T_HELP_SETTINGS)},
      {"[v]", I18n::t(T_HELP_VIEW)},
      {"[Spc]", I18n::t(T_HELP_LOCK)},
      {"[t]", I18n::t(T_HELP_TRACK)},
      {"[i]", I18n::t(T_HELP_INSERT_POI)},
      {"[TAB]", I18n::t(T_HELP_GPS_INFO)}
    };
    
    const HelpItem rightCol[] = {
      {"[u]", I18n::t(T_HELP_USB_DISK)},
      {"[w]", I18n::t(T_HELP_WIFI)},
      {"[+/-]", I18n::t(T_HELP_ZOOM)},
      {"[[/]]", I18n::t(T_HELP_VERT_SCALE)},
      {"[`]", I18n::t(T_HELP_DEBUG)},
      {"[Arr]", I18n::t(T_HELP_PAN)}
    };
    
    // 中间纵向浅灰分割线
    canvas.drawFastVLine(120, 30, 92, TFT_LIGHTGRAY);
    
    int yStart = 31;
    int yStep = 13;
    
    // 绘制左列
    for (int i = 0; i < 7; i++) {
      int y = yStart + i * yStep;
      canvas.setTextColor(TFT_BLUE, TFT_WHITE);
      canvas.setCursor(18, y);
      canvas.print(leftCol[i].key);
      canvas.setTextColor(TFT_BLACK, TFT_WHITE);
      canvas.setCursor(54, y);
      canvas.print(leftCol[i].desc);
    }
    
    // 绘制右列
    for (int i = 0; i < 6; i++) {
      int y = yStart + i * yStep;
      canvas.setTextColor(TFT_BLUE, TFT_WHITE);
      canvas.setCursor(124, y);
      canvas.print(rightCol[i].key);
      canvas.setTextColor(TFT_BLACK, TFT_WHITE);
      canvas.setCursor(160, y);
      canvas.print(rightCol[i].desc);
    }
    
    canvas.pushSprite(0, 0);
  } else {
    openMenu = false;
    helpMenuVisible = false;
    // 重新渲染界面
    renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(),
                        sdInitialized, hasRoute, pointPool, totalPoints,
                        kmlParser ? kmlParser->getPOIPool() : nullptr,
                        kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
    canvas.pushSprite(0, 0);
  }
}

void drawSettingsMenu(bool should_I) {
  if (should_I == true) {
    openMenu = true;
    settingsMenuOpen = true;

    // 自动修正视口偏移，确保选中项在可视区域内
    if (settingsMenuSelection < settingsMenuScrollOffset) {
      settingsMenuScrollOffset = settingsMenuSelection;
    } else if (settingsMenuSelection >= settingsMenuScrollOffset + SETTINGS_VISIBLE_ITEMS) {
      settingsMenuScrollOffset = settingsMenuSelection - SETTINGS_VISIBLE_ITEMS + 1;
    }
    if (settingsMenuScrollOffset < 0) settingsMenuScrollOffset = 0;
    if (settingsMenuScrollOffset > SETTINGS_MENU_OPTIONS - SETTINGS_VISIBLE_ITEMS) {
      settingsMenuScrollOffset = max(0, SETTINGS_MENU_OPTIONS - SETTINGS_VISIBLE_ITEMS);
    }

    // 白色背景与外框 (扩大至 224x123，留出充足内边距)
    canvas.fillRect(8, 6, SCREEN_WIDTH - 16, SCREEN_HEIGHT - 12, TFT_WHITE);
    canvas.drawRect(8, 6, SCREEN_WIDTH - 16, SCREEN_HEIGHT - 12, TFT_BLACK);
    
    // 显示标题栏 (下移至 y=13，与顶边留出安全内边距)
    setUiFont();
    canvas.setTextColor(TFT_BLUE, TFT_WHITE);
    canvas.setCursor(16, 13);
    canvas.print(I18n::t(T_SETTINGS_TITLE));
    canvas.drawFastHLine(10, 27, 220, 0xD6BA); // 标题分割线
    
    // 绘制可视选项 (SETTINGS_VISIBLE_ITEMS 项)
    const int itemStartY = 30;
    const int itemHeight = 13;
    for (int i = 0; i < SETTINGS_VISIBLE_ITEMS; i++) {
      int itemIdx = settingsMenuScrollOffset + i;
      if (itemIdx >= SETTINGS_MENU_OPTIONS) break;
      int curY = itemStartY + i * itemHeight;
      bool isSelected = (settingsMenuSelection == itemIdx);

      setUiFont();
      canvas.setTextColor(isSelected ? TFT_BLUE : TFT_BLACK, TFT_WHITE);
      canvas.setCursor(16, curY);

      switch (itemIdx) {
        case 0: // 选择 KML 文件
          canvas.print(I18n::t(T_SETTINGS_SELECT_KML));
          break;
        case 1: // 屏幕亮度
          canvas.print(I18n::t(T_SETTINGS_BRIGHTNESS));
          canvas.setTextColor(TFT_BLACK, TFT_WHITE);
          canvas.print(String(screenBrightness));
          break;
        case 2: // 屏幕超时时间
          canvas.print(I18n::t(T_SETTINGS_TIMEOUT));
          canvas.setTextColor(TFT_BLACK, TFT_WHITE);
          if (SCREEN_TIMEOUT == 0) {
            canvas.print(I18n::t(T_SETTINGS_TIMEOUT_NEVER));
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
          break;
        case 3: // 正常 GPS 更新频率
          canvas.print(I18n::t(T_SETTINGS_GPS_INT));
          canvas.setTextColor(TFT_BLACK, TFT_WHITE);
          canvas.print(String(GPS_UPDATE_INTERVAL_NORMAL / 1000) + "s");
          break;
        case 4: // 息屏 GPS 更新频率
          canvas.print(I18n::t(T_SETTINGS_SCROFF_GPS));
          canvas.setTextColor(TFT_BLACK, TFT_WHITE);
          canvas.print(String(GPS_UPDATE_INTERVAL_SCREEN_OFF / 1000) + "s");
          break;
        case 5: // 关键点显示模式
          canvas.print(I18n::t(T_SETTINGS_SHOW_POIS));
          canvas.setTextColor(TFT_BLACK, TFT_WHITE);
          if (showPOIsMode == 0) canvas.print("OFF");
          else if (showPOIsMode == 1) canvas.print("ON");
          else canvas.print("AUTO");
          break;
        case 6: // 语言设置
          canvas.print(I18n::t(T_SETTINGS_LANGUAGE));
          canvas.setTextColor(TFT_BLACK, TFT_WHITE);
          canvas.print(I18n::getInstance().isChinese() ? I18n::t(T_LANG_NAME_ZH) : I18n::t(T_LANG_NAME_EN));
          break;
      }
    }

    // 绘制右侧滚动条
    const int trackX = 222;
    const int trackY = 30;
    const int trackW = 3;
    const int trackH = 50;
    // 轨道背景槽
    canvas.fillRoundRect(trackX, trackY, trackW, trackH, 1, 0xDEFB);
    // 滑块
    int maxOffset = SETTINGS_MENU_OPTIONS - SETTINGS_VISIBLE_ITEMS;
    int thumbH = (trackH * SETTINGS_VISIBLE_ITEMS) / SETTINGS_MENU_OPTIONS;
    int thumbY = trackY;
    if (maxOffset > 0) {
      thumbY = trackY + ((trackH - thumbH) * settingsMenuScrollOffset) / maxOffset;
    }
    canvas.fillRoundRect(trackX, thumbY, trackW, thumbH, 1, TFT_BLUE);

    // 分割线 (菜单列表与电量图表之间)
    canvas.drawFastHLine(10, 83, 220, 0xEF7D);

    // 绘制电量消耗曲线
    if (batteryHistory.size() >= 1) {
      setUiFont();
      canvas.setTextColor(TFT_BLUE, TFT_WHITE);
      canvas.setCursor(16, 85);
      canvas.print(I18n::t(T_SETTINGS_BATTERY_TITLE));
      
      // 电量曲线配置
      const int CHART_HEIGHT = 18;
      const int CHART_WIDTH = SCREEN_WIDTH - 64; // 为左右两侧文字留出空间
      const int CHART_X = 40;
      const int CHART_Y = 98;
      
      // 计算电量范围
      int minBattery = 100;
      int maxBattery = 0;
      for (int bat : batteryHistory) {
        if (bat < minBattery) minBattery = bat;
        if (bat > maxBattery) maxBattery = bat;
      }
      
      int batteryRange = maxBattery - minBattery;
      if (batteryRange < 10) {
        minBattery = max(0, minBattery - 5);
        maxBattery = min(100, maxBattery + 5);
        batteryRange = 10;
      } else {
        minBattery = max(0, static_cast<int>(minBattery - batteryRange * 0.15));
        maxBattery = min(100, static_cast<int>(maxBattery + batteryRange * 0.15));
        batteryRange = maxBattery - minBattery;
      }
      
      if (batteryRange < 10) {
        batteryRange = 10;
        if (minBattery == maxBattery) {
          minBattery = max(0, minBattery - 5);
          maxBattery = min(100, maxBattery + 5);
        }
      }
      
      // 绘制电量折线
      int lastX = -1;
      int lastY = -1;
      int lastBattery = -1;
      
      for (size_t i = 0; i < batteryHistory.size(); i++) {
        int battery = batteryHistory[i];
        int x = CHART_X + 5 + (int)((double)i / (MAX_BATTERY_HISTORY - 1) * (CHART_WIDTH - 10));
        int batY = CHART_Y + CHART_HEIGHT - 5 - (int)((battery - minBattery) / (double)batteryRange * (CHART_HEIGHT - 10));
        
        if (batteryHistory.size() > 1 && lastX != -1 && lastY != -1) {
          int avgBattery = (lastBattery + battery) / 2;
          uint16_t lineColor = (avgBattery >= 20) ? TFT_BLUE : TFT_RED;
          canvas.drawLine(lastX, lastY, x, batY, lineColor);
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
        canvas.setFont(&fonts::Font0);
        canvas.setTextSize(1);
        canvas.setTextColor(TFT_BLACK, TFT_WHITE);
        canvas.setCursor(startX - 26, startY - 4);
        canvas.print(String(startBattery) + "%");
      }
      
      // 在曲线最右侧显示当前值
      if (batteryHistory.size() >= 1) {
        int currentBattery = batteryHistory[batteryHistory.size() - 1];
        int currentX = CHART_X + 5 + (int)((double)(batteryHistory.size() - 1) / (MAX_BATTERY_HISTORY - 1) * (CHART_WIDTH - 10));
        int currentY = CHART_Y + CHART_HEIGHT - 5 - (int)((currentBattery - minBattery) / (double)batteryRange * (CHART_HEIGHT - 10));
        canvas.setFont(&fonts::Font0);
        canvas.setTextSize(1);
        canvas.setTextColor(TFT_BLACK, TFT_WHITE);
        canvas.setCursor(currentX + 4, currentY - 4);
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
    {"$GBGSV", parseGSV},
    {"$GNGSV", parseGSV},
    {"$GPGSA", parseGSA},
    {"$GLGSA", parseGSA},
    {"$GAGSA", parseGSA},
    {"$BDGSA", parseGSA},
    {"$GBGSA", parseGSA},
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
  else if (line.startsWith("$BDGSV") || line.startsWith("$GBGSV")) system = "BeiDou";
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
  int x = 144;
  int y = 22; // 与数据表对齐
  int w = 96;
  int h = 96; // 8 行 * 12 像素 = 96
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
void drawSatelliteDataTab() {
  int x = 1;
  int y = 22; // 从 22 开始 (11 + 11)
  int h = 12; // 行高减小到 12
  // ... [保持原有 labels/values 逻辑]
  const char* c1labels[] = { "Lat", "Lng", "Alt", "Spd", "Crs", "Date", "Time", "HDOP" };
  char c1values[8][20];
  if (currentLocation.isValid) {
    sprintf(c1values[0], "%.6f", currentLocation.latitude);
    sprintf(c1values[1], "%.6f", currentLocation.longitude);
    sprintf(c1values[2], "%.2f", currentLocation.altitude);
    sprintf(c1values[3], "%.1f", gnssModule.getSpeedKmph());
    sprintf(c1values[4], "%.1f", gnssModule.getCourseDeg());
    if (gnssModule.isDateValid()) sprintf(c1values[5], "%02d/%02d/%02d", gnssModule.getLocalDay(), gnssModule.getLocalMonth(), gnssModule.getLocalYear() % 100); else sprintf(c1values[5], "0");
    if (gnssModule.isTimeValid()) sprintf(c1values[6], "%02d:%02d:%02d", gnssModule.getLocalHour(), gnssModule.getLocalMinute(), gnssModule.getLocalSecond()); else sprintf(c1values[6], "0");
    sprintf(c1values[7], "%.1f", gnssModule.getHDOP());
  } else {
    sprintf(c1values[0], "NoFix"); sprintf(c1values[1], "NoFix");
    for(int i=2; i<8; i++) sprintf(c1values[i], "0");
  }

  for (int i = 0; i < 8; i++) {
    canvas.fillRect(x, y, 90, h, TFT_BLACK);
    canvas.drawRect(x, y, 90, h, TFT_DARKGREY);
    canvas.setTextColor(TFT_WHITE);
    canvas.setTextDatum(TL_DATUM);
    canvas.setCursor(x + 4, y + 2); 
    canvas.printf("%s: %s", c1labels[i], c1values[i]);
    y += h; 
  }

  y = 22;
  x = 91;
  const char* c2labels[] = { "Seen", "Wish", "Used", "InFx", "GPS", "Gln", "Gal", "BDo" };
  char c2values[8][12];
  if (currentLocation.isValid) {
    int tAll = satellites.size(), tUsed = 0, tVis = 0, gV = 0, glV = 0, gaV = 0, bdV = 0;
    for (auto &sat : satellites) {
      if (sat.used) tUsed++;
      if (sat.visible) {
        tVis++;
        if (sat.system == "GPS") gV++;
        else if (sat.system == "GLONASS") glV++;
        else if (sat.system == "Galileo") gaV++;
        else if (sat.system == "BeiDou") bdV++;
      }
    }
    sprintf(c2values[0], "%d", tAll); sprintf(c2values[1], "%d", tVis); sprintf(c2values[2], "%d", tUsed);
    sprintf(c2values[3], "%d", gnssModule.getSatelliteCount());
    sprintf(c2values[4], "%d", gV); sprintf(c2values[5], "%d", glV);
    sprintf(c2values[6], "%d", gaV); sprintf(c2values[7], "%d", bdV);
  } else {
    for (int i = 0; i < 8; i++) sprintf(c2values[i], "0");
  }

  for (int i = 0; i < 8; i++) {
    canvas.fillRect(x, y, 53, h, TFT_BLACK);
    canvas.drawRect(x, y, 53, h, TFT_DARKGREY);
    canvas.setTextColor(TFT_WHITE);
    canvas.setTextDatum(TL_DATUM);
    canvas.setCursor(x + 4, y + 2);
    canvas.printf("%s: %s", c2labels[i], c2values[i]);
    y += h;
  }
}

void drawHeader(){
  int w = SCREEN_WIDTH;
  int h = 11; // 压缩到 11
  canvas.fillRect(0, 0, w, h, TFT_GREEN);
  canvas.setTextColor(TFT_BLACK);
  canvas.setTextDatum(MC_DATUM); 
  canvas.setTextSize(1);
  canvas.drawString("-= Cardputer GPS Info =-", w/2, h/2 + 1);
  
  canvas.fillRect(0, h, w, h, TFT_GREEN);
  canvas.drawString("[s]On/Off [c]Config [h]Help [Tab]Mode", w/2, h + h/2 + 1);
}

void drawStatus(){
  int y = 118; // 22 + 96
  int w = SCREEN_WIDTH;
  int h = 11; // 压缩到 11
  char buf[64];
  const char* gpsStr = (gpsSerialState == GPS_ON) ? "On" : (gpsSerialState == GPS_ERR ? "Err" : "Off");
  snprintf(buf, sizeof(buf), "GP:%s Rx:%d Tx:%d Bd:%d", gpsStr, gpsRxPin, gpsTxPin, gpsBaud);
  
  canvas.fillRect(0, y, w, h, TFT_BLACK);
  canvas.drawRect(0, y, w, h, TFT_DARKGREY);
  canvas.setTextColor(TFT_WHITE);
  canvas.setTextDatum(MC_DATUM);
  canvas.setTextSize(1);
  canvas.drawString(buf, w/2, y + h/2 + 1);
}

/*    Handle keyboard data inputs for GPS Info mode.
*/
void handleGPSInfoKeys(bool keyboardChanged, bool keyboardPressed, Keyboard_Class::KeysState keys) {
  // 处理GPS Info模式特定的键盘输入
  if(keyboardChanged) {
    if(keyboardPressed) {
      
      // 处理其他GPS Info模式的按键 (增加 200ms 消抖)
      static unsigned long lastGPSKeyTime = 0;
      const unsigned long GPS_KEY_DEBOUNCE = 200;
      unsigned long currentTime = millis();

      for(auto key : keys.word) {
        if (currentTime - lastGPSKeyTime > GPS_KEY_DEBOUNCE) {
          if (key == 's') {
            gpsSerial = !gpsSerial;
            initGPSSerial(gpsSerial);
            gpsSerialState = gpsSerial ? GPS_ON : GPS_OFF;
            drawStatus();
            lastGPSKeyTime = currentTime;
          }
          else if (key == 'c') {
            configsMenu = !configsMenu; // Invert status.
            drawConfig(configsMenu);
            lastGPSKeyTime = currentTime;
          }
          else if (key == 'h') {
            helpMenu = !helpMenu; // Invert status.
            drawHelp(helpMenu);
            lastGPSKeyTime = currentTime;
          }
          else if (key == 'i') {
            infoMenu = !infoMenu; // Invert status.
            drawInfo(infoMenu);
            lastGPSKeyTime = currentTime;
          }
          else if (key == 'p') {
            hidePlotId = !hidePlotId; // Invert status.
            lastGPSKeyTime = currentTime;
          }
          else if (key == 'o') {
            hidePlotSystem = !hidePlotSystem; // Invert status.
            lastGPSKeyTime = currentTime;
          }
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
    canvas.fillRect(10, 10, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 20, TFT_BLACK);
    canvas.drawRect(10, 10, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 20, TFT_GREEN);
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
    canvas.fillRect(10, 10, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 20, TFT_BLACK);
    canvas.drawRect(10, 10, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 20, TFT_GREEN);
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
    canvas.fillRect(10, 10, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 20, TFT_BLACK);
    canvas.drawRect(10, 10, SCREEN_WIDTH - 20, SCREEN_HEIGHT - 20, TFT_GREEN);
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
      // 如果通用文本输入对话框打开，全权接管所有按键
      if (inputDialogOpen) {
        IME& ime = IME::getInstance();

        // 1. ESC ('`') 退出输入法或取消当前拼写
        bool escPressed = false;
        for (auto key : keys.word) {
          if (key == '`') {
            escPressed = true;
            break;
          }
        }
        if (escPressed) {
          if (ime.active() && ime.composing()) {
            ime.reset();
            drawTextInputDialog(true);
          } else {
            drawTextInputDialog(false);
          }
          return;
        }

        // 2. Tab 键切换中英文输入法
        if (keys.tab) {
          ime.toggle();
          drawTextInputDialog(true);
          return;
        }

        // 3. 退格键 (keys.del)
        if (keys.del) {
          if (ime.active() && ime.composing()) {
            String dummyOut;
            ime.handleKey('\b', dummyOut);
          } else if (inputDialogText.length() > 0) {
            // UTF-8 安全退格
            do {
              inputDialogText.remove(inputDialogText.length() - 1);
            } while (inputDialogText.length() > 0 && 
                     ((uint8_t)inputDialogText[inputDialogText.length() - 1] & 0xC0) == 0x80);
          }
          drawTextInputDialog(true);
          return;
        }

        // 4. 回车键 (keys.enter)
        if (keys.enter) {
          if (ime.active() && ime.composing()) {
            // 正在拼写：首选候选字上屏
            String out;
            ime.handleKey('\n', out);
            if (out.length() > 0) {
              inputDialogText += out;
            }
            drawTextInputDialog(true);
            return;
          } else {
            // 根据当前业务类型执行提交操作
            if (currentInputType == INPUT_WAYPOINT_POI) {
              if (inputDialogText.length() == 0) {
                inputDialogText = I18n::t(T_TOAST_DEFAULT_POI);
              }
              trackingManager.addWaypoint(inputDialogText, currentLocation);
              showStatusToast(String(I18n::t(T_TOAST_MARKED_PREFIX)) + inputDialogText);
              Serial.printf("[POI] Added waypoint: %s at %.6f, %.6f\n",
                            inputDialogText.c_str(), currentLocation.latitude, currentLocation.longitude);
              drawTextInputDialog(false);
            } else if (currentInputType == INPUT_TRACKING_FILENAME) {
              if (inputDialogText.length() == 0) {
                inputDialogText = "Track";
              }
              if (trackingManager.startTracking(inputDialogText)) {
                renderEngine.setTrackingState(true);
                showStatusToast(String(I18n::t(T_TOAST_STARTED_PREFIX)) + inputDialogText);
                Serial.printf("[Tracking] Started tracking: %s\n", inputDialogText.c_str());
                if (gnssModule.isInStandbyMode()) {
                  gnssModule.exitStandbyMode();
                  gnssModule.setStandbyMode(false);
                }
              } else {
                showStatusToast(I18n::t(T_TOAST_START_FAILED));
              }
              drawTextInputDialog(false);
            } else if (currentInputType == INPUT_RENAME_KML) {
              if (inputDialogText.length() == 0) {
                showStatusToast(I18n::t(T_TOAST_NAME_EMPTY));
                drawTextInputDialog(true);
                return;
              }
              String newFileName = inputDialogText;
              if (!newFileName.endsWith(".kml") && !newFileName.endsWith(".KML")) {
                newFileName += ".kml";
              }

              // 清洗路径，规范化为 /HikePod/filename.kml，防止双斜杠导致 SD.rename 失败
              String cleanOld = inputDialogOriginalFile;
              while (cleanOld.startsWith("/")) cleanOld = cleanOld.substring(1);
              if (cleanOld.startsWith("HikePod/")) cleanOld = cleanOld.substring(8);
              while (cleanOld.startsWith("/")) cleanOld = cleanOld.substring(1);

              String cleanNew = newFileName;
              while (cleanNew.startsWith("/")) cleanNew = cleanNew.substring(1);
              if (cleanNew.startsWith("HikePod/")) cleanNew = cleanNew.substring(8);
              while (cleanNew.startsWith("/")) cleanNew = cleanNew.substring(1);

              String oldPath = String("/HikePod/") + cleanOld;
              String newPath = String("/HikePod/") + cleanNew;
              Serial.printf("[File] Rename request: %s -> %s\n", oldPath.c_str(), newPath.c_str());

              if (oldPath != newPath) {
                if (SD.exists(newPath)) {
                  showStatusToast(I18n::t(T_TOAST_FILE_EXISTS));
                } else if (SD.rename(oldPath, newPath)) {
                  showStatusToast(I18n::t(T_TOAST_RENAME_OK));
                  Serial.printf("[File] Renamed %s to %s successfully\n", oldPath.c_str(), newPath.c_str());
                  kmlFileList = listKMLFiles();
                  for (size_t i = 0; i < kmlFileList.size(); i++) {
                    if (kmlFileList[i] == cleanNew) {
                      selectedFileIndex = i;
                      break;
                    }
                  }
                } else {
                  showStatusToast(I18n::t(T_TOAST_RENAME_FAILED));
                  Serial.printf("[File] SD.rename failed from %s to %s!\n", oldPath.c_str(), newPath.c_str());
                }
              }
              menuJustOpened = true;
              lastFileMenuActionTime = millis();
              drawTextInputDialog(false);
              return;
            }
            return;
          }
        }

        // 5. 普通按键输入
        bool needRedraw = false;
        for (auto key : keys.word) {
          if (ime.active()) {
            // 中文模式
            if (!ime.composing() && (key >= '1' && key <= '8') && inputDialogText.length() == 0 && currentInputType == INPUT_WAYPOINT_POI) {
              // 仅在途经点输入且文本为空时按 1-8：快捷词上屏
              const char* quickWords[] = {
                "直行", "左转", "右转", "下坡", "打卡", "营地", "水源", "危险"
              };
              inputDialogText += quickWords[key - '1'];
              needRedraw = true;
            } else {
              String out;
              bool consumed = ime.handleKey(key, out);
              if (out.length() > 0) {
                inputDialogText += out;
                needRedraw = true;
              } else if (consumed) {
                needRedraw = true;
              } else if (!ime.composing() && key >= 32 && key <= 126) {
                inputDialogText += (char)key;
                needRedraw = true;
              }
            }
          } else {
            // 英文模式
            if (key >= '1' && key <= '8' && inputDialogText.length() == 0 && currentInputType == INPUT_WAYPOINT_POI) {
              const char* quickWords[] = {
                "Straight", "Left", "Right", "Down", "CheckIn", "Camp", "Water", "Danger"
              };
              inputDialogText += quickWords[key - '1'];
              needRedraw = true;
            } else if (key >= 32 && key <= 126) {
              inputDialogText += (char)key;
              needRedraw = true;
            }
          }
        }

        if (needRedraw) {
          drawTextInputDialog(true);
        }
        return;
      }

      // 如果 WiFi KML Manager 窗口打开，响应 'w' 或 '`' (Esc) 键退出
      if (httpServerMenuOpen) {
        bool closeWifi = false;
        for (auto key : keys.word) {
          if (key == 'w' || key == '`') {
            closeWifi = true;
            break;
          }
        }
        if (closeWifi) {
          httpServerMenuOpen = false;
          drawHttpServerWindow(false);
          return;
        }
        return; // 如果没按 'w' 或 '`'，直接忽略所有其他按键
      }

      // 处理提示信息框的关闭
      if (gpsNoFixAlertVisible || kmlFullAlertVisible || notTrackingAlertVisible) {
        bool dismiss = keys.enter;
        for (auto k : keys.word) {
          if (k == '`') dismiss = true;
        }
        if (dismiss) {
          gpsNoFixAlertVisible = false;
          kmlFullAlertVisible = false;
          notTrackingAlertVisible = false;
          // 重新渲染界面
          renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
          canvas.pushSprite(0, 0);
          return;
        }
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
      
      // 如果帮助菜单打开，按 'h' 或 '`' (Esc) 键退出
      if (helpMenuVisible) {
        bool closeHelp = false;
        for (auto key : keys.word) {
          if (key == 'h' || key == '`') {
            closeHelp = true;
            break;
          }
        }
        if (closeHelp) {
          helpMenuVisible = false;
          drawHikePodHelpMenu(false);
          return;
        }
      }

      // 处理“`”键切换调试信息显示/隐藏 (仅在没有任何窗口/菜单打开时响应)
      for(auto key : keys.word) {
        if (key == '`') {
          if (!settingsMenuOpen && !helpMenuVisible && !fileSelectionMenuOpen && 
              !httpServerMenuOpen && !inputDialogOpen && !gpsNoFixAlertVisible && 
              !kmlFullAlertVisible && !notTrackingAlertVisible) {
            static unsigned long lastBacktickPress = 0;
            const unsigned long BACKTICK_DEBOUNCE_DELAY = 200;
            
            unsigned long currentTime = millis();
            if (currentTime - lastBacktickPress > BACKTICK_DEBOUNCE_DELAY) {
              renderEngine.toggleDebugVisibility();
              lastBacktickPress = currentTime;
              Serial.println("Toggled debug info visibility");
            }
          }
        } else if (key == 'h' && currentMode == MODE_HIKEPOD) {
          // 切换帮助菜单 (增加 200ms 消抖)
          static unsigned long lastHPress = 0;
          const unsigned long H_DEBOUNCE_DELAY = 200;
          unsigned long currentTime = millis();
          if (currentTime - lastHPress > H_DEBOUNCE_DELAY) {
            helpMenuVisible = !helpMenuVisible;
            if (helpMenuVisible) {
              drawHikePodHelpMenu(true);
            } else {
              drawHikePodHelpMenu(false);
            }
            lastHPress = currentTime;
          }
        } else if (key == 'v' && currentMode == MODE_HIKEPOD) {
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
        } else if (key == ']' && currentMode == MODE_HIKEPOD) {
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
        } else if (key == '[' && currentMode == MODE_HIKEPOD) {
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
        } else if ((key == '=' || key == '+') && currentMode == MODE_HIKEPOD) {
          // 3D视图缩放 - 放大
          if (currentViewMode == MODE_3D) {
            static unsigned long lastEqualPress = 0;
            const unsigned long EQUAL_DEBOUNCE_DELAY = 200;
            
            unsigned long currentTime = millis();
            if (currentTime - lastEqualPress > EQUAL_DEBOUNCE_DELAY) {
              renderEngine.zoom3D(1.2);  // 放大20%
              lastEqualPress = currentTime;
              Serial.println("3D Zoom in");
              if (renderEngine.isLocationLockedState() && currentLocation.isValid) {
                renderEngine.center3DOnLocation(currentLocation);
              }
              renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
              canvas.pushSprite(0, 0);
            }
          }
        } else if ((key == '-' || key == '_') && currentMode == MODE_HIKEPOD) {
          // 3D视图缩放 - 缩小
          if (currentViewMode == MODE_3D) {
            static unsigned long lastMinusPress = 0;
            const unsigned long MINUS_DEBOUNCE_DELAY = 200;
            
            unsigned long currentTime = millis();
            if (currentTime - lastMinusPress > MINUS_DEBOUNCE_DELAY) {
              renderEngine.zoom3D(0.8);  // 缩小20%
              lastMinusPress = currentTime;
              Serial.println("3D Zoom out");
              if (renderEngine.isLocationLockedState() && currentLocation.isValid) {
                renderEngine.center3DOnLocation(currentLocation);
              }
              renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
              canvas.pushSprite(0, 0);
            }
          }
        } else if ((key == 'r' || key == 'R') && currentMode == MODE_HIKEPOD) {
          // 'r' 键切换 3D 旋转中心（起点 / 网格中心）
          if (currentViewMode == MODE_3D) {
            static unsigned long lastRPress = 0;
            const unsigned long R_DEBOUNCE_DELAY = 200;
            
            unsigned long currentTime = millis();
            if (currentTime - lastRPress > R_DEBOUNCE_DELAY) {
              renderEngine.toggleRotationCenter();
              lastRPress = currentTime;
              renderEngine.render(routePoints, currentLocation, trackingManager.getTrackPoints(), sdInitialized, hasRoute, pointPool, totalPoints, kmlParser ? kmlParser->getPOIPool() : nullptr, kmlParser ? kmlParser->getPOICount() : 0, showPOIsMode);
              canvas.pushSprite(0, 0);
            }
          }
        } else if ((key == ';' || key == '.' || key == ',' || key == '/') && currentMode == MODE_HIKEPOD) {
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
            }
            // 3D手动平移后解除锁定，允许自由浏览
            renderEngine.setLocationLocked(false);
            hasUserPanned = true;

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
              settingsMenuScrollOffset = 0;
              drawSettingsMenu(true);
              Serial.println("Opened settings menu via handleControls");
            } else {
              drawSettingsMenu(false);
              Serial.println("Closed settings menu via handleControls");
            }
            lastCPress = currentTime;
          }
        } else if (key == 'w' && currentMode == MODE_HIKEPOD) {
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
        } else if (key == 'u' && currentMode == MODE_HIKEPOD) {
          // USB MSC 文件传输模式 (带 200ms 防抖)
          static unsigned long lastUPress = 0;
          const unsigned long U_DEBOUNCE_DELAY = 200;
          
          unsigned long currentTime = millis();
          if (currentTime - lastUPress > U_DEBOUNCE_DELAY) {
            lastUPress = currentTime;
            enterUsbMscMode();
          }
        } else if (key == 'i' && currentMode == MODE_HIKEPOD) {
          // 插入途经标注点 (POI)
          static unsigned long lastIPress = 0;
          const unsigned long I_DEBOUNCE_DELAY = 250;
          unsigned long currentTime = millis();
          if (currentTime - lastIPress > I_DEBOUNCE_DELAY) {
            lastIPress = currentTime;
            if (trackingManager.isTracking()) {
              openTextInputDialog(INPUT_WAYPOINT_POI, I18n::t(T_DIALOG_POI_TITLE), "");
              Serial.println("[POI] Opened waypoint input dialog");
            } else {
              showNotTrackingAlert();
              Serial.println("[POI] Cannot insert POI: not in tracking mode. Press T first.");
            }
          }
        }
      }
      
      // 处理设置菜单交互
      if (openMenu && settingsMenuOpen) {
        for(auto key : keys.word) {
          if (key == '`') { // Esc 键退出设置菜单
            drawSettingsMenu(false);
            return;
          } else if (key == ';') { // 上箭头
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
                case 6: // Language (中英切换并保存至 NVS)
                  if (I18n::getInstance().isChinese()) {
                    I18n::getInstance().setLanguage(LANG_EN);
                  } else {
                    I18n::getInstance().setLanguage(LANG_ZH);
                  }
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
                case 6: // Language (中英切换并保存至 NVS)
                  if (I18n::getInstance().isChinese()) {
                    I18n::getInstance().setLanguage(LANG_EN);
                  } else {
                    I18n::getInstance().setLanguage(LANG_ZH);
                  }
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
