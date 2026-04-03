# HikePod

## Language / 语言
[中文](#中文) | [English](#english)

## 中文

### 项目简介

HikePod是一个基于M5Stack Cardputer ADV的户外徒步导航项目，集成了GPS定位、轨迹记录、离线地图加载等功能，为户外爱好者提供便捷的导航工具。

### 技术要点

- **硬件需求**：M5Stack Cardputer ADV + CAP-LoRa-1262 + SD卡
- **GPS模块**：支持多种GNSS模块（如 CAP-1262），Rx/Tx引脚可在设置中动态修改
- **显示技术**：采用离屏渲染（M5GFX Canvas）与高效 UI 绘制，全局支持中文字体 (`efontCN_12`)
- **离线导航**：支持流式解析 KML 路径文件，具备动态下采样和缩放自动适配功能
- **电量优化**：徒步实测单次充电可支持约 7-8 小时连续息屏定位（配合 30s 亮屏超时）
- **轨迹记录**：实时记录徒步轨迹并生成 KML 文件

### 操作使用方法

#### 刷入固件说明
- **刷入固件**：通过 M5Burner 或 PlatformIO 将固件刷入 M5Stack Cardputer ADV
- **合并固件**：
   ```bash
   esptool --chip esp32s3 merge-bin -o cardputer-factory.bin --flash-mode dio --flash-size 8MB 0x0000 .pio/build/m5stack-cardputer/bootloader.bin 0x8000 .pio/build/m5stack-cardputer/partitions.bin 0xe000 ./boot_app0.bin 0x10000 .pio/build/m5stack-cardputer/firmware.bin
   ```
- **手动刷入**：
   ```bash
   esptool --chip auto --port COM8 --baud 1500000 --before default_reset write_flash -z 0x000 cardputer-factory.bin
   ```

### 操作说明

#### 基本操作

- **[h]**：打开帮助菜单
- **[c]**：打开设置菜单 (支持亮度、超时、GPS 间隔、POI 开关等选项)
- **[w]**：打开 WiFi KML 传输窗口 (HTTP 文件管理)
- **[Space]**：锁定/解锁当前位置
- **[t]**：切换轨迹记录模式
- **[v]**：切换 2D / 3D 视图
- **[TAB]**：切换到 GPS Info 详细模式
- **[ESC]**：切换调试日志显示
- **[+/-]**：放大 / 缩小地图 (缩放级别支持 5m 至 100km)
- **[方向键]**：平移地图

#### 离线 KML 文件准备

1. 从两步路等网站/APP 下载 KML 格式的路径文件（确保包含海拔信息）。
2. **支持**：现在已支持显示中文文件名的文件。
3. 将 KML 文件保存到 SD 卡的 `HikePod` 目录下，或通过 WiFi 传输。
4. 加载完成后，系统会自动计算最佳比例尺并居中显示起点。

#### 通过 WiFi 传输 KML 文件

1. 在主界面按下 **[w]** 键。
2. 连接 WiFi `HikePod_XXXX` (无密码)，访问 `http://192.168.4.1`。
3. 网页端支持上传、删除和在线预览已存储的路径文件。

### 功能说明

#### 主要功能

- **中文全支持**：文件名、设置菜单、状态信息均支持中文显示。
- **超大文件加载**：支持 3.5MB+ 大文件（约 2700+ 点）流式加载，自动下采样保证运行流畅。
- **自动视口适配**：加载 KML 后自动缩放至全景视角并对准起点。
- **关键位置(POI)显示**：支持解析并渲染路径中的关键点（如营地、水源、岔路等），支持 CDATA 格式名称。
- **实时定位与轨迹**：显示当前 GPS 位置、海拔、信噪比及卫星分布图。
- **海拔剖面图**：直观展示当前路径的海拔变化曲线。

#### 注意事项

- 为了省电，GPS 默认更新频率较低，低速移动时航向可能滞后。
- 3D 模式下垂直夸张度可调，方便在高原地区查看微小起伏。
- 本应用定位为“手机省电助手”，旨在徒步过程中代替手机常亮显示位置，不应替代专业救援设备。

### 截图展示

![HikePod](cover.jpg)
![Grid View](2bulu1.jpg)
![3D Path](2bulu2.jpg)

### 已完成增强 (2024.03 更新)
- [x] 全局中文字体支持 (`efontCN_12`)
- [x] 3.5MB+ 超大 KML 文件加载优化 (流式解析 + 自动下采样)
- [x] KML 关键位置 (POI) 解析与渲染修复 (支持 CDATA)
- [x] 自动缩放适配功能 (Auto-Fit Range)
- [x] 缩放级别扩展至 100KM，提升全局视野
- [x] GPS Info 模式排版与本地时区自动计算

### 后续计划
1. **省电策略优化**：进一步研究息屏下的低功耗运行。
2. **高程数据增强**：考虑支持更精细的海拔记录。
3. **支持 Cardputer 1.1**：适配非 ADV 版本（如果有需求）。

### 致谢

### 致谢与来源 (Credits & Acknowledgements)

本项目的 **GPS Info** 详细模式逻辑移植自 [Cardputer-GPS-Info](https://github.com/alcor55/Cardputer-GPS-Info) 项目。

在此特别感谢原作者 [alcor55](https://github.com/alcor55) 的优秀工作，为 HikePod 的卫星定位信息显示功能奠定了基础。

### 许可证

MIT License

## English

### Project Introduction

HikePod is an outdoor hiking navigation project based on M5Stack Cardputer ADV, integrating GPS positioning, track recording, offline map loading and other functions, providing convenient navigation tools for outdoor enthusiasts.

### Technical Points

- **Hardware Requirements**: M5Stack Cardputer ADV + CAP-LoRa-1262 + SD card
- **GPS Module**: Supports multiple GNSS modules (e.g., CAP-1262), Rx/Tx pins can be dynamically modified in settings
- **Display Technology**: Off-screen rendering (M5GFX Canvas) with efficient UI drawing, globally supports Chinese fonts (`efontCN_12`)
- **Offline Navigation**: Streaming parse for KML path files with dynamic downsampling and auto-fit view scaling
- **Power Optimization**: Field-tested to last 7-8 hours of continuous tracking with screen-off positioning
- **Track Recording**: Real-time hiking track recording and KML generation

### Operation Instructions

#### Basic Operations

- **[h]**: Open help menu
- **[c]**: Open settings menu (Brightness, Timeout, GPS interval, POI toggle, etc.)
- **[w]**: Open WiFi KML transfer window (HTTP file management)
- **[Space]**: Lock/unlock current position
- **[t]**: Toggle track recording mode
- **[v]**: Switch between 2D and 3D views
- **[TAB]**: Switch to detailed GPS Info mode
- **[ESC]**: Toggle debug log display
- **[+/-]**: Zoom in/out map (supports scales from 5m to 100km)
- **[Arrow keys]**: Pan map / view

#### Preparing Offline KML Files

1. Download KML path files from websites/apps like 2bulu (ensure altitude data is included).
2. **Supported**: Chinese filenames are now fully supported.
3. Save KML files to the `HikePod` directory on the SD card or transfer via WiFi.
4. After loading, the system will automatically calculate the best scale and center on the start point.

#### Transfer KML via WiFi

1. Press **[w]** on the main screen.
2. Connect to WiFi `HikePod_XXXX` (no password) and visit `http://192.168.4.1`.
3. The web interface supports uploading, deleting, and managing KML files.

### Function Description

#### Main Functions

- **Full Chinese Support**: Filenames, settings, and status info all support Chinese display.
- **Large File Support**: Stream loading for 3.5MB+ files (approx. 2700+ points) with auto-downsampling for smooth performance.
- **Auto-Fit Viewport**: Automatically scales to an overview and centers the start point upon loading KML.
- **POI Support**: Parses and renders Point of Interests (Campsites, Water sources, etc.) from KML, including CDATA names.
- **Real-time GPS**: Displays position, altitude, SNR, and satellite distribution map.
- **Altitude Profile**: Real-time altitude change curve display for the current path.

#### Notes

- For power saving, the default GPS update rate is low; heading may lag during slow movement.
- Vertical exaggeration in 3D mode is adjustable to visualize terrain in high-altitude areas.
- This app is a "phone battery saver" for tracking and should not replace professional rescue equipment.

### Screenshot Display

![HikePod](cover.jpg)
![Grid View](2bulu1.jpg)
![3D Path](2bulu2.jpg)

### Completed Enhancements (March 2024)
- [x] Global Chinese font support (`efontCN_12`)
- [x] Optimized 3.5MB+ large KML loading (Streaming + Auto-downsampling)
- [x] Fixed KML POI parsing and rendering (Full CDATA support)
- [x] Auto-fit viewport functionality
- [x] Expanded zoom range up to 100KM for better overview
- [x] GPS Info layout and local timezone auto-calculation

### Future Plans
1. **Low Power Strategy**: Deep sleep and further screen-off optimizations.
2. **Enhanced Altitude Data**: Finer resolution altitude logging.
3. **Cardputer 1.1 Support**: Adapt for non-ADV versions if requested.

### Acknowledgements

The **GPS Info** detailed mode in this project is ported or derived from the [Cardputer-GPS-Info](https://github.com/alcor55/Cardputer-GPS-Info) project. Special thanks to the original author, [alcor55](https://github.com/alcor55), for his excellent work which provided the foundation for the GPS satellite information display in HikePod.

### License

MIT License