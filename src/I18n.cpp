#include "I18n.h"

const char* const I18n::_strings[2][T_COUNT] = {
    // -------------------------------------------------------------
    // 0: LANG_EN (English - Default)
    // -------------------------------------------------------------
    {
        // Settings
        "HikePod Settings",
        "> Select KML File ",
        "> Brightness (10-255) : ",
        "> Screen Timeout : ",
        "Never",
        "> GPS Int : ",
        "> ScreenOff GPS : ",
        "> Show POIs : ",
        "> GPS Module : ",
        "> Language : ",
        "English",
        "Chinese",
        "Battery Consumption :",

        // GPS Module Window
        "GPS Module Setup",
        "Cap LoRa-1262 (RX:15 TX:13)",
        "Unit GPS v1.1 (RX:1 TX:0)",
        "Custom Pins...",

        // KML List
        "Select KML File",
        "[Enter] Load | [r] Rename | [BS] Exit",

        // Help
        "HikePod Help",
        "Help (this)",
        "Settings",
        "2D/3D view",
        "Lock/Center",
        "Track toggle",
        "Insert POI",
        "GPS Info",
        "USB disk",
        "WiFi manager",
        "Zoom in/out",
        "Vert scale",
        "Debug info",
        "Pan map",
        "Toggle GPS",

        // USB
        "USB Transfer (MSC)",
        "SD Card: ",
        "Status: Mounted as USB Drive",
        "1. Drag & drop KML to /HikePod",
        "2. Safely eject on PC first!",
        "Press 'u' or `(ESC) to exit",

        // WiFi
        "WiFi KML Manager",
        "Status: Running",
        "Connect to: ",
        "Browser visit: ",
        "Press 'w' to close and return",

        // Alert
        "GPS no fix !",
        "KML Point Limit!",
        "Some points skipped.",
        "Not tracking !",
        "Press 't' to start.",

        // Toast
        "SD Card Not Ready",
        "Track Saved",
        "Marked: ",
        "Waypoint",
        "Started: ",
        "Failed to Start Track",
        "Filename Cannot Be Empty",
        "File Already Exists",
        "Rename Succeeded",
        "Rename Failed",
        "GPS Follow: ON",
        "Free Pan Mode",
        "GPS: ON",
        "GPS: OFF",
        "Applied Module: ",

        // Dialog
        "Insert Waypoint (POI)",
        "Start Track: File Name",
        "Rename KML File",
        "Quick: 1Straight 2Left 3Right 4Down",
        "       5CheckIn  6Camp 7Water 8Danger",
        "[Enter] Save | [ESC] Exit | [Tab] IME",
        "Timestamp will be appended:",
        "e.g. Name_YYYYMMDD_HHMMSS",
        "[Enter] Start | [ESC] Exit | [Tab] IME",
        "Enter new name (.kml not needed)",
        "Press Enter to confirm rename",
        "[Enter] Rename | [ESC] Exit | [Tab] IME",

        // Custom GPS
        "Custom GPS Module Setup",
        "RX Pin (Receive):",
        "TX Pin (Transmit):",
        "Baud Rate (bps):",

        "GPS: Not Started",
        "Press 's' to start GPS",
        "No data! Check GPS setup (c)",
        "Found module ",
        "Press 'y' to switch",
        "Fix timeout! Move to open sky",

        // GPS Detected Dialog
        "GPS Module Detected",
        "Data stream active. Switch?",
        "[Y/Enter] Confirm Switch",
        "[N/ESC] Ignore",
    },

    // -------------------------------------------------------------
    // 1: LANG_ZH (Simplified Chinese)
    // -------------------------------------------------------------
    {
        // Settings
        "HikePod 设置",
        "> 选择 KML 文件 ",
        "> 屏幕亮度 (10-255) : ",
        "> 息屏超时 : ",
        "从不",
        "> GPS 运行间隔 : ",
        "> 息屏 GPS 间隔 : ",
        "> 关键点显示 : ",
        "> GPS 硬件模块 : ",
        "> 系统语言 : ",
        "英文",
        "简体中文",
        "电量消耗曲线 :",

        // GPS 模块窗口
        "选择 GPS 硬件模块",
        "Cap LoRa-1262 (RX:15 TX:13)",
        "Unit GPS v1.1 (RX:1 TX:0)",
        "自定义引脚...",

        // KML List
        "选择路线 KML 文件",
        "[Enter] 加载 | [r] 重命名 | [BS] 退出",

        // Help
        "HikePod 操作指南",
        "帮助菜单 (本页)",
        "系统设置",
        "切换 2D/3D",
        "定位居中/自由平移",
        "轨迹记录 开/关",
        "打卡标注点",
        "GPS 信息",
        "USB 存储",
        "WiFi 传路线",
        "缩放地图",
        "3D 地形高度",
        "调试抽屉面板",
        "移动地图",
        "开关 GPS",

        // USB
        "USB 大容量存储 (MSC)",
        "SD 卡容量: ",
        "状态: 已挂载为电脑 U 盘",
        "1. 将 KML 文件复制到 /HikePod 目录",
        "2. 断开前请先在电脑上“安全弹出”！",
        "按 'u' 或 `(ESC) 退出 U 盘模式",

        // WiFi
        "WiFi 路线管理",
        "状态: 正在运行",
        "已连接至 WiFi: ",
        "请在手机/电脑浏览器打开:",
        "按 'w' 或 `(ESC) 退出 WiFi 管理",

        // Alert
        "当前 GPS 未定位！",
        "路线点数已达上限 (1500点)！",
        "已智能跳过后续多余点",
        "未开启轨迹记录模式！",
        "按 't' 键开启记录后方可打卡标注",

        // Toast
        "SD卡未就绪，无法操作",
        "轨迹已保存至SD卡",
        "已记录打卡点: ",
        "打卡点",
        "开始记录: ",
        "启动记录失败",
        "文件名不能为空！",
        "同名文件已存在！",
        "重命名成功",
        "重命名失败",
        "视角锁定居中",
        "自由平移浏览",
        "GPS 开启",
        "GPS 关闭",
        "GPS 模块配置已保存",

        // Dialog
        "插入途经标注点 (POI)",
        "开启记录: 输入文件名",
        "重命名 KML 文件",
        "快捷: 1直行 2左转 3右转 4下坡",
        "      5打卡 6营地 7水源 8危险",
        "[Enter] 保存 | [ESC] 退出 | [Tab] 输入法",
        "文件名后将自动拼接时间戳:",
        "例如: 名称_YYYYMMDD_HHMMSS",
        "[Enter] 开始 | [ESC] 退出 | [Tab] 输入法",
        "输入新文件名 (无需填写 .kml)",
        "按回车确认修改并刷新",
        "[Enter] 重命名 | [ESC] 退出 | [Tab] 输入法",

        // Custom GPS
        "自定义 GPS 模块引脚",
        "RX 引脚 (接收):",
        "TX 引脚 (发送):",
        "波特率 (bps):",

        // Debug Panel
        "GPS: 未开启",
        "按 's' 开启 GPS",
        "无数据! 请检查模块设置(按c)",
        "检测到模块 ",
        "按 'y' 键自动切换",
        "搜星超时! 请到开阔地重新搜星",

        // GPS Detected Dialog
        "检测到备选 GPS 模块",
        "数据流正常，是否切换?",
        "[Y/Enter] 确认切换",
        "[N/ESC] 忽略保持",
    }
};
