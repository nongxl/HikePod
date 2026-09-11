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
        "> GPS 息屏间隔 : ",
        "> 关键点显示 : ",
        "> GPS 模块设置 : ",
        "> 系统语言 : ",
        "English",
        "简体中文",
        "电池消耗曲线 :",

        // GPS 模块窗口
        "GPS 硬件模块设置",
        "Cap LoRa-1262 (RX:15 TX:13)",
        "Unit GPS v1.1 (RX:1 TX:0)",
        "自定义引脚...",

        // KML List
        "选择 KML 路线文件",
        "[Enter] 加载 | [r] 重命名 | [BS] 退出",

        // Help
        "HikePod 按键帮助",
        "显示帮助",
        "系统设置",
        "2D/3D 视角",
        "锁定/居中",
        "记录航迹",
        "添加途经点",
        "GPS 详情",
        "挂载 U 盘",
        "WiFi 传文件",
        "地图缩放",
        "3D 垂直比例",
        "调试信息",
        "平移地图",
        "开关 GPS",

        // USB
        "USB 存储传输 (MSC)",
        "SD 卡容量: ",
        "状态: 已挂载为电脑 U 盘",
        "1. 拖放 KML 文件至 /HikePod",
        "2. 拔出前请先在电脑安全弹出!",
        "按 'u' 或 `(ESC) 退出模式",

        // WiFi
        "WiFi KML 管理器",
        "状态: 服务运行中",
        "请连接热点: ",
        "浏览器访问: ",
        "按 'w' 键关闭并返回主界面",

        // Alert
        "GPS 尚未定位 !",
        "KML 路线点数超限!",
        "部分冗余点已跳过。",
        "尚未开启航迹记录 !",
        "请先按 't' 键开始。",

        // Toast
        "SD 卡未就绪",
        "航迹已保存",
        "已标记: ",
        "途经点",
        "开始记录: ",
        "航迹记录启动失败",
        "文件名不能为空",
        "同名文件已存在",
        "重命名成功",
        "重命名失败",
        "定位跟随已开启",
        "自由浏览模式",
        "GPS: 开启",
        "GPS: 关闭",
        "已应用模块: ",

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
    }
};
