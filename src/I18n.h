#pragma once

#include <Arduino.h>
#include <Preferences.h>

enum Language {
    LANG_EN = 0, // 默认英文
    LANG_ZH = 1  // 简体中文
};

enum TextId {
    // 设置菜单
    T_SETTINGS_TITLE,
    T_SETTINGS_SELECT_KML,
    T_SETTINGS_BRIGHTNESS,
    T_SETTINGS_TIMEOUT,
    T_SETTINGS_TIMEOUT_NEVER,
    T_SETTINGS_GPS_INT,
    T_SETTINGS_SCROFF_GPS,
    T_SETTINGS_SHOW_POIS,
    T_SETTINGS_GPS_MODULE,
    T_SETTINGS_LANGUAGE,
    T_LANG_NAME_EN,
    T_LANG_NAME_ZH,
    T_SETTINGS_BATTERY_TITLE,

    // GPS 模块选择窗口
    T_GPS_MOD_TITLE,
    T_GPS_MOD_CAP_LORA,
    T_GPS_MOD_UNIT_V11,
    T_GPS_MOD_CUSTOM,

    // KML 文件列表
    T_FILE_SELECT_TITLE,
    T_FILE_SELECT_HINT,

    // 帮助菜单
    T_HELP_TITLE,
    T_HELP_THIS,
    T_HELP_SETTINGS,
    T_HELP_VIEW,
    T_HELP_LOCK,
    T_HELP_TRACK,
    T_HELP_INSERT_POI,
    T_HELP_GPS_INFO,
    T_HELP_USB_DISK,
    T_HELP_WIFI,
    T_HELP_ZOOM,
    T_HELP_VERT_SCALE,
    T_HELP_DEBUG,
    T_HELP_PAN,
    T_HELP_TOGGLE_GPS,

    // USB 模式
    T_USB_TITLE,
    T_USB_SD_CARD,
    T_USB_STATUS_MOUNTED,
    T_USB_HINT_1,
    T_USB_HINT_2,
    T_USB_EXIT_HINT,

    // WiFi 模式
    T_WIFI_TITLE,
    T_WIFI_STATUS_RUNNING,
    T_WIFI_CONNECT_TO,
    T_WIFI_BROWSER_VISIT,
    T_WIFI_HINT_EXIT,

    // 确认弹窗
    T_ALERT_GPS_NO_FIX_1,
    T_ALERT_KML_LIMIT_1,
    T_ALERT_KML_LIMIT_2,
    T_ALERT_NOT_TRACKING_1,
    T_ALERT_NOT_TRACKING_2,

    // Toast 状态提示
    T_TOAST_SD_NOT_READY,
    T_TOAST_TRACK_SAVED,
    T_TOAST_MARKED_PREFIX,
    T_TOAST_DEFAULT_POI,
    T_TOAST_STARTED_PREFIX,
    T_TOAST_START_FAILED,
    T_TOAST_NAME_EMPTY,
    T_TOAST_FILE_EXISTS,
    T_TOAST_RENAME_OK,
    T_TOAST_RENAME_FAILED,
    T_TOAST_FOLLOW_ON,
    T_TOAST_FOLLOW_OFF,
    T_TOAST_GPS_ON,
    T_TOAST_GPS_OFF,
    T_TOAST_GPS_MOD_SAVED,

    // 输入对话框
    T_DIALOG_POI_TITLE,
    T_DIALOG_TRACK_TITLE,
    T_DIALOG_RENAME_TITLE,
    T_DIALOG_POI_QUICK_1,
    T_DIALOG_POI_QUICK_2,
    T_DIALOG_SAVE_HINT,
    T_DIALOG_TRACK_HINT_1,
    T_DIALOG_TRACK_HINT_2,
    T_DIALOG_START_HINT,
    T_DIALOG_RENAME_HINT_1,
    T_DIALOG_RENAME_HINT_2,
    T_DIALOG_RENAME_HINT_3,

    // 自定义 GPS 模块
    T_CUSTOM_GPS_TITLE,
    T_CUSTOM_GPS_RX,
    T_CUSTOM_GPS_TX,
    T_CUSTOM_GPS_BAUD,

    T_COUNT
};

class I18n {
public:
    static I18n& getInstance() {
        static I18n instance;
        return instance;
    }

    void begin() {
        Preferences prefs;
        if (prefs.begin("hikepod", true)) {
            uint8_t lang = prefs.getUChar("lang", (uint8_t)LANG_EN);
            _currentLang = (lang <= LANG_ZH) ? (Language)lang : LANG_EN;
            prefs.end();
        } else {
            _currentLang = LANG_EN;
        }
    }

    void setLanguage(Language lang) {
        _currentLang = lang;
        Preferences prefs;
        if (prefs.begin("hikepod", false)) {
            prefs.putUChar("lang", (uint8_t)_currentLang);
            prefs.end();
        }
    }

    Language getLanguage() const {
        return _currentLang;
    }

    bool isChinese() const {
        return _currentLang == LANG_ZH;
    }

    static const char* t(TextId id) {
        return getInstance().getText(id);
    }

    const char* getText(TextId id) const {
        if (id >= T_COUNT) return "";
        return _strings[_currentLang][id];
    }

private:
    I18n() : _currentLang(LANG_EN) {}
    Language _currentLang;

    static const char* const _strings[2][T_COUNT];
};
