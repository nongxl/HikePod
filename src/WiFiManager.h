#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <WiFi.h>
#include <WebServer.h>
#include <FS.h>
#include <functional>

class WiFiManager {
public:
    WiFiManager();
    
    // 初始化 WiFiManager，传入预启动回调（用于释放内存）
    void begin(std::function<void()> preStartCallback = nullptr);
    
    // 停止 WiFi 和 HTTP 服务
    void stop();
    
    // 在 loop 中调用，处理客户端请求
    void handle();
    
    // 检查服务是否正在运行
    bool isRunning() const { return _isRunning; }
    
    // 获取状态消息
    String getStatusMsg() const { return _statusMsg; }
    
    // 获取设备 SSID
    String getSSID() const;

private:
    WebServer _server;
    bool _isRunning;
    String _statusMsg;
    
    // 路由处理函数
    void handleFileList();
    void handleFileUpload();
    void handleFileDelete();
    
    // 辅助函数
    String getContentType(String filename);
};

#endif // WIFI_MANAGER_H
