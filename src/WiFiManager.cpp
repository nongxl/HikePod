#include "WiFiManager.h"
#include <SD.h>

WiFiManager::WiFiManager() : _server(80), _isRunning(false), _statusMsg("WiFi Off") {
}

void WiFiManager::begin(std::function<void()> preStartCallback) {
    if (_isRunning) return;

    // 如果有回调，先执行（通常用于释放内存）
    if (preStartCallback) {
        preStartCallback();
    }

    delay(100);

    String ssid = getSSID();
    Serial.println("Starting AP: " + ssid);

    WiFi.mode(WIFI_AP);
    delay(100);

    if (WiFi.softAP(ssid.c_str())) {
        IPAddress IP = WiFi.softAPIP();
        Serial.print("AP IP address: ");
        Serial.println(IP);

        // 设置路由
        _server.on("/", HTTP_GET, std::bind(&WiFiManager::handleFileList, this));
        _server.on("/upload", HTTP_POST, [this]() {
            _server.sendHeader("Location", "/");
            _server.send(303);
        }, std::bind(&WiFiManager::handleFileUpload, this));
        _server.on("/delete", HTTP_GET, std::bind(&WiFiManager::handleFileDelete, this));

        _server.begin();
        _isRunning = true;
        _statusMsg = "Server Run: http://" + IP.toString();
        Serial.println("HTTP Server started");
    } else {
        Serial.println("WiFi softAP failed");
        _statusMsg = "WiFi Error!";
    }
}

void WiFiManager::stop() {
    if (!_isRunning) return;

    _server.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(200); // 允许底层栈开始回收堆资源
    _isRunning = false;
    _statusMsg = "WiFi Off";
    Serial.println("HTTP Server stopped");
}

void WiFiManager::handle() {
    if (_isRunning) {
        _server.handleClient();
    }
}

String WiFiManager::getSSID() const {
    String deviceId = String((uint32_t)ESP.getEfuseMac(), HEX).substring(0, 4);
    return "HikePod_" + deviceId;
}

void WiFiManager::handleFileList() {
    String html = "<html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'><title>HikePod File Manager</title>";
    html += "<style>body{font-family:sans-serif;margin:20px;background:#f0f2f5}h1{color:#1a73e8}.card{background:white;padding:20px;border-radius:8px;box-shadow:0 2px 4px rgba(0,0,0,0.1)}table{width:100%;border-collapse:collapse;margin-top:20px}th,td{padding:12px;text-align:left;border-bottom:1px solid #ddd}th{background:#f8f9fa}tr:hover{background:#f1f3f4}.btn{padding:6px 12px;border-radius:4px;text-decoration:none;color:white;font-size:14px}.btn-del{background:#d93025}.btn-up{background:#1a73e8;border:none;padding:10px 20px;cursor:pointer}input[type=file]{margin-bottom:10px}</style></head><body>";
    html += "<h1>HikePod KML 管理器</h1><div class='card'>";
    html += "<h3>上传新 KML 文件</h3><form method='POST' action='/upload' enctype='multipart/form-data'><input type='file' name='upload'><br><input type='submit' value='上传' class='btn btn-up'></form>";
    html += "<table><thead><tr><th>文件名</th><th>大小</th><th>操作</th></tr></thead><tbody>";

    File root = SD.open("/HikePod");
    if (root && root.isDirectory()) {
        File file = root.openNextFile();
        while (file) {
            String fileName = String(file.name());
            if (fileName.endsWith(".kml") || fileName.endsWith(".KML")) {
                html += "<tr><td>" + fileName + "</td><td>" + String(file.size() / 1024) + " KB</td>";
                html += "<td><a href='/delete?filename=" + fileName + "' class='btn btn-del' onclick='return confirm(\"确定删除?\")'>删除</a></td></tr>";
            }
            file = root.openNextFile();
        }
        root.close();
    }
    
    html += "</tbody></table></div></body></html>";
    _server.send(200, "text/html", html);
}

void WiFiManager::handleFileUpload() {
    HTTPUpload& upload = _server.upload();
    static File uploadFile;

    if (upload.status == UPLOAD_FILE_START) {
        String filename = "/HikePod/" + upload.filename;
        Serial.println("Upload Start: " + filename);
        if (SD.exists(filename)) {
            SD.remove(filename);
        }
        uploadFile = SD.open(filename, FILE_WRITE);
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (uploadFile) {
            uploadFile.write(upload.buf, upload.currentSize);
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (uploadFile) {
            uploadFile.close();
            Serial.println("Upload End: " + String(upload.totalSize));
        }
    }
}

void WiFiManager::handleFileDelete() {
    String filename = _server.arg("filename");
    if (filename != "") {
        String path = "/HikePod/" + filename;
        if (SD.exists(path)) {
            SD.remove(path);
            Serial.println("Deleted: " + path);
        }
    }
    _server.sendHeader("Location", "/");
    _server.send(303);
}

String WiFiManager::getContentType(String filename) {
    if (filename.endsWith(".html")) return "text/html";
    else if (filename.endsWith(".css")) return "text/css";
    else if (filename.endsWith(".js")) return "application/javascript";
    else if (filename.endsWith(".kml")) return "application/vnd.google-earth.kml+xml";
    return "text/plain";
}
