/*
    Copyright (C) 2025  Alexis Martinez

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Affero General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Affero General Public License for more details.

    You should have received a copy of the GNU Affero General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#ifndef LIDAR_HTTP_SERVER_H
#define LIDAR_HTTP_SERVER_H

#include "lidar_reader_linux.h"
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <map>
#include <functional>
#include <sstream>
#include <chrono>

// Simple HTTP Request structure
struct HttpRequest {
    std::string method;
    std::string path;
    std::string body;
    std::map<std::string, std::string> headers;
    std::map<std::string, std::string> query_params;
};

// Simple HTTP Response structure
struct HttpResponse {
    int status_code = 200;
    std::string content_type = "application/json";
    std::string body;
    std::map<std::string, std::string> headers;
};

// Simple HTTP Server class
class SimpleHttpServer {
public:
    using RequestHandler = std::function<HttpResponse(const HttpRequest&)>;
    
    SimpleHttpServer(int port = 5000);
    ~SimpleHttpServer();
    
    void addRoute(const std::string& method, const std::string& path, RequestHandler handler);
    bool start();
    void stop();
    
private:
    int port_;
    int server_fd_;
    std::atomic<bool> running_;
    std::thread server_thread_;
    std::map<std::string, RequestHandler> routes_;
    
    void serverLoop();
    void handleClient(int client_fd);
    HttpRequest parseRequest(const std::string& request_str);
    std::string buildResponse(const HttpResponse& response);
    std::string urlDecode(const std::string& str);
};

// LiDAR HTTP Server class
class LidarHttpServer {
public:
    LidarHttpServer(const std::string& port = "/dev/ttyUSB0", int baudrate = 230400, int http_port = 5000);
    ~LidarHttpServer();
    
    bool start();
    void stop();
    
private:
    std::unique_ptr<LidarReader> lidar_;
    std::unique_ptr<SimpleHttpServer> server_;
    std::atomic<bool> scanning_;
    std::thread scan_thread_;
    std::mutex data_mutex_;
    
    // Data storage
    std::vector<LidarPoint> latest_scan_;
    std::vector<std::vector<LidarPoint>> all_scans_;
    
    // Scan worker
    void scanWorker();
    
    // HTTP endpoint handlers
    HttpResponse handleIndex(const HttpRequest& request);
    HttpResponse handleStart(const HttpRequest& request);
    HttpResponse handleStop(const HttpRequest& request);
    HttpResponse handleStatus(const HttpRequest& request);
    HttpResponse handleScan(const HttpRequest& request);
    HttpResponse handleSave(const HttpRequest& request);
    HttpResponse handleClear(const HttpRequest& request);
    HttpResponse handleStats(const HttpRequest& request);
    HttpResponse handleDebugAngles(const HttpRequest& request);
    
    // Utility functions
    std::string pointsToJson(const std::vector<LidarPoint>& points);
    std::string getCurrentTimestamp();
    void addCorsHeaders(HttpResponse& response);
};

#endif // LIDAR_HTTP_SERVER_H
