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
#include "lidar_http_server.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <iomanip>
#include <ctime>

// SimpleHttpServer implementation
SimpleHttpServer::SimpleHttpServer(int port) : port_(port), server_fd_(-1), running_(false) {}

SimpleHttpServer::~SimpleHttpServer() {
    stop();
}

void SimpleHttpServer::addRoute(const std::string& method, const std::string& path, RequestHandler handler) {
    std::string key = method + " " + path;
    routes_[key] = handler;
}

bool SimpleHttpServer::start() {
    // Create socket
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ == -1) {
        std::cerr << "Failed to create socket" << std::endl;
        return false;
    }
    
    // Set socket options
    int opt = 1;
    if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt))) {
        std::cerr << "Failed to set socket options" << std::endl;
        close(server_fd_);
        return false;
    }
    
    // Bind socket
    struct sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port_);
    
    if (bind(server_fd_, (struct sockaddr*)&address, sizeof(address)) < 0) {
        std::cerr << "Failed to bind socket to port " << port_ << std::endl;
        close(server_fd_);
        return false;
    }
    
    // Listen
    if (listen(server_fd_, 10) < 0) {
        std::cerr << "Failed to listen on socket" << std::endl;
        close(server_fd_);
        return false;
    }
    
    running_ = true;
    server_thread_ = std::thread(&SimpleHttpServer::serverLoop, this);
    
    std::cout << "HTTP Server started on port " << port_ << std::endl;
    return true;
}

void SimpleHttpServer::stop() {
    if (running_) {
        running_ = false;
        if (server_fd_ >= 0) {
            close(server_fd_);
            server_fd_ = -1;
        }
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
        std::cout << "HTTP Server stopped" << std::endl;
    }
}

void SimpleHttpServer::serverLoop() {
    while (running_) {
        struct sockaddr_in client_addr;
        socklen_t client_len = sizeof(client_addr);
        
        int client_fd = accept(server_fd_, (struct sockaddr*)&client_addr, &client_len);
        if (client_fd < 0) {
            if (running_) {
                std::cerr << "Failed to accept client connection" << std::endl;
            }
            continue;
        }
        
        // Handle client in separate thread (for simplicity, we'll handle synchronously)
        handleClient(client_fd);
        close(client_fd);
    }
}

void SimpleHttpServer::handleClient(int client_fd) {
    char buffer[4096];
    ssize_t bytes_read = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
    
    if (bytes_read <= 0) {
        return;
    }
    
    buffer[bytes_read] = '\0';
    std::string request_str(buffer);
    
    HttpRequest request = parseRequest(request_str);
    HttpResponse response;
    
    // Find matching route
    std::string route_key = request.method + " " + request.path;
    auto it = routes_.find(route_key);
    
    if (it != routes_.end()) {
        response = it->second(request);
    } else {
        response.status_code = 404;
        response.body = "{\"error\": \"Not Found\"}";
    }
    
    std::string response_str = buildResponse(response);
    send(client_fd, response_str.c_str(), response_str.length(), 0);
}

HttpRequest SimpleHttpServer::parseRequest(const std::string& request_str) {
    HttpRequest request;
    std::istringstream iss(request_str);
    std::string line;
    
    // Parse request line
    if (std::getline(iss, line)) {
        std::istringstream line_iss(line);
        std::string path_query;
        line_iss >> request.method >> path_query;
        
        // Split path and query parameters
        size_t query_pos = path_query.find('?');
        if (query_pos != std::string::npos) {
            request.path = path_query.substr(0, query_pos);
            std::string query = path_query.substr(query_pos + 1);
            
            // Parse query parameters (simple implementation)
            std::istringstream query_iss(query);
            std::string param;
            while (std::getline(query_iss, param, '&')) {
                size_t eq_pos = param.find('=');
                if (eq_pos != std::string::npos) {
                    std::string key = urlDecode(param.substr(0, eq_pos));
                    std::string value = urlDecode(param.substr(eq_pos + 1));
                    request.query_params[key] = value;
                }
            }
        } else {
            request.path = path_query;
        }
    }
    
    // Parse headers
    while (std::getline(iss, line) && line != "\r") {
        size_t colon_pos = line.find(':');
        if (colon_pos != std::string::npos) {
            std::string key = line.substr(0, colon_pos);
            std::string value = line.substr(colon_pos + 2); // Skip ": "
            if (!value.empty() && value.back() == '\r') {
                value.pop_back();
            }
            request.headers[key] = value;
        }
    }
    
    // Parse body (if any)
    std::string body_line;
    while (std::getline(iss, body_line)) {
        request.body += body_line + "\n";
    }
    
    return request;
}

std::string SimpleHttpServer::buildResponse(const HttpResponse& response) {
    std::ostringstream oss;
    oss << "HTTP/1.1 " << response.status_code << " ";
    
    switch (response.status_code) {
        case 200: oss << "OK"; break;
        case 404: oss << "Not Found"; break;
        case 500: oss << "Internal Server Error"; break;
        default: oss << "Unknown"; break;
    }
    
    oss << "\r\n";
    oss << "Content-Type: " << response.content_type << "\r\n";
    oss << "Content-Length: " << response.body.length() << "\r\n";
    
    // Add custom headers
    for (const auto& header : response.headers) {
        oss << header.first << ": " << header.second << "\r\n";
    }
    
    oss << "\r\n" << response.body;
    return oss.str();
}

std::string SimpleHttpServer::urlDecode(const std::string& str) {
    std::string result;
    for (size_t i = 0; i < str.length(); ++i) {
        if (str[i] == '%' && i + 2 < str.length()) {
            int hex_value;
            std::istringstream hex_stream(str.substr(i + 1, 2));
            if (hex_stream >> std::hex >> hex_value) {
                result += static_cast<char>(hex_value);
                i += 2;
            } else {
                result += str[i];
            }
        } else if (str[i] == '+') {
            result += ' ';
        } else {
            result += str[i];
        }
    }
    return result;
}

// LidarHttpServer implementation
LidarHttpServer::LidarHttpServer(const std::string& port, int baudrate, int http_port)
    : lidar_(std::make_unique<LidarReader>(port, baudrate)),
      server_(std::make_unique<SimpleHttpServer>(http_port)),
      scanning_(false) {
    
    // Add routes
    server_->addRoute("GET", "/", [this](const HttpRequest& req) { return handleIndex(req); });
    server_->addRoute("POST", "/api/start", [this](const HttpRequest& req) { return handleStart(req); });
    server_->addRoute("POST", "/api/stop", [this](const HttpRequest& req) { return handleStop(req); });
    server_->addRoute("GET", "/api/status", [this](const HttpRequest& req) { return handleStatus(req); });
    server_->addRoute("GET", "/api/scan", [this](const HttpRequest& req) { return handleScan(req); });
    server_->addRoute("POST", "/api/save", [this](const HttpRequest& req) { return handleSave(req); });
    server_->addRoute("POST", "/api/clear", [this](const HttpRequest& req) { return handleClear(req); });
    server_->addRoute("GET", "/api/stats", [this](const HttpRequest& req) { return handleStats(req); });
    server_->addRoute("GET", "/api/debug_angles", [this](const HttpRequest& req) { return handleDebugAngles(req); });
}

LidarHttpServer::~LidarHttpServer() {
    stop();
}

bool LidarHttpServer::start() {
    return server_->start();
}

void LidarHttpServer::stop() {
    if (scanning_) {
        scanning_ = false;
        if (scan_thread_.joinable()) {
            scan_thread_.join();
        }
        lidar_->disconnect();
    }
    server_->stop();
}

void LidarHttpServer::scanWorker() {
    std::cout << "LiDAR scan worker started" << std::endl;
    
    // Start the LiDAR's continuous scanning
    if (!lidar_->startContinuousScanning()) {
        std::cerr << "Failed to start LiDAR continuous scanning" << std::endl;
        scanning_ = false;
        return;
    }
    
    while (scanning_) {
        // Get the latest complete scan from the LiDAR
        std::vector<LidarPoint> complete_scan = lidar_->getLatestCompleteScan();
        
        if (!complete_scan.empty()) {
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_scan_ = complete_scan;
            all_scans_.push_back(complete_scan);
            
            // Keep only last 100 scans (complete scans are larger)
            if (all_scans_.size() > 100) {
                all_scans_.erase(all_scans_.begin());
            }
            
            std::cout << "Updated with complete scan of " << complete_scan.size() << " points" << std::endl;
        }
        
        // Check for complete scans very frequently for maximum responsiveness
        std::this_thread::sleep_for(std::chrono::nanoseconds(500));
    }
    
    // Stop the LiDAR's continuous scanning
    lidar_->stopContinuousScanning();
    
    std::cout << "LiDAR scan worker stopped" << std::endl;
}

HttpResponse LidarHttpServer::handleIndex(const HttpRequest& /* request */) {
    HttpResponse response;
    response.content_type = "text/html";
    
    // Read the HTML template file
    std::ifstream file("templates/index.html");
    if (file.is_open()) {
        std::string line;
        response.body = "";
        while (std::getline(file, line)) {
            response.body += line + "\n";
        }
        file.close();
    } else {
        // Fallback if template file not found
        response.body = R"(
<!DOCTYPE html>
<html>
<head>
    <title>C++ LiDAR Server</title>
</head>
<body>
    <h1>C++ LiDAR HTTP Server</h1>
    <p>LiDAR REST API is running!</p>
    <p><strong>Error:</strong> Could not load templates/index.html</p>
    <h2>Available Endpoints:</h2>
    <ul>
        <li>GET /api/status - Get LiDAR status</li>
        <li>POST /api/start - Start LiDAR scanning</li>
        <li>POST /api/stop - Stop LiDAR scanning</li>
        <li>GET /api/scan - Get latest scan data</li>
        <li>GET /api/stats - Get statistics</li>
        <li>GET /api/debug_angles - Debug angle ranges</li>
        <li>POST /api/save - Save scan data</li>
        <li>POST /api/clear - Clear accumulated data</li>
    </ul>
</body>
</html>
        )";
    }
    
    addCorsHeaders(response);
    return response;
}

HttpResponse LidarHttpServer::handleStart(const HttpRequest& request) {
    HttpResponse response;
    addCorsHeaders(response);
    
    if (scanning_) {
        response.body = "{\"status\": \"already_running\"}";
        return response;
    }
    
    if (!lidar_->connect()) {
        response.status_code = 500;
        response.body = "{\"error\": \"Failed to connect to LiDAR\"}";
        return response;
    }
    
    scanning_ = true;
    scan_thread_ = std::thread(&LidarHttpServer::scanWorker, this);
    
    response.body = "{\"status\": \"started\"}";
    return response;
}

HttpResponse LidarHttpServer::handleStop(const HttpRequest& request) {
    HttpResponse response;
    addCorsHeaders(response);
    
    if (!scanning_) {
        response.body = "{\"status\": \"not_running\"}";
        return response;
    }
    
    scanning_ = false;
    if (scan_thread_.joinable()) {
        scan_thread_.join();
    }
    lidar_->disconnect();
    
    response.body = "{\"status\": \"stopped\"}";
    return response;
}

HttpResponse LidarHttpServer::handleStatus(const HttpRequest& request) {
    HttpResponse response;
    addCorsHeaders(response);
    
    std::ostringstream oss;
    oss << "{\"running\": " << (scanning_ ? "true" : "false") << "}";
    response.body = oss.str();
    
    return response;
}

HttpResponse LidarHttpServer::handleScan(const HttpRequest& request) {
    HttpResponse response;
    addCorsHeaders(response);
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    std::ostringstream oss;
    oss << "{\"points\": " << pointsToJson(latest_scan_) 
        << ", \"count\": " << latest_scan_.size() << "}";
    response.body = oss.str();
    
    return response;
}

HttpResponse LidarHttpServer::handleSave(const HttpRequest& request) {
    HttpResponse response;
    addCorsHeaders(response);
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (all_scans_.empty()) {
        response.body = "{\"status\": \"no_data\"}";
        return response;
    }
    
    std::string filename = "ld19_scan_cpp_" + getCurrentTimestamp() + ".json";
    std::ofstream file(filename);
    
    if (!file.is_open()) {
        response.status_code = 500;
        response.body = "{\"error\": \"Failed to create file\"}";
        return response;
    }
    
    file << "[";
    for (size_t i = 0; i < all_scans_.size(); ++i) {
        if (i > 0) file << ",";
        file << pointsToJson(all_scans_[i]);
    }
    file << "]";
    file.close();
    
    std::ostringstream oss;
    oss << "{\"status\": \"saved\", \"filename\": \"" << filename 
        << "\", \"scans\": " << all_scans_.size() << "}";
    response.body = oss.str();
    
    return response;
}

HttpResponse LidarHttpServer::handleClear(const HttpRequest& request) {
    HttpResponse response;
    addCorsHeaders(response);
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    all_scans_.clear();
    latest_scan_.clear();
    
    response.body = "{\"status\": \"cleared\"}";
    return response;
}

HttpResponse LidarHttpServer::handleStats(const HttpRequest& request) {
    HttpResponse response;
    addCorsHeaders(response);
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    std::ostringstream oss;
    oss << "{"
        << "\"total_scans\": " << all_scans_.size() << ","
        << "\"latest_scan_points\": " << latest_scan_.size() << ","
        << "\"is_running\": " << (scanning_ ? "true" : "false") << ","
        << "\"connection_status\": \"" << (lidar_->isConnected() ? "connected" : "disconnected") << "\"";
    
    if (!latest_scan_.empty()) {
        std::vector<uint16_t> distances;
        for (const auto& point : latest_scan_) {
            if (point.distance() > 0) {
                distances.push_back(point.distance());
            }
        }
        
        if (!distances.empty()) {
            auto minmax = std::minmax_element(distances.begin(), distances.end());
            uint32_t sum = 0;
            for (auto d : distances) sum += d;
            
            oss << ","
                << "\"min_distance\": " << *minmax.first << ","
                << "\"max_distance\": " << *minmax.second << ","
                << "\"avg_distance\": " << (sum / distances.size()) << ","
                << "\"valid_points\": " << distances.size();
        }
    }
    
    oss << "}";
    response.body = oss.str();
    
    return response;
}

HttpResponse LidarHttpServer::handleDebugAngles(const HttpRequest& request) {
    HttpResponse response;
    addCorsHeaders(response);
    
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    if (latest_scan_.empty()) {
        response.body = "{\"error\": \"No scan data available\"}";
        return response;
    }
    
    // Define angle ranges
    std::vector<std::pair<std::string, std::pair<float, float>>> ranges = {
        {"85-95", {85.0f, 95.0f}},
        {"175-185", {175.0f, 185.0f}},
        {"265-275", {265.0f, 275.0f}},
        {"355-5", {355.0f, 365.0f}}  // Special case for wraparound
    };
    
    std::ostringstream oss;
    oss << "{"
        << "\"total_points\": " << latest_scan_.size() << ","
        << "\"angle_ranges\": {";
    
    bool first_range = true;
    for (const auto& range : ranges) {
        if (!first_range) oss << ",";
        first_range = false;
        
        std::vector<LidarPoint> points_in_range;
        
        for (const auto& point : latest_scan_) {
            float angle = point.angle();
            
            if (range.first == "355-5") {
                // Handle wraparound
                if (angle >= 355.0f || angle <= 5.0f) {
                    points_in_range.push_back(point);
                }
            } else {
                if (angle >= range.second.first && angle <= range.second.second) {
                    points_in_range.push_back(point);
                }
            }
        }
        
        oss << "\"" << range.first << "\": {"
            << "\"count\": " << points_in_range.size();
        
        if (!points_in_range.empty()) {
            std::vector<uint16_t> distances;
            for (const auto& p : points_in_range) {
                distances.push_back(p.distance());
            }
            
            auto minmax = std::minmax_element(distances.begin(), distances.end());
            uint32_t sum = 0;
            for (auto d : distances) sum += d;
            
            oss << ","
                << "\"min_distance\": " << *minmax.first << ","
                << "\"max_distance\": " << *minmax.second << ","
                << "\"avg_distance\": " << (sum / distances.size()) << ","
                << "\"points\": [";
            
            // Show first 10 points
            size_t show_count = std::min(size_t(10), points_in_range.size());
            for (size_t i = 0; i < show_count; ++i) {
                if (i > 0) oss << ",";
                const auto& p = points_in_range[i];
                oss << "{"
                    << "\"distance\": " << p.distance() << ","
                    << "\"intensity\": " << (int)p.intensity() << ","
                    << "\"angle\": " << p.angle()
                    << "}";
            }
            oss << "]";
        }
        
        oss << "}";
    }
    
    oss << "},"
        << "\"all_angles\": [";
    
    for (size_t i = 0; i < latest_scan_.size(); ++i) {
        if (i > 0) oss << ",";
        oss << latest_scan_[i].angle();
    }
    
    oss << "]}";
    response.body = oss.str();
    
    return response;
}

std::string LidarHttpServer::pointsToJson(const std::vector<LidarPoint>& points) {
    std::ostringstream oss;
    oss << "[";
    
    for (size_t i = 0; i < points.size(); ++i) {
        if (i > 0) oss << ",";
        const auto& point = points[i];
        oss << "{"
            << "\"distance\": " << point.distance() << ","
            << "\"intensity\": " << (int)point.intensity() << ","
            << "\"angle\": " << point.angle()
            << "}";
    }
    
    oss << "]";
    return oss.str();
}

std::string LidarHttpServer::getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
    return oss.str();
}

void LidarHttpServer::addCorsHeaders(HttpResponse& response) {
    response.headers["Access-Control-Allow-Origin"] = "*";
    response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS";
    response.headers["Access-Control-Allow-Headers"] = "Content-Type";
}
