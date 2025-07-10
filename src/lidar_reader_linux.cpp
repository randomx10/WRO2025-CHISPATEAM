/*
    Copyright (C) 2025 Alexis Martinez

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
#include "lidar_reader_linux.h"
#include <algorithm>

// LidarPoint implementation
LidarPoint::LidarPoint(uint16_t distance, uint8_t intensity, float angle)
    : _distance(distance), _intensity(intensity), _angle(angle) {}

std::string LidarPoint::toString() const {
    return "(distance=" + std::to_string(_distance) + 
           ", intensity=" + std::to_string(_intensity) + 
           ", angle=" + std::to_string(_angle) + ")";
}

// LidarReader implementation
LidarReader::LidarReader(const std::string& port, int baudrate)
    : port(port), baudrate(baudrate), fd(-1), is_scanning(false), last_speed(0) {}

LidarReader::~LidarReader() {
    stopContinuousScanning();
    disconnect();
}

bool LidarReader::connect() {
    // Open serial port
    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "Error opening " << port << std::endl;
        return false;
    }
    
    // Configure serial port
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error getting tty attributes" << std::endl;
        close(fd);
        fd = -1;
        return false;
    }
    
    // Set baud rate
    speed_t speed;
    switch (baudrate) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        case 921600: speed = B921600; break;
        default:
            std::cerr << "Unsupported baud rate: " << baudrate << std::endl;
            close(fd);
            fd = -1;
            return false;
    }
    
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    
    // Configure port settings
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 1;            // read blocks until at least 1 byte available
    tty.c_cc[VTIME] = 1;            // 0.1 seconds read timeout (reduced from 0.5)
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
    
    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error setting tty attributes" << std::endl;
        close(fd);
        fd = -1;
        return false;
    }
    
    std::cout << "Connected to LiDAR on " << port << " at " << baudrate << " baud" << std::endl;
    return true;
}

void LidarReader::disconnect() {
    if (fd >= 0) {
        close(fd);
        fd = -1;
        std::cout << "Disconnected from LiDAR" << std::endl;
    }
}

bool LidarReader::findHeader() {
    uint8_t byte;
    while (true) {
        ssize_t result = read(fd, &byte, 1);
        if (result <= 0) {
            return false;
        }
        
        if (byte == 0x54) {  // 'T'
            result = read(fd, &byte, 1);
            if (result <= 0) {
                return false;
            }
            if (byte == 0x2C) {  // ','
                return true;
            }
        }
    }
}

ssize_t LidarReader::readBytes(uint8_t* buffer, size_t length) {
    size_t totalRead = 0;
    while (totalRead < length) {
        ssize_t result = read(fd, buffer + totalRead, length - totalRead);
        if (result <= 0) {
            break;
        }
        totalRead += result;
    }
    return totalRead;
}

std::vector<LidarPoint> LidarReader::getPoints() {
    std::vector<LidarPoint> points;
    
    if (fd < 0) {
        std::cerr << "LiDAR not connected" << std::endl;
        return points;
    }
    
    if (!findHeader()) {
        std::cerr << "lidar_reader.getPoints : error, no header found in RX for the LiDAR LD19" << std::endl;
        return points;
    }
    
    // The previous instruction (findHeader) found the header
    // Now the stream is aligned
    uint8_t buffer[45];
    ssize_t nbrBytesReceived = readBytes(buffer, 45);
    
    if (nbrBytesReceived != 45) {
        std::cerr << "lidar_reader.getPoints : error, wrong number of bytes received (" 
                  << nbrBytesReceived << ")" << std::endl;
        return points;
    }
    
    uint16_t speed = get2BytesLsbMsb(buffer, 0);
    uint16_t startAngle = get2BytesLsbMsb(buffer, 2);
    
    // Extract 12 measurement points
    LidarPoint data[] = {
        LidarPoint(get2BytesLsbMsb(buffer, 4), buffer[6], 0),
        LidarPoint(get2BytesLsbMsb(buffer, 7), buffer[9], 0),
        LidarPoint(get2BytesLsbMsb(buffer, 10), buffer[12], 0),
        LidarPoint(get2BytesLsbMsb(buffer, 13), buffer[15], 0),
        LidarPoint(get2BytesLsbMsb(buffer, 16), buffer[18], 0),
        LidarPoint(get2BytesLsbMsb(buffer, 19), buffer[21], 0),
        LidarPoint(get2BytesLsbMsb(buffer, 22), buffer[24], 0),
        LidarPoint(get2BytesLsbMsb(buffer, 25), buffer[27], 0),
        LidarPoint(get2BytesLsbMsb(buffer, 28), buffer[30], 0),
        LidarPoint(get2BytesLsbMsb(buffer, 31), buffer[33], 0),
        LidarPoint(get2BytesLsbMsb(buffer, 34), buffer[36], 0),
        LidarPoint(get2BytesLsbMsb(buffer, 37), buffer[39], 0)
    };
    
    uint16_t endAngle = get2BytesLsbMsb(buffer, 40);
    uint16_t timestamp = get2BytesLsbMsb(buffer, 42);
    uint8_t crcCheck = buffer[44];
    
    if (calcCRC8FromBuffer(buffer, 44) == crcCheck) {
        uint16_t step = angleStep(startAngle, endAngle);
        for (unsigned int i = 0; i < 12; i++) {
            points.push_back(
                LidarPoint(
                    data[i].distance(),
                    data[i].intensity(),
                    angleFromStep(startAngle, step, i) / 100.0f  // Convert to degrees
                )
            );
        }
    } else {
        std::cerr << "CRC check failed" << std::endl;
    }
    
    return points;
}

uint8_t LidarReader::calcCRC8FromBuffer(uint8_t* p, uint8_t lenWithoutCRCCheckValue) {
    uint8_t crc = 0xD8;  // pre-calculated header and verlen values
    for (uint16_t i = 0; i < lenWithoutCRCCheckValue; i++) {
        crc = crcTable[(crc ^ *p++) & 0xff];
    }
    return crc;
}

uint16_t LidarReader::get2BytesLsbMsb(uint8_t buffer[], int index) {
    return (buffer[index + 1] << 8) | buffer[index];
}

uint16_t LidarReader::angleStep(uint16_t startAngle, uint16_t endAngle, unsigned int lenMinusOne) {
    if (startAngle <= endAngle) {
        return (endAngle - startAngle) / lenMinusOne;
    } else {
        return (36000 + endAngle - startAngle) / lenMinusOne;
    }
}

uint16_t LidarReader::angleFromStep(uint16_t startAngle, uint16_t step, unsigned int indice) {
    return (startAngle + (step * indice)) % 36000;
}

bool LidarReader::startContinuousScanning() {
    if (is_scanning) {
        return true;
    }
    
    if (!isConnected()) {
        if (!connect()) {
            return false;
        }
    }
    
    is_scanning = true;
    scan_thread = std::thread(&LidarReader::scanWorker, this);
    
    std::cout << "Started continuous LiDAR scanning" << std::endl;
    return true;
}

void LidarReader::stopContinuousScanning() {
    if (!is_scanning) {
        return;
    }
    
    is_scanning = false;
    
    if (scan_thread.joinable()) {
        scan_thread.join();
    }
    
    std::cout << "Stopped continuous LiDAR scanning" << std::endl;
}

std::vector<LidarPoint> LidarReader::getLatestCompleteScan() {
    std::lock_guard<std::mutex> lock(scan_mutex);
    return latest_complete_scan;
}

std::vector<LidarPoint> LidarReader::getCompleteScan() {
    // Read packets until we have a complete 360° scan
    temp_scan_data.clear();
    
    int max_packets = 100; // Prevent infinite loops
    int packet_count = 0;
    
    while (packet_count < max_packets) {
        std::vector<LidarPoint> packet = getPoints();
        if (!packet.empty()) {
            addPacketToScan(packet);
            if (assembleCompleteScan()) {
                std::vector<LidarPoint> complete_scan = temp_scan_data;
                temp_scan_data.clear();
                return complete_scan;
            }
        }
        packet_count++;
    }
    
    // Return whatever we have if we can't get a complete scan
    std::vector<LidarPoint> partial_scan = temp_scan_data;
    temp_scan_data.clear();
    return partial_scan;
}

void LidarReader::scanWorker() {
    std::cout << "LiDAR scan worker started" << std::endl;
    
    while (is_scanning) {
        std::vector<LidarPoint> packet = getPoints();
        if (!packet.empty()) {
            std::lock_guard<std::mutex> lock(scan_mutex);
            addPacketToScan(packet);
            if (assembleCompleteScan()) {
                latest_complete_scan = temp_scan_data;
                temp_scan_data.clear();
            }
        }
        
        // Minimal delay - prioritize speed over CPU usage
        std::this_thread::sleep_for(std::chrono::nanoseconds(100));
    }
    
    std::cout << "LiDAR scan worker stopped" << std::endl;
}

void LidarReader::addPacketToScan(const std::vector<LidarPoint>& points) {
    for (const auto& point : points) {
        temp_scan_data.push_back(point);
    }
}

bool LidarReader::assembleCompleteScan() {
    if (temp_scan_data.size() < 50) {  // Reduced from 100 for faster response
        return false; // Need more data
    }
    
    // Check for complete rotation by looking for angle wraparound
    float last_angle = 0.0f;
    bool found_wraparound = false;
    size_t wraparound_index = 0;
    
    for (size_t i = 0; i < temp_scan_data.size(); i++) {
        float current_angle = temp_scan_data[i].angle();
        
        // Detect wraparound: angle < 20° after angle > 340°
        if (current_angle < 20.0f && last_angle > 340.0f) {
            found_wraparound = true;
            wraparound_index = i;
            break;
        }
        
        last_angle = current_angle;
    }
    
    if (found_wraparound && wraparound_index > 200) {
        // We have a complete scan, trim to the wraparound point
        temp_scan_data.resize(wraparound_index);
        
        // Sort by angle for better visualization
        std::sort(temp_scan_data.begin(), temp_scan_data.end(),
                  [](const LidarPoint& a, const LidarPoint& b) {
                      return a.angle() < b.angle();
                  });
        
        return true;
    }
    
    // Prevent excessive accumulation
    if (temp_scan_data.size() > 2000) {
        temp_scan_data.erase(temp_scan_data.begin(), temp_scan_data.begin() + 500);
    }
    
    return false;
}
