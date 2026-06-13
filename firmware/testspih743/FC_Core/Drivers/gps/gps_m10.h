#pragma once

#include "main.h"
#include <stdint.h>
#include "../../MessageBus/topics/sensor_gps.h"

// Cấu trúc dữ liệu cục bộ của GPS
struct GPS_M10_Data_t {
    float lat_deg;
    float lon_deg;
    float alt_m;
    uint8_t fix_quality;
    uint8_t satellites;
    float hdop;
    float speed_knots;
    float course_deg;
    bool valid;
};

class GPS_M10 {
public:
    GPS_M10(UART_HandleTypeDef* huart);
    
    // Gọi hàm này trong loop để parse dữ liệu UART và publish uORB
    void update();
    
    // Gọi hàm này trong ngắt UART RX (RXNE) để hứng byte vào buffer
    void rx_callback(uint8_t byte);

private:
    UART_HandleTypeDef* _huart;
    GPS_M10_Data_t _data;
    
    // NMEA Parsing buffer
    char _rx_buf[128];
    uint8_t _rx_idx;
    
    bool parse_nmea();
    void publish_uorb();
};
