#include "gps_m10.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static uORB::PublicationMulti<sensor_gps_s> gps_pub(orb_sensor_gps);

GPS_M10::GPS_M10(UART_HandleTypeDef* huart) : _huart(huart), _rx_idx(0) {
    memset(&_data, 0, sizeof(_data));
}

void GPS_M10::rx_callback(uint8_t byte) {
    if (byte == '$') {
        _rx_idx = 0;
    }
    if (_rx_idx < sizeof(_rx_buf) - 1) {
        _rx_buf[_rx_idx++] = byte;
        if (byte == '\n') {
            _rx_buf[_rx_idx] = '\0';
            if (parse_nmea()) {
                publish_uorb();
            }
            _rx_idx = 0;
        }
    } else {
        _rx_idx = 0; // Overflow
    }
}

void GPS_M10::update() {
    // Nếu dùng ngắt DMA hoặc ngắt RXNE, hàm này có thể check timeout hoặc state
    // Tạm thời parser đã chạy trong rx_callback() khi thấy '\n'.
    // Chú ý: Ở FC thực tế, KHÔNG NÊN parse trong ISR. 
    // Tốt nhất là rx_callback() dùng RingBuffer, và update() lấy từ RingBuffer ra để parse.
    // (Đây là logic cơ bản để demo MAVLink GPS)
}

bool GPS_M10::parse_nmea() {
    // Chỉ parse GGA để lấy tọa độ và fix quality (ví dụ tối giản)
    if (strncmp(_rx_buf, "$GPGGA", 6) == 0 || strncmp(_rx_buf, "$GNGGA", 6) == 0) {
        // Tách chuỗi bằng strtok (lưu ý strtok không an toàn với chuỗi có dấu phẩy rỗng,
        // nhưng đây là bản demo)
        char* token = strtok(_rx_buf, ","); // Header
        token = strtok(NULL, ",");          // Time
        token = strtok(NULL, ",");          // Lat
        if (token) {
            float raw_lat = atof(token);
            _data.lat_deg = (int)(raw_lat / 100) + fmod(raw_lat, 100.0) / 60.0;
        }
        token = strtok(NULL, ",");          // N/S
        if (token && token[0] == 'S') _data.lat_deg = -_data.lat_deg;
        
        token = strtok(NULL, ",");          // Lon
        if (token) {
            float raw_lon = atof(token);
            _data.lon_deg = (int)(raw_lon / 100) + fmod(raw_lon, 100.0) / 60.0;
        }
        token = strtok(NULL, ",");          // E/W
        if (token && token[0] == 'W') _data.lon_deg = -_data.lon_deg;
        
        token = strtok(NULL, ",");          // Fix Quality
        if (token) _data.fix_quality = atoi(token);
        
        token = strtok(NULL, ",");          // Satellites
        if (token) _data.satellites = atoi(token);
        
        token = strtok(NULL, ",");          // HDOP
        if (token) _data.hdop = atof(token);
        
        token = strtok(NULL, ",");          // Alt
        if (token) _data.alt_m = atof(token);
        
        _data.valid = (_data.fix_quality > 0);
        return _data.valid;
    }
    return false;
}

void GPS_M10::publish_uorb() {
    sensor_gps_s msg;
    msg.timestamp_us = HAL_GetTick() * 1000ULL;
    msg.lat = (int32_t)(_data.lat_deg * 1E7);
    msg.lon = (int32_t)(_data.lon_deg * 1E7);
    msg.alt = (int32_t)(_data.alt_m * 1000.0f);
    msg.alt_ellipsoid = msg.alt;
    
    // Ánh xạ NMEA fix -> uORB fix type
    if (_data.fix_quality == 0) msg.fix_type = 0; // No fix
    else if (_data.fix_quality == 1) msg.fix_type = 3; // 3D fix
    else if (_data.fix_quality == 2) msg.fix_type = 4; // DGPS
    else if (_data.fix_quality == 4) msg.fix_type = 6; // RTK Fixed
    else msg.fix_type = 3;
    
    msg.satellites_used = _data.satellites;
    msg.hdop = _data.hdop;
    msg.vdop = _data.hdop; // Tạm thời
    msg.vel_m_s = _data.speed_knots * 0.514444f;
    msg.cog_rad = _data.course_deg * 3.14159265f / 180.0f;
    
    msg.device_id = 0;
    
    gps_pub.publish(msg);
}
