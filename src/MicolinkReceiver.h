#ifndef MICOLINKRECEIVER_H
#define MICOLINKRECEIVER_H

#include <Arduino.h>

struct MicoSensorData {
    uint32_t system_time;
    uint32_t raw_distance;  // Raw distance in mm
    uint8_t signal_strength;
    uint8_t reserved1;
    uint8_t distance_status;
    uint8_t reserved2;
    int16_t raw_flow_speed_x;  // Raw flow speed in cm/s
    int16_t raw_flow_speed_y;  // Raw flow speed in cm/s
    uint8_t flow_quality;
    uint8_t flow_status;
    uint16_t reserved3;
    float distance;      // Distance in m
    float flow_speed_x;  // Flow speed in m/s at 1 m
    float flow_speed_y;  // Flow speed in m/s at 1 m
};

class MicolinkReceiver {
public:
    explicit MicolinkReceiver(HardwareSerial& serial_port);

    void begin(long baud_rate);
    bool receive();

    MicoSensorData data;
private:
    HardwareSerial& serial;
    static constexpr uint8_t HEADER_BYTE = 0xEF;
    static constexpr uint8_t DEVICE_ID = 0x0F;
    static constexpr uint8_t SYSTEM_ID = 0x00;
    static constexpr uint8_t MESSAGE_ID = 0x51;
    static constexpr uint8_t PAYLOAD_LENGTH = 0x14;

    static constexpr float DISTANCE_CONVERSION_FACTOR = 0.001;  // Convert mm to m
    static constexpr float FLOW_SPEED_CONVERSION_FACTOR = 0.01;  // Convert cm/s to m/s (at 1 m)

    uint8_t calculate_checksum(const uint8_t* data, size_t length) const;
};

#endif // MICOLINKRECEIVER_H