#include "MicolinkReceiver.h"

MicolinkReceiver::MicolinkReceiver(HardwareSerial& serialPort) : serial(serialPort) {}

void MicolinkReceiver::begin(long baudRate) {
    this->serial.begin(baudRate);
}

uint8_t MicolinkReceiver::calculate_checksum(const uint8_t* data, size_t length) const {
    uint8_t checksum = 0;
    for (size_t i = 0; i < length; ++i) {
        checksum += data[i];
    }
    return checksum;
}

bool MicolinkReceiver::receive() {
    if (serial.available() >= 7 + PAYLOAD_LENGTH) {
        // Read and verify the header bytes
        uint8_t header[5];
        header[0] = serial.read(); // HEADER_BYTE
        header[1] = serial.read(); // DEVICE_ID
        header[2] = serial.read(); // SYSTEM_ID
        header[3] = serial.read(); // MESSAGE_ID

        // Check if the header is correct
        if (header[0] == HEADER_BYTE &&
            header[1] == DEVICE_ID &&
            header[2] == SYSTEM_ID &&
            header[3] == MESSAGE_ID) {

            // Read the sequence number
            uint8_t sequence_number = serial.read();

            // Read payload length
            uint8_t length = serial.read();
            if (length == PAYLOAD_LENGTH) {
                // Read payload
                uint8_t payload[PAYLOAD_LENGTH];
                for (uint8_t i = 0; i < PAYLOAD_LENGTH; ++i) {
                    payload[i] = serial.read();
                }

                // Read checksum
                uint8_t received_checksum = serial.read();

                // Calculate checksum including header, sequence number, length, and payload
                uint8_t calculated_checksum = calculate_checksum(header, 4); // Header bytes
                calculated_checksum += sequence_number + length;
                calculated_checksum += calculate_checksum(payload, PAYLOAD_LENGTH); // Payload

                // Verify checksum
                if (received_checksum == calculated_checksum) {
                    // Parse the payload
                    this->data.system_time = (uint32_t)payload[0] |
                                             ((uint32_t)payload[1] << 8) |
                                             ((uint32_t)payload[2] << 16) |
                                             ((uint32_t)payload[3] << 24);
                    this->data.raw_distance = (uint32_t)payload[4] |
                                              ((uint32_t)payload[5] << 8) |
                                              ((uint32_t)payload[6] << 16) |
                                              ((uint32_t)payload[7] << 24);
                    this->data.signal_strength = payload[8];
                    this->data.reserved1 = payload[9];
                    this->data.distance_status = payload[10];
                    this->data.reserved2 = payload[11];
                    this->data.raw_flow_speed_x = (int16_t)payload[12] |
                                                  ((int16_t)payload[13] << 8);
                    this->data.raw_flow_speed_y = (int16_t)payload[14] |
                                                  ((int16_t)payload[15] << 8);
                    this->data.flow_quality = payload[16];
                    this->data.flow_status = payload[17];
                    this->data.reserved3 = (uint16_t)payload[18] |
                                           ((uint16_t)payload[19] << 8);

                    // Convert raw unit to SI unit
                    this->data.distance = data.raw_distance * DISTANCE_CONVERSION_FACTOR;
                    this->data.flow_speed_x = data.raw_flow_speed_x * FLOW_SPEED_CONVERSION_FACTOR;
                    this->data.flow_speed_y = data.raw_flow_speed_y * FLOW_SPEED_CONVERSION_FACTOR;

                    return true;
                } else {
                    return false; // Checksum mismatch
                }
            }
        }
    }
    return false; // Not enough data available
}