#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>

// ========================================
// PACKET PROTOCOL DEFINITION (Phase 2)
// ========================================
// Pi → ESP (Command Packet): 8 bytes
// [0xA5, RPM1_H, RPM1_L, RPM2_H, RPM2_L, RPM3_H, RPM3_L, CHECKSUM]
//
// ESP → Pi (Feedback Packet): 18 bytes  
// [0xA5, Tick1(4), Tick2(4), Tick3(4), GyroZ(2), AccelZ(2), CHECKSUM]
// ========================================

#define PACKET_HEADER       0xA5
#define CMD_PACKET_SIZE     8
#define FEEDBACK_PACKET_SIZE 18
#define WATCHDOG_TIMEOUT_MS 1000

struct CommandPacket {
    int16_t rpm1;
    int16_t rpm2;
    int16_t rpm3;
    bool valid;
};

struct FeedbackPacket {
    int32_t tick1; //delta ticks
    int32_t tick2;
    int32_t tick3;
    int16_t gyro_z;
    int16_t accel_z;
};

class SerialProtocol {
private:
    HardwareSerial* _serial;
    uint8_t _rxBuffer[CMD_PACKET_SIZE];
    uint8_t _rxIndex;
    unsigned long _lastValidPacketTime;
    bool _watchdogTriggered;
    void (*_motorStopCallback)();
    
    uint8_t calculateChecksum(uint8_t* data, uint8_t len) {
        uint8_t checksum = 0;
        for (uint8_t i = 0; i < len; i++) {
            checksum ^= data[i];
        }
        return checksum;
    }
    
    int16_t parseInt16BE(uint8_t* data) {
        return (int16_t)((data[0] << 8) | data[1]);
    }

public:
    SerialProtocol(HardwareSerial* serial) : _serial(serial) {
        _rxIndex = 0;
        _lastValidPacketTime = 0;
        _watchdogTriggered = false;
        _motorStopCallback = nullptr;
    }
    
    void begin(unsigned long baudrate) {
        _serial->begin(baudrate);
        _lastValidPacketTime = millis();
    }
    
    void setMotorStopCallback(void (*callback)()) {
        _motorStopCallback = callback;
    }
    
    CommandPacket parseCommand() {
        CommandPacket cmd = {0, 0, 0, false};
        
        while (_serial->available() > 0) {
            uint8_t byte = _serial->read();
            
            if (_rxIndex == 0) {
                if (byte == PACKET_HEADER) {
                    _rxBuffer[_rxIndex++] = byte;
                }
            } else {
                _rxBuffer[_rxIndex++] = byte;
                
                if (_rxIndex >= CMD_PACKET_SIZE) {
                    uint8_t receivedChecksum = _rxBuffer[CMD_PACKET_SIZE - 1];
                    uint8_t calculatedChecksum = calculateChecksum(_rxBuffer, CMD_PACKET_SIZE - 1);
                    
                    if (receivedChecksum == calculatedChecksum) {
                        cmd.rpm1 = parseInt16BE(&_rxBuffer[1]);
                        cmd.rpm2 = parseInt16BE(&_rxBuffer[3]);
                        cmd.rpm3 = parseInt16BE(&_rxBuffer[5]);
                        cmd.valid = true;
                        
                        _lastValidPacketTime = millis();
                        _watchdogTriggered = false;
                    }
                    
                    _rxIndex = 0;
                }
            }
        }
        
        return cmd;
    }
    
    void sendFeedback(const FeedbackPacket& feedback) {
        uint8_t packet[FEEDBACK_PACKET_SIZE];
        uint8_t idx = 0;
        
        packet[idx++] = PACKET_HEADER;
        
        // Tick1 (Big Endian int32)
        packet[idx++] = (feedback.tick1 >> 24) & 0xFF;
        packet[idx++] = (feedback.tick1 >> 16) & 0xFF;
        packet[idx++] = (feedback.tick1 >> 8) & 0xFF;
        packet[idx++] = feedback.tick1 & 0xFF;
        
        // Tick2
        packet[idx++] = (feedback.tick2 >> 24) & 0xFF;
        packet[idx++] = (feedback.tick2 >> 16) & 0xFF;
        packet[idx++] = (feedback.tick2 >> 8) & 0xFF;
        packet[idx++] = feedback.tick2 & 0xFF;
        
        // Tick3
        packet[idx++] = (feedback.tick3 >> 24) & 0xFF;
        packet[idx++] = (feedback.tick3 >> 16) & 0xFF;
        packet[idx++] = (feedback.tick3 >> 8) & 0xFF;
        packet[idx++] = feedback.tick3 & 0xFF;
        
        // Gyro Z (Big Endian int16)
        packet[idx++] = (feedback.gyro_z >> 8) & 0xFF;
        packet[idx++] = feedback.gyro_z & 0xFF;
        
        // Accel Z
        packet[idx++] = (feedback.accel_z >> 8) & 0xFF;
        packet[idx++] = feedback.accel_z & 0xFF;
        
        // Checksum
        packet[idx++] = calculateChecksum(packet, FEEDBACK_PACKET_SIZE - 1);
        
        _serial->write(packet, FEEDBACK_PACKET_SIZE);
    }
    
    bool checkWatchdog() {
        unsigned long elapsed = millis() - _lastValidPacketTime;
        
        if (elapsed > WATCHDOG_TIMEOUT_MS && !_watchdogTriggered) {
            _watchdogTriggered = true;
            
            if (_motorStopCallback != nullptr) {
                _motorStopCallback();
            }
            
            return true;
        }
        
        return false;
    }
    
    bool isCommHealthy() {
        return !_watchdogTriggered;
    }
    
    void resetWatchdog() {
        _lastValidPacketTime = millis();
        _watchdogTriggered = false;
    }
    
    unsigned long getTimeSinceLastPacket() {
        return millis() - _lastValidPacketTime;
    }
};

#endif
