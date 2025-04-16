//These functions are for read the packet from openMV h7 plus after object detection
//Ores_openmvUART.py
#include "UARTRead.h"

HardwareSerial SerialPort(2);
DetectionData detection;

// State machine variables
static uint8_t buffer[sizeof(DetectionData)];
static uint8_t idx = 0;

void initializeUART() {
    Serial.begin(115200);
    SerialPort.begin(115200, SERIAL_8N1, 16, 17);
}

void readUART() {
    static enum { SYNC1, SYNC2, READ } state = SYNC1;
    while (SerialPort.available()) {
        uint8_t byte = SerialPort.read();

        switch(state) {
            case SYNC1:
                if(byte == 0xAA) state = SYNC2;
                break;

            case SYNC2:
                if(byte == 0xBB) {
                    state = READ;
                    idx = 0;
                } else {
                    state = SYNC1;
                }
                break;

            case READ:
                buffer[idx++] = byte;
                if(idx == sizeof(DetectionData)) {
                    // Copy to struct
                    memcpy(&detection, buffer, sizeof(DetectionData));
                    
                    // Convert endianness
                    detection.label = __builtin_bswap16(detection.label);
                    detection.x = __builtin_bswap16(detection.x);
                    detection.y = __builtin_bswap16(detection.y);

                    if(detection.label == 0xFFFF) {
                        Serial.println("No detection");
                    } else {
                        Serial.printf("Detected: %u @ (%u,%u)\n", 
                                    detection.label, detection.x, detection.y);
                    }
                    state = SYNC1;
                }
                break;
        }
    }
}