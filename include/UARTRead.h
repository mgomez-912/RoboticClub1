#include <Arduino.h>
#include "GlobalVariables.h"

#define START_BYTE_1 0xAA
#define START_BYTE_2 0xBB

extern HardwareSerial SerialPort; // UART2: RX=16, TX=17
const int pinRX = 16;
const int pinTX = 17;

#pragma pack(push, 1)
struct DetectionData {
    uint16_t label;
    uint16_t x;
    uint16_t y;
};
#pragma pack(pop)

extern HardwareSerial SerialPort;
extern DetectionData detection;

void initializeUART();
void readUART();
