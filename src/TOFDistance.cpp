#include "TOFDistance.h"

HardwareSerial TOFSerial(2); // TX: 17, RX: 16
volatile long tofDistanceMM = -1;

const byte SYSCMD[4][8] = {
    {0x01, 0x06, 0x00, 0x20, 0x00, 0x8C, 0x89, 0xA5},
    {0x01, 0x06, 0x00, 0x21, 0x00, 0x64, 0xD8, 0x2B},
    {0x01, 0x06, 0x00, 0x06, 0x00, 0x01, 0xA8, 0x0B},
    {0x01, 0x06, 0x00, 0x01, 0x10, 0x00, 0xD5, 0xCA},
};
const byte distanceCMD[2][8] = {
    {0x01, 0x06, 0x00, 0x04, 0x00, 0x00, 0xC8, 0x0B},
    {0x01, 0x06, 0x00, 0x04, 0x00, 0x01, 0x09, 0xCB},
};
const byte timeCMD[1][8] = {
    {0x01, 0x06, 0x00, 0x05, 0x01, 0xF4, 0x99, 0xDC},
};

void sendCommand(const byte *cmd) {
    for (int i = 0; i < 8; i++)
        TOFSerial.write(cmd[i]);
}

void initTOFDistanceSensor() {
    sendCommand(distanceCMD[1]);
    delay(2000);
    sendCommand(timeCMD[0]);
    delay(2000);
    sendCommand(SYSCMD[3]);
    delay(2000);
}

long getTOFDistance() {
    return tofDistanceMM;
}

void TOFDistanceTask(void *pvParameters) {
    Serial.println("TOF Distance Task Started");
    while (true) {
        if (TOFSerial.available() > 6) {
            char a = TOFSerial.read();
            if (a != 0x01)
                continue;
            byte Buf[6];
            TOFSerial.readBytes(Buf, 6);

            if (Buf[2] == 0xFF) {
                tofDistanceMM = -1;
            } else {
                tofDistanceMM = Buf[2] * 256 + Buf[3];
            }
        }
        vTaskDelay(1); // Minimal delay to yield
    }
}