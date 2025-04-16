#include "LineFol.h"

HardwareSerial SerialLine(1);

void LineFollow(void *pvParameters) {
    SerialLine.begin(115200, SERIAL_8N1, lineRX, lineTX);  // Initialize custom UART

    while(true) {
        // Check for incoming UART data
        if (SerialLine.available()) {
          Serial.print("Received: ");
          Serial.println(SerialLine.readString());
        }

        vTaskDelay(taskLineFollow.getIntervalms() / portTICK_PERIOD_MS);  // Delay for the blink time
    }
}



//This must be adapted to the task.
// #include <Arduino.h>

// // UART Configuration
// #define SENSOR_RX 16
// #define SENSOR_TX 17
// HardwareSerial SensorSerial(2);

// // Data variables
// uint8_t Patrol_data[3];
// unsigned long lastRequest = 0;
// int deltime = 500;

// void processSensorData();
// int calculatePosition(uint8_t status);
// void sendRequest() ;

// void setup() {
//   Serial.begin(115200);
//   SensorSerial.begin(9600, SERIAL_8N1, SENSOR_RX, SENSOR_TX);
//   Serial.println("Sensor Monitoring Started");
//   sendRequest();
// }

// void sendRequest() {
//   SensorSerial.write(0x57);
//   lastRequest = millis();
// }

// // Not returning values like 500, 1500, it need to be checked
// int calculatePosition(uint8_t status) {
//   // Mask out the direction bit (LSB)
//   uint8_t pattern = status >> 1;
  
//   // Special case for center position
//   if (pattern == 0b01111111) return 0;
  
//   // Count leading ones in the 7-bit pattern
//   uint8_t leadingOnes = 0;
//   uint8_t mask = 0b1000000;  // Check 7 bits (excluding LSB)
  
//   while (mask && (pattern & mask)) {
//     leadingOnes++;
//     mask >>= 1;
//   }
  
//   // Calculate position value
//   int position = (7 - leadingOnes) * 1000;
  
//   // Adjust for direction using LSB
//   if (!(status & 0x01)) {
//     position = 7000 - position;
//   }
  
//   return position;
// }

// void processSensorData() {
//   if (millis() - lastRequest > deltime*2) {
//     while (SensorSerial.available() > 0) SensorSerial.read();
//     sendRequest();
//     return;
//   }

//   if (SensorSerial.available()) {
//     Patrol_data[0] = SensorSerial.read();  // Status byte
//     Patrol_data[1] = SensorSerial.read();  // High byte
//     Patrol_data[2] = SensorSerial.read();  // Low byte

//     // Calculate position from status byte
//     int position = calculatePosition(Patrol_data[0]);

//     // Debug output
//     Serial.print("Status: 0b");
//     for (int i = 7; i >= 0; i--) {
//       Serial.print((Patrol_data[0] >> i) & 0x01);
//     }
//     Serial.print(" | Position: ");
//     Serial.println(position);

//     sendRequest();
//   }
// }

// void loop() {
//   processSensorData();
//   delay(deltime);
// }