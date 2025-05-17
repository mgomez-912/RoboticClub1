  // #include "SBUSReader.h"
  // #include "sbus.h"

  // #define SBUS_CHANNELS 16
  // #define SBUS_RX_PIN 16   // Replace with your real pin
  // #define SBUS_TX_PIN 17   // Not used but needed by Serial2

  // HardwareSerial sbusSerial(2);       // Use UART1
  // bfs::SbusRx sbus_rx(&sbusSerial, SBUS_RX_PIN, SBUS_TX_PIN, true, false);
  // bfs::SbusData sbus_data;

  // int channelSbus[SBUS_CHANNELS];
  // bool sbusFailsafe = false;
  // bool sbusLostFrame = false;

  // void SBUSRead(void *pvParameters) {
  //     sbusInit();  // Initialize UART and SBUS
  //     while (true) {
  //         sbusReading();
  //         vTaskDelay(taskRXRead.getIntervalms() / portTICK_PERIOD_MS);
  //     }
  // }

  // void sbusInit() {
  //   sbusSerial.begin(100000, SERIAL_8E2, SBUS_RX_PIN, SBUS_TX_PIN);
  //   sbus_rx.Begin();
  // }

  // void sbusReading() {
  //   if (sbus_rx.Read()) {
  //     sbus_data = sbus_rx.data();

  //     for (int i = 0; i < SBUS_CHANNELS; i++) {
  //       channelSbus[i] = sbus_data.ch[i];
  //     //   channelSbus[i] = sbusToMicroseconds(sbus_data.ch[i]);
  //     }

  //     // Print the most recent values for each channel
  //     for (int i = 0; i < SBUS_CHANNELS; i++) {

  //         Serial.print(channelSbus[i]);
  //         Serial.print(" \t");
  //     }
  //     Serial.println();

  //     sbusFailsafe = sbus_data.failsafe;
  //     sbusLostFrame = sbus_data.lost_frame;
  //   }
  // }

  // int sbusToMicroseconds(uint16_t sbusVal) {
  //   // SBUS usually outputs from ~172 to ~1811
  //   return map(sbusVal, 172, 1811, 900, 2100);  // Adjust as needed
  // }