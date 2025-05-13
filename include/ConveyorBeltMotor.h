#ifndef CONVEYOR_BELT_MOTOR_H
#define CONVEYOR_BELT_MOTOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Define custom I2C pins for ESP32
#define SDA_PIN 40          // Data pin (SDA)
#define SCL_PIN 41          // Clock pin (SCL)

class MotorController {
public:
    enum OperationMode { COLLECT_MODE, DELIVER_MODE };

    MotorController();
    void begin();

    void setMode(OperationMode mode) { currentMode = mode; }
    bool isBusy() const { return operationInProgress; }

    void executeCollect(String color, int level);
    void executeDeliver(String color, int level);
    void stopAll();

    // Task control functions
    static void ConveyorBeltMotor(void* pvParameters);

private:
    struct MotorConfig {
        uint8_t in1;
        uint8_t in2;
        int collectSpeed;
        int deliverSpeed;

        MotorConfig(uint8_t pin1, uint8_t pin2, int cSpeed = 2500, int dSpeed = 3000)
            : in1(pin1), in2(pin2), collectSpeed(cSpeed), deliverSpeed(dSpeed) {}
    };

    Adafruit_PWMServoDriver pwm;
    OperationMode currentMode = COLLECT_MODE;
    bool operationInProgress = false;

    // Motor configurations (fixed typo in BLUE_MOTOR)
    const MotorConfig BLUE_MOTOR {1, 2};  // Pin 1 and 2, with default speeds
    const MotorConfig RED_MOTOR {3, 4};   // Pin 3 and 4, with default speeds

    // Motor control functions
    void moveMotor(const MotorConfig& motor, int speed);
    int getDuration(int level, bool isCollect) const;
};

#endif // CONVEYOR_BELT_MOTOR_H