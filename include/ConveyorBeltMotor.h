#ifndef CONVEYOR_BELT_MOTOR_H
#define CONVEYOR_BELT_MOTOR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
//#include <Adafruit_MCP23017.h>
#include <MCP23017.h>

#define COLLECT_MODE 0
#define DELIVER_MODE 1

struct MotorConfig {
    uint8_t in1;
    uint8_t in2;
    uint16_t collectSpeed;
    uint16_t deliverSpeed;
};

class ConveyorSystem {
public:
    ConveyorSystem();
    void begin();
    static void ConveyorBeltMotor(void* pvParameters);
    
    // Motor control
    void moveMotor(const MotorConfig& motor, int speed);
    void stopAll();
    
    // Mode control
    void setMode(uint8_t mode);
    bool isBusy();

    bool isSwitchPressed(uint8_t switchNumber);

private:
    // Internal movement methods
    void startCollectionMovement(String color);
    void executeDelivery();
    
    // Switch handling
    //bool isSwitchPressed(uint8_t switchNumber);
    bool isSwitchActiveDebounced(uint8_t switchNumber);
    bool checkDeliveryCondition();
    uint8_t getTargetSwitch(String color, uint8_t level);
    
    // Helper functions
    int getDeliveryDuration(uint8_t level);

    // Hardware controllers
    Adafruit_PWMServoDriver pwm;
    MCP23017 limitSwitchController;
    
    // State variables
    uint8_t currentMode = COLLECT_MODE;
    bool operationInProgress = false;
    bool isMoving = false;
    String movingColor = "";
    unsigned long moveStartTime = 0;
    
    // Level tracking (0-4)
    uint8_t blueLevel = 0;
    uint8_t redLevel = 0;
    
    // Motor configurations
    const MotorConfig BLUE_MOTOR = {0, 1, 4095, 2048};
    const MotorConfig RED_MOTOR = {3, 2, 4095, 2048};
    
    #ifndef SDA_PIN
        #define SDA_PIN 40   // Default if not defined elsewhere
    #endif
  
    #ifndef SCL_PIN
        #define SCL_PIN 41   // Default if not defined elsewhere
    #endif
    
    // Switch mappings
    const uint8_t blueConveyorSwitches[3] = {13, 12, 15};
    const uint8_t redConveyorSwitches[3] = {0, 1, 2};
    const uint8_t deliverySwitches[2] = {14, 3};     
};

#endif