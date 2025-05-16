#include "ConveyorBeltMotor.h"
#include "ColorSensor.h"
#include <MCP23017.h>

// Constructor for ConveyorSystem class
ConveyorSystem::ConveyorSystem() 
  : pwm(0x40), limitSwitchController(0x20) // I2C address 0x20 for MCP23017
{
    // Constructor body is empty
}

/* const uint8_t LIMIT_SWITCH_PIN = 0;  // MCP23017 pin 0 corresponds to GPA0

// Initialize the conveyor system
void ConveyorSystem::begin() {
    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);  // ESP32 allows custom SDA/SCL pins
    limitSwitchController.init();  // Important to initialize the chip

    // Set GPA0 as input
    limitSwitchController.pinMode(LIMIT_SWITCH_PIN, INPUT);

    // Read current pull-up configuration of port A
    uint8_t pullupA = limitSwitchController.readRegister(MCP23017Register::GPPU_A);

    // Set pull-up on GPA0 (bit 0)
    pullupA |= (1 << LIMIT_SWITCH_PIN);

    // Write the updated pull-up configuration back to GPPU_A
    limitSwitchController.writeRegister(MCP23017Register::GPPU_A, pullupA);

    Serial.println("Limit switch configured on MCP23017 GPIO A0 (pin 0)");
}

// Check if the limit switch is pressed
bool ConveyorSystem::isSwitchPressed() {
    // Pull-up makes default state HIGH. Pressed state is LOW.
    return !limitSwitchController.digitalRead(LIMIT_SWITCH_PIN);
} */

void ConveyorSystem::begin() {
    Serial.begin(115200);
    while (!Serial);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(50000);  // 50kHz for stability

    if (!pwm.begin()) {
        Serial.println("PCA9685 not found!");
        while (1);
    }
    pwm.setPWMFreq(1600);
    Serial.println("PWM initialized");

    // Initialize MCP23017 using Lemasle's library
    limitSwitchController.init();

    // Configure all pins (0 to 15) as input with pull-ups enabled individually
    for (uint8_t pin = 0; pin < 16; pin++) {
    limitSwitchController.pinMode(pin, INPUT);
    }
    // Enable pull-ups on pins 0-3 of port A (bits 0-3)
    MCP23017Register targetPullupRegA = MCP23017Register::GPPU_A;
    uint8_t pullupRegA = limitSwitchController.readRegister(targetPullupRegA);
    pullupRegA |= 0x0F;  // 0b00001111 for pins 0,1,2,3
    limitSwitchController.writeRegister(targetPullupRegA, pullupRegA);

    // Enable pull-ups on pins 12-15 of port B (pins 12-15 correspond to bits 4-7 of port B)
    MCP23017Register targetPullupRegB = MCP23017Register::GPPU_B;
    uint8_t pullupRegB = limitSwitchController.readRegister(targetPullupRegB);
    pullupRegB |= 0xF0;  // 0b11110000 for bits 4,5,6,7 (pins 12,13,14,15)
    limitSwitchController.writeRegister(targetPullupRegB, pullupRegB);

    /* // Set all pins to input
    limitSwitchController.writeRegister(MCP23017Register::IODIR_A, 0xFF);
    limitSwitchController.writeRegister(MCP23017Register::IODIR_B, 0xFF);

    // Enable pull-ups
    limitSwitchController.writeRegister(MCP23017Register::GPPU_A, 0xFF);
    limitSwitchController.writeRegister(MCP23017Register::GPPU_B, 0xFF);

    // Test MCP23017 communication
    uint8_t testRead = limitSwitchController.readRegister(MCP23017Register::GPIO_A);
    Serial.print("MCP23017 test read (GPIOA): 0x");
    Serial.println(testRead, HEX);

    if (testRead == 0xFF) {
        Serial.println("MCP23017 communication established");
    } else {
        Serial.println("MCP23017 communication failed!");
        while (1);
    }

    // Configure limit switches
    uint8_t usedPins[] = {0, 1, 2, 3, 12, 13, 14, 15};
    Serial.println("Configuring limit switches:");

    for (uint8_t pin : usedPins) {
        Serial.print("  Setting pin ");
        Serial.print(pin);
        Serial.println(" as INPUT with pull-up");

        delay(10);  // Optional for stability

        MCP23017Register gpioReg = (pin < 8) ? MCP23017Register::GPIO_A : MCP23017Register::GPIO_B;
        uint8_t gpioVal = limitSwitchController.readRegister(gpioReg);
        bool state = !(gpioVal & (1 << (pin % 8)));

        Serial.print("    Readback value: ");
        Serial.println(state ? "LOW (pressed)" : "HIGH");

        if (state) {
            Serial.println("    WARNING: Pin reads LOW (might be pressed or not pulled up)");
        }
    } */

    Serial.println("All switches configured");
}


void ConveyorSystem::ConveyorBeltMotor(void* pvParameters) {
    ConveyorSystem* controller = static_cast<ConveyorSystem*>(pvParameters);
    Serial.println("Conveyor task started");

    for (;;) {
        uint8_t targetSwitch = 16;
        // Check for delivery mode (delivery switches pressed)
        if (controller->isSwitchActiveDebounced(controller->deliverySwitches[0]) && 
            controller->isSwitchActiveDebounced(controller->deliverySwitches[1])) {
            controller->setMode(DELIVER_MODE);
            Serial.println("Delivery mode activated");
        }
        // --- COLLECTION MODE ---
        if (controller->currentMode == COLLECT_MODE) {
            if (detectedColor == "blue" || detectedColor == "red") {
                uint8_t* level = (detectedColor == "blue") ? &controller->blueLevel : &controller->redLevel;
                Serial.println(detectedColor + " Level:" + *level);

                // First detection (move to Level 1)
                if (*level == 0) {
                    *level = 1;
                    Serial.println(detectedColor + " detected (Level 1)");
                }
                // Subsequent detections
                else if (*level > 0 && (!controller->isBusy())) {
                    controller->startCollectionMovement(detectedColor);
                    uint8_t currentLevel = (controller->movingColor == "blue") ? 
                        controller->blueLevel : controller->
                        ;
                    uint8_t targetSwitch = controller->getTargetSwitch(controller->movingColor, currentLevel);
                    Serial.print("TARGET SWITCH: ");
                    Serial.println(controller->getTargetSwitch(controller->movingColor, currentLevel));
                    // Check movement completion
                    if (controller->isMoving) {
                        while (!controller->isSwitchPressed(targetSwitch)) {
                            vTaskDelay(pdMS_TO_TICKS(10)); // sleep for 10ms, yield CPU
                        }

                        //if (controller->isSwitchActiveDebounced(targetSwitch)) {
                        controller->stopAll();
                        controller->isMoving = false;
                        controller->operationInProgress = false;
                            
                        // Advance level (1→2→3→4→0)
                        uint8_t* level = (controller->movingColor == "blue") ? &controller->blueLevel : &controller->redLevel;
                        *level = *level + 1;
                        Serial.println(controller->movingColor + " reached Level " + String(*level));
                    }
                }
                
                delay(1000);
            }

            /* // Check movement completion
            if (controller->isMoving) {
                uint8_t currentLevel = (controller->movingColor == "blue") ? 
                    controller->blueLevel : controller->redLevel;
                uint8_t targetSwitch = controller->getTargetSwitch(controller->movingColor, currentLevel);

                if (controller->isSwitchActiveDebounced(targetSwitch)) {
                    controller->stopAll();
                    controller->isMoving = false;
                    controller->operationInProgress = false;
                    
                    // Advance level (1→2→3→4→0)
                    uint8_t* level = (controller->movingColor == "blue") ? 
                        &controller->blueLevel : &controller->redLevel;
                    *level = *level + 1;
                    Serial.println(controller->movingColor + " reached Level " + String(*level));
                }
            } */
        }
        // --- DELIVERY MODE ---
        else if (controller->currentMode == DELIVER_MODE) {
            controller->executeDelivery();
            controller->setMode(COLLECT_MODE);
            Serial.println("Returned to COLLECT_MODE");
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(NULL);
}

void ConveyorSystem::startCollectionMovement(String color) {
    operationInProgress = true;
    isMoving = true;
    movingColor = color;
    moveStartTime = millis();
    
    const MotorConfig& motor = (color == "blue") ? BLUE_MOTOR : RED_MOTOR;
    moveMotor(motor, motor.collectSpeed);
    Serial.println("Moving " + color + " to Level " + String((color == "blue") ? blueLevel : redLevel));
}

void ConveyorSystem::executeDelivery() {
    operationInProgress = true;
    
    // Get delivery durations based on current levels (both at Level 4)
    int duration = getDeliveryDuration();
    
    // Start both motors
    moveMotor(BLUE_MOTOR, BLUE_MOTOR.deliverSpeed);
    moveMotor(RED_MOTOR, RED_MOTOR.deliverSpeed);
    
    // Run for the delivery duration
    if (duration > 0) {
        delay(duration);
    }
    
    stopAll();
    operationInProgress = false;
}

int ConveyorSystem::getDeliveryDuration(uint8_t level) {
    // Level 1: 1500ms, Level 2: 1200ms, Level 3: 900ms, Level 4: 600ms
    static const int durations[] = {1500, 1200, 900, 600};
    return (level >= 1 && level <= 4) ? durations[level - 1] : 0;
}

uint8_t ConveyorSystem::getTargetSwitch(String color, uint8_t level) {
    if (color == "blue") {
        return blueConveyorSwitches[level - 1];
    } else { // Covers all other cases
        return redConveyorSwitches[level - 1];
    }
}

bool ConveyorSystem::isSwitchPressed(uint8_t switchNumber) {
    return !limitSwitchController.digitalRead(switchNumber);  // Active LOW
}

bool ConveyorSystem::isSwitchActiveDebounced(uint8_t switchNumber) {
    // Determine the correct GPIO register (Port A or B)
    MCP23017Register reg = (switchNumber < 8) ? 
        MCP23017Register::GPIO_A : 
        MCP23017Register::GPIO_B;
    uint8_t mask = (1 << (switchNumber % 8));

    // LOW = pressed
    bool currentState = !(limitSwitchController.readRegister(reg) & mask);

    static bool lastState[16] = {0};
    static unsigned long lastDebounceTime[16] = {0};

    if (currentState != lastState[switchNumber]) {
        lastDebounceTime[switchNumber] = millis();
    }

    if ((millis() - lastDebounceTime[switchNumber]) > 50) {
        lastState[switchNumber] = currentState;
        return currentState;
    }

    return lastState[switchNumber];
}


bool ConveyorSystem::checkDeliveryCondition() {
    return isSwitchActiveDebounced(deliverySwitches[0]) && 
           isSwitchActiveDebounced(deliverySwitches[1]);
}

void ConveyorSystem::setMode(uint8_t mode) {
    currentMode = mode;
}

bool ConveyorSystem::isBusy() {
    return operationInProgress;
}

void ConveyorSystem::moveMotor(const MotorConfig& motor, int speed) {
    pwm.setPWM(motor.in1, 0, speed);
    pwm.setPWM(motor.in2, 0, 0);
}

void ConveyorSystem::stopAll() {
    pwm.setPWM(BLUE_MOTOR.in1, 0, 0);
    pwm.setPWM(BLUE_MOTOR.in2, 0, 0);
    pwm.setPWM(RED_MOTOR.in1, 0, 0);
    pwm.setPWM(RED_MOTOR.in2, 0, 0);
}