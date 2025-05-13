#include "ConveyorBeltMotor.h"
#include "ColorSensor.h"

// Constructor
MotorController::MotorController() : pwm(0x40) {}

// Inicializa el controlador PWM
void MotorController::begin() {
    Serial.begin(115200);
    while(!Serial); // Wait for serial monitor in debug mode
  
    // Serial.printf("Using I2C pins: SDA=%d, SCL=%d\n", SDA_PIN, SCL_PIN);
  
    Wire.begin(SDA_PIN, SCL_PIN);
    if(!pwm.begin()) {
      Serial.println("PCA9685 not found!");
      while(1); // Halt if initialization fails
    }
    Serial.println("PWM initialized");
}

// Ejecuta la acción de recolectar
void MotorController::executeCollect(String color, int level) {
    if (isBusy() || currentMode != COLLECT_MODE) return;

    operationInProgress = true;
    const MotorConfig& motor = (color == "blue") ? BLUE_MOTOR : RED_MOTOR;
    moveMotor(motor, motor.collectSpeed);
    delay(getDuration(level, true));
    moveMotor(motor, 0);
    operationInProgress = false;
}

// Ejecuta la acción de entregar
void MotorController::executeDeliver(String color, int level) {
    if (isBusy() || currentMode != DELIVER_MODE) return;

    operationInProgress = true;
    const MotorConfig& motor = (color == "blue") ? BLUE_MOTOR : RED_MOTOR;
    moveMotor(motor, motor.deliverSpeed);
    delay(getDuration(level, false));
    moveMotor(motor, 0);
    operationInProgress = false;
}

// Mueve el motor con una velocidad específica
void MotorController::moveMotor(const MotorConfig& motor, int speed) {
    pwm.setPWM(motor.in1, 0, speed);
    pwm.setPWM(motor.in2, 0, 0);
}

// Detiene todos los motores
void MotorController::stopAll() {
    pwm.setPWM(BLUE_MOTOR.in1, 0, 0);
    pwm.setPWM(BLUE_MOTOR.in2, 0, 0);
    pwm.setPWM(RED_MOTOR.in1, 0, 0);
    pwm.setPWM(RED_MOTOR.in2, 0, 0);
}

// Calcula la duración de movimiento
int MotorController::getDuration(int level, bool isCollect) const {
    static const int DURATIONS[2][4] = {
        {0, 1000, 1000, 1000},  // Duraciones para recolección
        {1500, 1000, 1000, 500}   // Duraciones para entrega
    };
    return (level >= 1 && level <= 4) ? DURATIONS[!isCollect][level - 1] : 0;
}

void MotorController::ConveyorBeltMotor(void* pvParameters) {
    if(pvParameters == nullptr) {
        Serial.println("NULL controller pointer!");
        vTaskDelete(NULL);
    }
    
    MotorController* controller = static_cast<MotorController*>(pvParameters);
    Serial.println("Conveyor task started");
    const int detectionDelay = 500; // ms wait after color detection
    
    // Test variables
    int blueLevel = 0;  // Start at level 0
    int redLevel = 0;   // Start at level 0
    bool testMode = true; // Enable test mode
    
    for (;;) {
        if (controller->currentMode == COLLECT_MODE) {
            if ((detectedColor == "red" || detectedColor == "blue")) {
                vTaskDelay(pdMS_TO_TICKS(detectionDelay));
                
                // Determine which motor to activate
                const bool isBlue = (detectedColor == "blue");
                const MotorConfig& motor = isBlue ? 
                    controller->BLUE_MOTOR : controller->RED_MOTOR;
                
                // Get current level and increment
                int& currentLevel = isBlue ? blueLevel : redLevel;
                const int duration = controller->getDuration(currentLevel, true);
                
                // Only run if duration > 0 (level > 0)
                if (duration > 0) {
                    controller->moveMotor(motor, motor.collectSpeed);
                    vTaskDelay(pdMS_TO_TICKS(duration));
                    controller->stopAll();
                }
                
                // Increment level for next time
                currentLevel = (currentLevel + 1) % 5; // Cycle 0-4
                
                // Check if both reached level 4 and switch to delivery
                if (blueLevel >= 4 && redLevel >= 4) {
                    controller->setMode(DELIVER_MODE);
                    // Reset levels for delivery test
                    blueLevel = 0;
                    redLevel = 0;
                }
            }
            else {
                controller->stopAll();
            }
        } 
        else { // DELIVER_MODE
            // Delivery test behavior
            vTaskDelay(pdMS_TO_TICKS(200)); // Small initial delay
            
            // Increment levels for delivery test
            blueLevel = (blueLevel + 1) % 5;
            redLevel = (redLevel + 1) % 5;
            
            // Get durations (skip if level 0)
            const int blueDuration = blueLevel > 0 ? controller->getDuration(blueLevel, false) : 0;
            const int redDuration = redLevel > 0 ? controller->getDuration(redLevel, false) : 0;
            
            if (blueDuration > 0 || redDuration > 0) {
                // Start motors that have non-zero duration
                if (blueDuration > 0) {
                    controller->moveMotor(controller->BLUE_MOTOR, controller->BLUE_MOTOR.deliverSpeed);
                }
                if (redDuration > 0) {
                    controller->moveMotor(controller->RED_MOTOR, controller->RED_MOTOR.deliverSpeed);
                }
                
                // Handle different durations
                const int maxDuration = max(blueDuration, redDuration);
                vTaskDelay(pdMS_TO_TICKS(maxDuration));
                controller->stopAll();
                
                // Switch back to collect mode if both reached level 4
                if (blueLevel >= 4 && redLevel >= 4) {
                    controller->setMode(COLLECT_MODE);
                    // Reset levels for next cycle
                    blueLevel = 0;
                    redLevel = 0;
                }
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(50)); // Small delay between checks
    }
    
    vTaskDelete(NULL);
}