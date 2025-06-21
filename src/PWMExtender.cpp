#include "PWMExtender.h"

// Initialize the PWM driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- Using a Struct for Cleaner Position Definitions ---
struct ArmPosition {
    int rotation;
    int angle;
    int shoulder;
    int elbow;
    int wrist;
};

// --- Global State and Position Variables ---

// Define all arm positions using the new struct
const ArmPosition storagePos = {front - 5, 105, 155, 130, 135};
const ArmPosition deliveryPos = {front, 90, 65, 85, 135};
const ArmPosition liftUpPos   = {front, 120, 20, 145, 135}; // Safe transition pose
const ArmPosition pickPos1    = {front, 175, 10, 170, 135};

// State machine control
State currentState = WAIT_FOR_OBJECT;
bool actionInProgress = false;

// Store the current state of the arm's joints to make intelligent decisions
int currentShoulder = 0; 
int currentElbow = 0;


// --- Core Functions ---

void PWMExtender(void *pvParameters) {
    // NOTE: This initialization should ideally be in your main setup() function, not here.
    // Calling it once is sufficient.
    initializeServo();
    vTaskDelay(10);

    while(true) {
        inputHandle();
        vTaskDelay(taskPWMExt.getIntervalms() / portTICK_PERIOD_MS);
    }
}

// Function to initialize the PCA9685
void initializeServo() {
    Serial.begin(115200);
    while(!Serial); // Wait for serial monitor

    Wire.begin(SDA_PIN, SCL_PIN);
    if(!pwm.begin()) {
        Serial.println("PCA9685 not found!");
        while(1); // Halt if initialization fails
    }
  
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);
}

// Function to set a single servo's angle
void setServoAngle(uint8_t channel, uint8_t angle) {
    angle = constrain(angle, 0, 180);
    uint16_t pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
    pwm.setPWM(channel, 0, pulse);
}


// --- Simplified and Optimized Movement Functions ---

/**
 * @brief Sets all arm servos to a target position and waits for the move to complete.
 * @param pos The target ArmPosition to move to.
 * @param totalMoveTime The time in milliseconds to allow for the physical movement.
 */
void armPos(const ArmPosition& pos, int totalMoveTime) {
    // Update global state of the arm
    currentShoulder = pos.shoulder;
    currentElbow = pos.elbow;

    // Command all servos to move to their target angles concurrently
    setServoAngle(0, pos.rotation);
    setServoAngle(1, pos.angle);
    setServoAngle(2, pos.shoulder);
    setServoAngle(3, pos.elbow);
    setServoAngle(4, pos.wrist);

    // Wait for the physical movement to complete with a single delay
    vTaskDelay(pdMS_TO_TICKS(totalMoveTime));
}

/**
 * @brief Intelligently moves the arm to a target, going to a safe lift position first if needed.
 * This function replaces processArmPosition() and moveArmSafely().
 * @param targetPos The final destination for the arm.
 * @param moveTime The time allocated for the final movement.
 */
void moveTo(const ArmPosition& targetPos, int moveTime) {
    // Heuristic to decide if a safe lift is needed:
    // If the arm is currently in a "low" position (e.g., picking something up).
    // You can tune these numbers based on your arm's physical behavior.
    bool needsSafeLift = (currentShoulder < 45 || currentElbow > 140);

    if (needsSafeLift) {
        // Move to the safe "lift up" position first to avoid collisions
        armPos(liftUpPos, 400); // Allow 400ms for the lift
    }

    // Now, move to the final target position
    armPos(targetPos, moveTime);
}


// --- Main Application Logic (State Machine) ---

/**
 * @brief Manages the automated pick-and-place sequence using a state machine.
 */
void inputHandle() {
    static unsigned long lastActionTime = 0;

    switch (currentState) {
        case WAIT_FOR_OBJECT:
            if (!actionInProgress) {
                moveTo(pickPos1, 300); // Move to the pickup position
                actionInProgress = true;
            }
            speedlim = 50;
            // Condition to transition to the next state
            if (distance > 0 && distance < 20) {
                // If you want to use requestedTarget and processArmPosition, make sure they are defined and meaningful
                // requestedTarget = PICKUP_1;
                // processArmPosition();
                currentState = CLOSE_CLAW;
                actionInProgress = false; // Reset flag for the next state
            }
            break;

        case CLOSE_CLAW:
            if (!actionInProgress) {
                setServoAngle(5, closedClaw); // Close the claw
                actionInProgress = true;
                lastActionTime = millis();
            }
            // Wait for a non-blocking delay before transitioning
            if (millis() - lastActionTime > 250) {
                currentState = MOVE_TO_STORAGE;
                actionInProgress = false;
                speedlim = 25;
            }
            break;

        case MOVE_TO_STORAGE:
            if (!actionInProgress) {
                moveTo(storagePos, 300); // Move to the storage position
                actionInProgress = true;
                lastActionTime = millis();
            }
            if (millis() - lastActionTime > 500) { // Wait for move to be "finished"
                currentState = OPEN_CLAW;
                actionInProgress = false;
            }
            break;

        case OPEN_CLAW:
            if (!actionInProgress) {
                setServoAngle(5, openClaw); // Open the claw
                actionInProgress = true;
                lastActionTime = millis();
            }
            if (millis() - lastActionTime > 650) {
                currentState = RETURN_TO_PICKUP;
                actionInProgress = false;
            }
            break;

        case RETURN_TO_PICKUP:
            currentState = WAIT_FOR_OBJECT;
            // The actionInProgress flag will be false, triggering the move in the next state.
            break;
    }
}