#include "DriveMotor.h"

bool isFirstRun = true;         // Tracks if this is the first run

// Helper variables
bool lastDirM1 = true;          // True: Forward, False: Reverse
bool lastDirM2 = true;
int currentSpeedM1 = 0;         // Current speed of Motor 1
int currentSpeedM2 = 0;         // Current speed of Motor 2

// State definitions
enum RobotState { IDLE, FORWARD, REVERSE, TURN_LEFT, TURN_RIGHT };
RobotState currentState = IDLE;
RobotState lastState = IDLE; // Track the previous state for transitions

// FreeRTOS tick-based timer
TickType_t idleStartTick = 0;
bool idleTimerRunning = false;                  // FreeRTOS timmer helper

void MotorDriving(void *pvParameters) {
    if (isFirstRun) {
        stopMotors(); // Ensure all motors are stopped
        isFirstRun = false; // Reset the first-run flag
        vTaskDelay(500 / portTICK_PERIOD_MS); // Optional delay to ensure stability
    }

    while (true) {
        // Read input signals (e.g., from receiver or other input source)
        int advSignal = channelValues[1]; // Forward/Reverse
        int turnSignal = channelValues[3]; // Left/Right

        // Update state based on input
        updateState(advSignal, turnSignal);

        // Execute actions for the current state
        executeState();

        // Small delay for stability
        vTaskDelay(taskMotorDriving.getIntervalms() / portTICK_PERIOD_MS);
    }
}


void updateState(int advSignal, int turnSignal) {
    // Determine current state based on input signals
    if (advSignal > minres && advSignal < maxres && turnSignal > minres && turnSignal < maxres) {
        currentState = IDLE;

        if (!idleTimerRunning) {                        // motor protection timer starts in deadband
            idleStartTick = xTaskGetTickCount();        // Record the tick count when IDLE starts
            idleTimerRunning = true;
        }
    } else {
        idleTimerRunning = false;
    }

    if (advSignal >= maxres) {
        currentState = FORWARD;
    } else if (advSignal <= minres) {
        currentState = REVERSE;
    } else if (turnSignal >= maxres) {
        currentState = TURN_RIGHT;
    } else if (turnSignal <= minres) {
        currentState = TURN_LEFT;
    }
}

void executeState() {
    switch (currentState) {
        case IDLE:
            stopMotors();
            break;
        case FORWARD:
            controlForward();
            break;
        case REVERSE:
            controlReverse();
            break;
        case TURN_RIGHT:
            setMotorDirection(true, map(channelValues[3], maxres, 2005, 0, speedlim*(0.5)),
                              false, map(channelValues[3], maxres, 2005, 0, speedlim*(0.5)));
            break;
        case TURN_LEFT:
            setMotorDirection(false, map(channelValues[3], minres, 995, 0, speedlim*(0.5)),
                              true, map(channelValues[3], minres, 995, 0, speedlim*(0.5)));
            break;
    }

    // Track the previous state for debugging or future use
    lastState = currentState;
}

void controlForward() {
    int baseSpeed = map(channelValues[1], maxres, 2005, 0, speedlim);
    int adjustment = 0;

    if (channelValues[3] > maxres) {
        adjustment = map(channelValues[3], maxres, 2005, 0, baseSpeed);
        setMotorDirection(true, baseSpeed, true, baseSpeed - adjustment);
    } else if (channelValues[3] < minres) {
        adjustment = map(channelValues[3], minres, 995, 0, baseSpeed);
        setMotorDirection(true, baseSpeed - adjustment, true, baseSpeed);
    } else {
        setMotorDirection(true, baseSpeed, true, baseSpeed);
    }
}

void controlReverse() {
    int baseSpeed = map(channelValues[1], minres, 995, 0, speedlim);
    int adjustment = 0;

    if (channelValues[3] > maxres) {
        adjustment = map(channelValues[3], maxres, 2005, 0, baseSpeed);
        setMotorDirection(false, baseSpeed, false, baseSpeed - adjustment);
    } else if (channelValues[3] < minres) {
        adjustment = map(channelValues[3], minres, 995, 0, baseSpeed);
        setMotorDirection(false, baseSpeed - adjustment, false, baseSpeed);
    } else {
        setMotorDirection(false, baseSpeed, false, baseSpeed);
    }
}

void stopMotors() {
    analogWrite(M1_Forward, LOW);
    analogWrite(M1_Reverse, LOW);
    analogWrite(M2_Forward, LOW);
    analogWrite(M2_Reverse, LOW);
    currentSpeedM1 = 0;
    currentSpeedM2 = 0;
}

void setMotorDirection(bool dirM1, int targetSpeedM1, bool dirM2, int targetSpeedM2) {
    // Stop both motors if either motor direction is changing
    if (dirM1 != lastDirM1 || dirM2 != lastDirM2) {
        stopMotors(); // Stop both motors
        vTaskDelay(motorProt / portTICK_PERIOD_MS); // Brief delay to ensure motors are fully stopped
    }

    // Update Motor 1
    rampSpeed(currentSpeedM1, targetSpeedM1, dirM1 ? M1_Forward : M1_Reverse, dirM1 ? M1_Reverse : M1_Forward);
    lastDirM1 = dirM1;

    // Update Motor 2
    rampSpeed(currentSpeedM2, targetSpeedM2, dirM2 ? M2_Forward : M2_Reverse, dirM2 ? M2_Reverse : M2_Forward);
    lastDirM2 = dirM2;
}

void rampSpeed(int &currentSpeed, int targetSpeed, int controlPinForward, int controlPinReverse) {
    while (currentSpeed != targetSpeed) {
        if (currentSpeed < targetSpeed) {
            currentSpeed += rampStep;
            if (currentSpeed > targetSpeed) currentSpeed = targetSpeed;
        } else {
            currentSpeed -= rampStep;
            if (currentSpeed < targetSpeed) currentSpeed = targetSpeed;
        }
        analogWrite(controlPinForward, currentSpeed);
        analogWrite(controlPinReverse, LOW);
        vTaskDelay(rampDelay / portTICK_PERIOD_MS);
    }
}