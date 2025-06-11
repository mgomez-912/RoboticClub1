#include "DriveMotor.h"
#include "LineFol.h"
#include "LinePosition.h"

MotorState motors[4] = {
    {true, 0, 0, M1_Forward, M1_Reverse}, // M1: Front-Right
    {true, 0, 0, M2_Forward, M2_Reverse}, // M2: Front-Left
    {true, 0, 0, M3_Forward, M3_Reverse}, // M3: Rear-Right
    {true, 0, 0, M4_Forward, M4_Reverse}  // M4: Rear-Left
};

// int lost_count = 0;

void MotorDriving(void *pvParameters)
{
    stopMotors();

    while (true)
    {
        // // Read channels with CORRECT inversion, this is used in RF control
        // int throttle = scaleChannel(channelValues[1], false);  //  True invert the channel
        // int strafe = scaleChannel(channelValues[3], true);     // Dont Invert rotation
        // int rotation = scaleChannel(channelValues[0], false);   // Invert strafe

        // // Immediate stop detection
        // if(abs(throttle) + abs(strafe) + abs(rotation) < stopThreshold) {
        //     stopMotors();
        //     vTaskDelay(10 / portTICK_PERIOD_MS);
        //     continue;
        // }
        // calculateMotors(throttle,strafe,rotation);

        // Get PID-computed rotation
        if (!actionDone)
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }
        float rotation;
        portENTER_CRITICAL(&pidMux);
        rotation = rotationOutput;
        portEXIT_CRITICAL(&pidMux);
        // Serial.println("rotation: ");
        // Serial.print(rotation);

        // Dynamic throttle calculation
        int throttle = speedlim - abs(rotation) * 0.5; // Reserve rotation space

        // Calculate motor outputs
        if (statusLine == 1)
        { // Handling no line
            if (lost_count >= lostCycles)
            {
                stopMotors();
                vTaskDelay(10 / portTICK_PERIOD_MS);
                continue;
            }
            calculateMotors(throttle, 0, rotation); // strafe=0
        }
        else
        {
            calculateMotors(throttle, 0, rotation); // strafe=0
        }

        vTaskDelay(taskMotorDriving.getIntervalms() / portTICK_PERIOD_MS);
    }
}

int scaleChannel(int channelValue, bool invert)
{
    const int neutral = 1500;
    const int effectiveMin = 1000;
    const int effectiveMax = 2000;

    // Apply deadzone
    if (abs(channelValue - neutral) < deadzone)
        return 0;

    // Split into two ranges: 1000-1500 and 1500-2000
    if (channelValue < neutral)
    {
        // Reverse range (1000-1500)
        int scaled = map(channelValue,
                         effectiveMin,
                         neutral - deadzone,
                         speedlim,
                         0);
        return invert ? scaled : -scaled;
    }
    else
    {
        // Forward range (1500-2000)
        int scaled = map(channelValue,
                         neutral + deadzone,
                         effectiveMax,
                         0,
                         speedlim);
        return invert ? -scaled : scaled;
    }
}

void updateMotor(MotorState &m)
{
    bool newDir = m.targetSpeed >= 0;
    int absTarget = abs(m.targetSpeed);

    // Direction change handling
    if (newDir != m.currentDir)
    {
        analogWrite(m.forwardPin, 0);
        analogWrite(m.reversePin, 0);
        m.currentSpeed = 0;
        m.currentDir = newDir;
        vTaskDelay(motorProt / portTICK_PERIOD_MS);
    }

    // Aggressive ramping for stopping
    int step = (absTarget > m.currentSpeed) ? rampStep : // Acceleration
                   -rampStep * 4;                        // Faster deceleration

    m.currentSpeed = constrain(m.currentSpeed + step, 0, speedlim);

    // Apply to motor
    int activePin = m.currentDir ? m.forwardPin : m.reversePin;
    analogWrite(activePin, m.currentSpeed);
    analogWrite((m.currentDir ? m.reversePin : m.forwardPin), 0);
}

void calculateMotors(int throttle, int strafe, int rotation)
{
    // Mecanum calculations (keep original signs)
    motors[0].targetSpeed = throttle + strafe + rotation; // FR
    motors[1].targetSpeed = throttle + strafe - rotation; // FL
    motors[2].targetSpeed = throttle - strafe + rotation; // RR
    motors[3].targetSpeed = throttle - strafe - rotation; // RL
                                                          //   Serial.println(motors[0].targetSpeed);

    // Constrain and update to motors
    for (int i = 0; i < 4; i++)
    {
        motors[i].targetSpeed = constrain(motors[i].targetSpeed, -255, 255);
        updateMotor(motors[i]);
    }
}

void stopMotors()
{
    for (int i = 0; i < 4; i++)
    {
        motors[i].targetSpeed = 0;
        motors[i].currentSpeed = 0;
        analogWrite(motors[i].forwardPin, 0);
        analogWrite(motors[i].reversePin, 0);
    }
}