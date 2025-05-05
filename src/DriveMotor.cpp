#include "DriveMotor.h"

MotorState motors[4] = {
    {true, 0, 0, M1_Forward, M1_Reverse},  // M1: Front-Right
    {true, 0, 0, M2_Forward, M2_Reverse},  // M2: Front-Left
    {true, 0, 0, M3_Forward, M3_Reverse},  // M3: Rear-Right 
    {true, 0, 0, M4_Forward, M4_Reverse}   // M4: Rear-Left
};

void MotorDriving(void *pvParameters) {
    // setupAuxMotor();
    while(true) {
        // Read channels with CORRECT inversion, this is used in RF control
        int throttle = scaleChannel(channelValues[1], false);  //  True invert the channel 
        int strafe = scaleChannel(channelValues[0], false);     // Dont Invert rotation (strafe and rotation are changed)
        int rotation = scaleChannel(channelValues[3], true);   // Invert strafe

        // Immediate stop detection
        if(abs(throttle) + abs(strafe) + abs(rotation) < stopThreshold) {
            stopMotors();
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }

        // Mecanum calculations (keep original signs)
        motors[0].targetSpeed = throttle + strafe + rotation;  // FR
        motors[1].targetSpeed = throttle - strafe + rotation;  // FL
        motors[2].targetSpeed = throttle + strafe - rotation;  // RR
        motors[3].targetSpeed = throttle - strafe - rotation;  // RL

        // Constrain and update motors
        for(int i=0; i<4; i++) {
            motors[i].targetSpeed = constrain(motors[i].targetSpeed, -speedlim, speedlim);
            updateMotor(motors[i]);
        }

        // // Add intake motor
        // updateAuxMotor();

        vTaskDelay(taskMotorDriving.getIntervalms() / portTICK_PERIOD_MS);
    }
}

int scaleChannel(int channelValue, bool invert) {
    const int neutral = 1500;
    const int effectiveMin = 1000;
    const int effectiveMax = 2000;

    // Apply deadzone
    if(abs(channelValue - neutral) < deadzone) return 0;

    // Split into two ranges: 1000-1500 and 1500-2000
    if(channelValue < neutral) {
        // Reverse range (1000-1500)
        int scaled = map(channelValue, 
                       effectiveMin, 
                       neutral - deadzone, 
                       speedlim, 
                       0);
        return invert ? scaled : -scaled;
    }
    else {
        // Forward range (1500-2000)
        int scaled = map(channelValue, 
                       neutral + deadzone, 
                       effectiveMax, 
                       0, 
                       speedlim);
        return invert ? -scaled : scaled;
    }
}

void updateMotor(MotorState &m) {
    bool newDir = m.targetSpeed >= 0;
    int absTarget = abs(m.targetSpeed);

    // Direction change handling
    if(newDir != m.currentDir) {
        analogWrite(m.forwardPin, 0);
        analogWrite(m.reversePin, 0);
        m.currentSpeed = 0;
        m.currentDir = newDir;
        vTaskDelay(motorProt / portTICK_PERIOD_MS);
    }

    // Aggressive ramping for stopping
    int step = (absTarget > m.currentSpeed) ? 
        rampStep :  // Acceleration
        -rampStep * 4;  // Faster deceleration

    m.currentSpeed = constrain(m.currentSpeed + step, 0, speedlim);

    // Apply to motor
    int activePin = m.currentDir ? m.forwardPin : m.reversePin;
    analogWrite(activePin, m.currentSpeed);
    analogWrite((m.currentDir ? m.reversePin : m.forwardPin), 0);
}

void stopMotors() {
    for(int i=0; i<4; i++) {
        motors[i].targetSpeed = 0;
        motors[i].currentSpeed = 0;
        analogWrite(motors[i].forwardPin, 0);
        analogWrite(motors[i].reversePin, 0);
    }
}


//////////////////////////////////////////////////
volatile uint8_t auxDuty = 0;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Hardware Timer (using hw_timer_t from Arduino-ESP32 core)
hw_timer_t *timer = NULL;

void IRAM_ATTR auxPWMUpdate() {
  static bool level = false;
  static uint8_t currentDuty = 0;
  
  portENTER_CRITICAL_ISR(&timerMux);
  uint8_t target = auxDuty;
  portEXIT_CRITICAL_ISR(&timerMux);

  if(target > 0) {
    if(level) {
      digitalWrite(AUX_PIN, LOW);
      timerAlarmWrite(timer, (255 - target) * 40, true);
    } else {
      digitalWrite(AUX_PIN, HIGH);
      timerAlarmWrite(timer, target * 40, true);
    }
    level = !level;
  } else {
    digitalWrite(AUX_PIN, LOW);
  }
}

void setupAuxMotor() {
  pinMode(AUX_PIN, OUTPUT);
  
  timer = timerBegin(0, 80, true);  // 1MHz clock (80MHz/80)
  timerAttachInterrupt(timer, &auxPWMUpdate, true);
  timerAlarmWrite(timer, 10000, true);  // 100Hz base frequency
  timerAlarmEnable(timer);
}

void updateAuxMotor() {
  static uint8_t targetDuty = 0;
  const uint8_t rampStep = 3;
  
  // Safety check
  if(channelValues[4] <= 1500) {
    portENTER_CRITICAL(&timerMux);
    auxDuty = 0;
    portEXIT_CRITICAL(&timerMux);
    return;
  }

  // Scale input
  uint8_t newTarget = map(constrain(channelValues[2], 1000, 2000), 1000, 2000, 0, 255);

  // Ramping
  if(newTarget > targetDuty) {
    targetDuty = (uint8_t)min((int)targetDuty + rampStep, (int)newTarget);
  } else if(newTarget < targetDuty) {
    targetDuty = (uint8_t)max((int)targetDuty - (rampStep * 2), 0);
  }

  portENTER_CRITICAL(&timerMux);
  auxDuty = targetDuty;
  portEXIT_CRITICAL(&timerMux);
}