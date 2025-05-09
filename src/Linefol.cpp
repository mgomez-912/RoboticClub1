
#include "LineFol.h"
#include "DriveMotor.h"
#include "LinePosition.h"

//////////////////////////////////////////
// PID Variables
float linePosition;
float rotationOutput = 0.0;
float setpoint = 3500.0;  // Center position
QuickPID linePID(&linePosition, &rotationOutput, &setpoint);

///////////////////////////////
static TickType_t now = 0;
static TickType_t slideTime = 0;
static TickType_t rotationTime = 0;
bool actionDone = true;
bool brakeDone = false;

void LineFollow(void *pvParameters) {
    SerialLine.begin(9600, SERIAL_8N1, lineRX, lineTX); // Fixed baud rate to 9600 for this model
    Serial.begin(115200);
    Serial.println("Sensor Monitoring Started");
    
    initPIDController();
    sendRequest();      // Initial request

    while(true) {
        processSensorData();
        actionsPID(statusLine);

        vTaskDelay(taskLineFollow.getIntervalms() / portTICK_PERIOD_MS);
    }
}

///////////////////////////////////////////
void initPIDController() {
    linePID.SetTunings(Kp, Ki, Kd);
    linePID.SetMode(linePID.Control::automatic);
    linePID.SetOutputLimits(-outputLimit, outputLimit);
  }

void actionsPID(int status){      //Navigation function
  switch (status)
  {
  case 0:                         //Following line, normal scenario
  // Serial.println("Case0");
    //////////////////// PID 
    portENTER_CRITICAL(&pidMux);
    linePID.SetTunings(Kp, Ki, Kd);
    linePosition = position;
    linePID.Compute();
    portEXIT_CRITICAL(&pidMux);
    break;
  
  case 1:                         //Handle no line scenario
  // Serial.println("Case1");
    portENTER_CRITICAL(&pidMux);
    linePID.SetTunings(Kp_lost,Ki_lost,Kd_lost);
    linePID.Compute();
    portEXIT_CRITICAL(&pidMux);
    // stopMotors();
    break;

  case 2:                         //Handle intersection scenario
  // Serial.println(inter_count);
    if(inter_count == 3){
      actionDone=false;
      brakeCorrection(225, 65);
      if(brakeDone) interRotation(950,100,1);
      // interRotation(750,100,1);
      if(actionDone) {
        inter_count = 0;
      }
    }
    break;
  
  default:
    break;
  }

}

// Speed correection for rotations or other decision (speedlim=55; time=150, mag=50 works well)
void brakeCorrection(int time, int correctionMag) {
  // these static locals keep state between calls
  static TickType_t  startTick = 0;
  static bool        braking   = false;

  TickType_t now = xTaskGetTickCount();

  if (!braking) {
      // --- start the braking interval ---
      startTick  = now;
      braking    = true;
      brakeDone = false;
  }

  // --- actively keep braking until time elapses ---
  if (braking) {
      calculateMotors(-correctionMag, 0, 0);

      if ((now - startTick) >= pdMS_TO_TICKS(time)) {
          braking    = false;
          brakeDone = true;
      }
  }
}

// 90 degrees rotation for intersections, 0 left, 1 right
void interRotation (int time, int rotationMag, bool dir){
  // these static locals keep state between calls
  static TickType_t  startTick = 0;
  static bool        rotating   = false;

  TickType_t now = xTaskGetTickCount();

  if (!rotating) {
    startTick  = now;
    stopMotors();
    rotating    = true;
    actionDone = false;
  }
  if (rotating) {
    calculateMotors(50, 0, rotationMag * (dir?1:-1)); // try speedlim/2 in throttle
  
    if ((now - startTick) >= pdMS_TO_TICKS(time)) {
        rotating    = false;
        actionDone = true;
    }
  }
}
