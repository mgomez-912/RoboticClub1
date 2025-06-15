#include "LineFol.h"
#include "DriveMotor.h"
#include "LinePosition.h"

//////////////////////////////////////////
// PID Variables
float linePosition;
float rotationOutput = 0.0;
float setpoint = 3500.0; // Center position
QuickPID linePID(&linePosition, &rotationOutput, &setpoint);

///////////////////////////////
static TickType_t now = 0;
static TickType_t slideTime = 0;
static TickType_t rotationTime = 0;
bool actionDone = true;
bool brakeDone = false;

static bool rotating = false;
static bool hasLeftLine = false;
static int intersectionPhase = 0;

void LineFollow(void *pvParameters)
{
  initPIDController();

  while (true)
  {
    actionsPID(statusLine);
    vTaskDelay(taskLineFollow.getIntervalms() / portTICK_PERIOD_MS);
  }
}

///////////////////////////////////////////
void initPIDController()
{
  linePID.SetTunings(Kp, Ki, Kd);
  linePID.SetMode(linePID.Control::automatic);
  linePID.SetOutputLimits(-outputLimit, outputLimit);
}

void actionsPID(int status)
{ // Navigation function
  Serial.println(status);
  switch (status)
  {
  case 0: // Following line, normal scenario
    // Serial.println("Case0");
      //////////////////// PID
      portENTER_CRITICAL(&pidMux);
      linePID.SetTunings(Kp, Ki, Kd);
      linePosition = position;
      linePID.Compute();
      portEXIT_CRITICAL(&pidMux);
      if (actionDone)
      {
        portENTER_CRITICAL(&pidMux);
        linePID.SetTunings(Kp, Ki, Kd);
        linePosition = position;
        linePID.Compute();
        portEXIT_CRITICAL(&pidMux);
      }
      break;

    case 1:                         //Handle no line scenario
    // Serial.println("Case1");
      if (actionDone)
      {
        portENTER_CRITICAL(&pidMux);
        linePID.SetTunings(Kp_lost,Ki_lost,Kd_lost);
        linePID.Compute();
        portEXIT_CRITICAL(&pidMux);
      // stopMotors();
      }
      break;

    case 2:                         //Handle intersection scenario
    // Serial.println(inter_count);
      if(inter_count == 1){
        interRotationUntilLineFound(40);
      }
      break;

    default:
      break;
    }
  }

  void brakeCorrection(int time, int correctionMag)
  {
    static TickType_t startTick = 0;
    static bool braking = false;
    TickType_t now = xTaskGetTickCount();

    if (!braking)
    {
      startTick = now;
      braking = true;
      brakeDone = false;
      Serial.println("BrakeCorrection started.");
    }

    if (braking)
    {
      calculateMotors(-correctionMag, 0, 0);
      if ((now - startTick) >= pdMS_TO_TICKS(time))
      {
        stopMotors();
        braking = false;
        brakeDone = true;
        Serial.println("BrakeCorrection finished.");
      }
    }
  }

  void interRotationUntilLineFound(int rotationSpeed)
  {
    if (!rotating)
    {
      stopMotors();
      rotating = true;
      hasLeftLine = false;
      actionDone = false;
    }

    if (rotating)
    {
      // keep rotating
      calculateMotors(0, 0, rotationSpeed);
      

      // still on the origial line（statusLine ！= 0）
      if (!hasLeftLine && statusLine == 1)
      {
        hasLeftLine = true;
        Serial.println("1");
      }

      // left the  original line and find the line back（statusLine == 0）
      if (hasLeftLine && statusLine == 0)
      {
        Serial.println("3");
        stopMotors();
        rotating = false;
        actionDone = true;
        inter_count = 0;
        return;
      }
    }
  }

  // //
  // --- The oringinal version ---
  // //

  // void actionsPID(int status){      //Navigation function
  //   switch (status)
  //   {
  //   case 0:                         //Following line, normal scenario
  //   // Serial.println("Case0");
  //     //////////////////// PID
  //     portENTER_CRITICAL(&pidMux);
  //     linePID.SetTunings(Kp, Ki, Kd);
  //     linePosition = position;
  //     linePID.Compute();
  //     portEXIT_CRITICAL(&pidMux);
  //     break;

  //   case 1:                         //Handle no line scenario
  //   // Serial.println("Case1");
  //     portENTER_CRITICAL(&pidMux);
  //     linePID.SetTunings(Kp_lost,Ki_lost,Kd_lost);
  //     linePID.Compute();
  //     portEXIT_CRITICAL(&pidMux);
  //     // stopMotors();
  //     break;

  //   case 2:                         //Handle intersection scenario
  //   // Serial.println(inter_count);
  //     if(inter_count == 3){
  //       actionDone=false;
  //       brakeCorrection(225, 65);
  //       if(brakeDone) interRotation(950,100,1);
  //       // interRotation(750,100,1);
  //       if(actionDone) {
  //         inter_count = 0;
  //       }
  //     }
  //     break;

  //   default:
  //     break;
  //   }

  // }

  // // Speed correction for rotations or other decision (speedlim=55; time=150, mag=50 works well)
  // void brakeCorrection(int time, int correctionMag) {
  //   // these static locals keep state between calls
  //   static TickType_t  startTick = 0;
  //   static bool        braking   = false;

  //   TickType_t now = xTaskGetTickCount();

  //   if (!braking) {
  //       // --- start the braking interval ---
  //       startTick  = now;
  //       braking    = true;
  //       brakeDone = false;
  //   }

  //   // --- actively keep braking until time elapses ---
  //   if (braking) {
  //       calculateMotors(-correctionMag, 0, 0);

  //       if ((now - startTick) >= pdMS_TO_TICKS(time)) {
  //           braking    = false;
  //           brakeDone = true;
  //       }
  //   }
  // }

  // // 90 degrees rotation for intersections, 0 left, 1 right
  // void interRotation (int time, int rotationMag, bool dir){
  //   // these static locals keep state between calls
  //   static TickType_t  startTick = 0;
  //   static bool        rotating   = false;

  //   TickType_t now = xTaskGetTickCount();

  // if (!rotating)
  // {
  //   startTick = now;
  //   stopMotors();
  //   rotating = true;
  //   actionDone = false;
  // }
  // if (rotating)
  // {
  //   calculateMotors(50, 0, rotationMag * (dir ? 1 : -1)); // try speedlim/2 in throttle

  //   if ((now - startTick) >= pdMS_TO_TICKS(time))
  //   {
  //     rotating = false;
  //     actionDone = true;
  //   }
  // }