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
int c = 0;
int rotationSpeed = 70; // Speed for rotation in degrees per second

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
  if(c==8){
    stopMotors();
    return;
  }
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
      if(c==0 && inter_count == 1)
        {
          interRotationUntilLineFound(rotationSpeed ,1);
        }
      if(c==1 && inter_count == 2)
      {
        interRotationUntilLineFound(rotationSpeed ,0);
      }
      if(c==2 && inter_count == 2)
      {
        interRotationUntilLineFound(rotationSpeed ,0);
      }
      if(c==3 && inter_count == 4)
      {
        interRotationUntilLineFound(rotationSpeed ,0);
      }
      if(c==4 && inter_count == 2)
      {
        interRotationUntilLineFound(rotationSpeed ,0);
      }
      if(c==5 && inter_count == 1)
      {
        interRotationUntilLineFound(rotationSpeed ,0);
      }
      if(c==6 && inter_count == 1)
      {
        interRotationUntilLineFound(rotationSpeed ,1);
      }
      if(c==7 && inter_count == 2)
      {
        c++;
      }
      break;

    default:
      break;
    }
  }


  void interRotationUntilLineFound(int rotationSpeed, bool dir)
  {
    if (!rotating)
    {
      stopMotors();
      rotating = true;
      actionDone = false;
    }

    if (rotating)
    {
      // keep rotating
      calculateMotors(0, 0, rotationSpeed* (dir ? -1 : 1));
      vTaskDelay(pdMS_TO_TICKS(10));
      // left the  original line and find the line back（statusLine == 0）
      if (sensors[(dir ? 7 : 0)] == 1)
      {
        stopMotors();
        rotating = false;
        actionDone = true;
        inter_count = 0;
        c++;
        return;
      }
    }
    else{
      // stopMotors();
      calculateMotors(0, 0, -rotationSpeed* (dir ? -1 : 1));
    }
  }