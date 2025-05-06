
#include "LineFol.h"
#include "DriveMotor.h"

// FreeRTOS timing constants
static TickType_t xLastRequestTime = 0;
const TickType_t xResponseTimeout = pdMS_TO_TICKS(20);    //keep it bigger than taskLineFollow.getIntervalms()
const TickType_t xRequestInterval = pdMS_TO_TICKS(20);   //For these example i used *5 and *10 respectively

HardwareSerial SerialLine(1);
uint8_t line_data[3];

int position = 3500;
int lost_count = 0;
int inter_count = 0;
bool inIntersection = false;

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

void sendRequest() {
    SerialLine.write(0x57);
    xLastRequestTime = xTaskGetTickCount();
}

int calculatePosition(uint8_t status) {
  // Convert status byte to sensor activations (0 = active)
  bool sensors[8];
  for(int i=0; i<8; i++) {
    sensors[i] = !((status >> (7 - i)) & 0x01); // Bit 7 = sensor 0 (leftmost)
  }

  // Calculate weighted average
  float sum = 0;
  int count = 0;
  
  for(int i=0; i<8; i++) {
    if(sensors[i]) {
      sum += i;
      count++;
    }
  }

  position = (sum / count) * 1000;

  if(actionDone){
    if(count == 0){
      statusLine = 1;            // Handle no line detected (turn right possibly)
      lost_count ++;
      return position = 3500;
    }
    else if(count >= 5) {
      statusLine = 2;                         // Handle intersection
  
      if (!inIntersection) {
        inter_count++;
        inIntersection = true;
      }
      else {
        inIntersection = false;
      }
    }
    else{
      statusLine=0;                           // Normal situation compute PID
      lost_count = 0;
    }
  }
  
  // Serial.println(inter_count);                    

  // Return scaled position 0-7000
  return position;
}

void processSensorData() {
    TickType_t xCurrentTime = xTaskGetTickCount();
    
    // Handle timeout
    if((xCurrentTime - xLastRequestTime) > xResponseTimeout) {
        while(SerialLine.available() > 0) SerialLine.read();
        sendRequest();
        return;
    }

    // Wait for complete packet
    if(SerialLine.available()) {
        line_data[0] = SerialLine.read();
        line_data[1] = SerialLine.read();
        line_data[2] = SerialLine.read();

        // Process data
        calculatePosition(line_data[0]);
        
        // Debug output (consider moving to separate task)
        // Serial.print("Status: 0b");
        // for(int i=7; i>=0; i--) Serial.print((line_data[0] >> i) & 0x01);
        // Serial.print(" | Position: ");
        // Serial.println(position);

        // Rate-limited requests
        if((xTaskGetTickCount() - xLastRequestTime) >= xRequestInterval) {
            sendRequest();
        }
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
    // linePID.SetTunings(Kp_lost,Ki_lost,Kd_lost);
    linePID.Compute();
    portEXIT_CRITICAL(&pidMux);
    // stopMotors();
    break;

  case 2:                         //Handle intersection scenario
  Serial.println(inter_count);
    if(inter_count == 3){
      actionDone=false;
      // brakeCorrection(1000, 50);
      interRotation(500,50,1);
      if(actionDone) {
        inter_count = 0;
      }
    }
    break;
  
  default:
    break;
  }

}

void brakeCorrection(int time, int correctionMag) {
  // these static locals keep state between calls
  static TickType_t  startTick = 0;
  static bool        braking   = false;

  TickType_t now = xTaskGetTickCount();

  if (!braking) {
      // --- start the braking interval ---
      startTick  = now;
      braking    = true;
      actionDone = false;
  }

  // --- actively keep braking until time elapses ---
  if (braking) {
      calculateMotors(-correctionMag, 0, 0);

      if ((now - startTick) >= pdMS_TO_TICKS(time)) {
          // --- done ---
          braking    = false;
          actionDone = true;
      }
  }
}

void interRotation (int time, int rotationMag, bool dir){
  // these static locals keep state between calls
  static TickType_t  startTick = 0;
  static bool        rotating   = false;

  TickType_t now = xTaskGetTickCount();

  if (!rotating) {
    startTick  = now;
    rotating    = true;
    actionDone = false;
  }
  if (rotating) {
    calculateMotors(0, 0, rotationMag * (dir?1:-1)); // try speedlim/2 in throttle
  
    if ((now - startTick) >= pdMS_TO_TICKS(time)) {
        rotating    = false;
        actionDone = true;
    }
  }
}
