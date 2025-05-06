
#include "LineFol.h"
#include "DriveMotor.h"

// FreeRTOS timing constants
static TickType_t xLastRequestTime = 0;
const TickType_t xResponseTimeout = pdMS_TO_TICKS(15);    //keep it bigger than taskLineFollow.getIntervalms()
const TickType_t xRequestInterval = pdMS_TO_TICKS(30);   //For these example i used *5 and *10 respectively

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

  if(count == 0){
    statusLine = 1;            // Handle no line detected (turn right possibly)
    lost_count ++;
    return position = 3500;
  } 
  else if(count >= 5) {
    statusLine = 2;                         // Handle intersection
    // inter_count ++;

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
  Serial.println(inter_count);                    

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
    //////////////////// PID 
    portENTER_CRITICAL(&pidMux);
    linePID.SetTunings(Kp, Ki, Kd);
    linePosition = position;
    linePID.Compute();
    portEXIT_CRITICAL(&pidMux);
    break;
  
  case 1:                         //Handle no line scenario
    portENTER_CRITICAL(&pidMux);
    // linePID.SetTunings(Kp_lost,Ki_lost,Kd_lost);
    linePID.Compute();
    portEXIT_CRITICAL(&pidMux);
    // stopMotors();
    break;

  case 2:                         //Handle intersection scenario
    if(inter_count == 3){
      inter_count = 0;
    }

    break;
  
  default:
    break;
  }

}

void brakeCorrection (int time, int correctionMag){
  TickType_t now = xTaskGetTickCount();
      if (now - slideTime >= pdMS_TO_TICKS(time)) {
        slideTime=now;
      }

}