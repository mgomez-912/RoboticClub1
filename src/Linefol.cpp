
#include "LineFol.h"

// FreeRTOS timing constants
static TickType_t xLastRequestTime = 0;
const TickType_t xResponseTimeout = pdMS_TO_TICKS(50);    //keep it bigger than taskLineFollow.getIntervalms()
const TickType_t xRequestInterval = pdMS_TO_TICKS(100);   //For these example i used *5 and *10 respectively

HardwareSerial SerialLine(1);
uint8_t Patrol_data[3];
int position = 3500;

//////////////////////////////////////////
// PID Variables
float linePosition;
float rotationOutput = 0.0;
float setpoint = 3500.0;  // Center position
QuickPID linePID(&linePosition, &rotationOutput, &setpoint);

void LineFollow(void *pvParameters) {
    SerialLine.begin(9600, SERIAL_8N1, lineRX, lineTX); // Fixed baud rate to 9600 for this model
    Serial.begin(115200);
    Serial.println("Sensor Monitoring Started");
    
    initPIDController();
    sendRequest();      // Initial request

    while(true) {
        processSensorData();

        // PID Update
        portENTER_CRITICAL(&pidMux);
        linePID.Compute();
        portEXIT_CRITICAL(&pidMux);

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
    statusLine = 1;            // Handle no line detected (turn right)
    position = 3500;
    actionsPID(0);
    return position = 3500;
  } 
  else if(count == 8) statusLine = 2;       // Handle intersection
  else{
    statusLine=0;                           // Normal situation compute PID
    actionsPID(statusLine);
  }                    

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
        Patrol_data[0] = SerialLine.read();
        Patrol_data[1] = SerialLine.read();
        Patrol_data[2] = SerialLine.read();

        // Process data
        calculatePosition(Patrol_data[0]);
        
        // Debug output (consider moving to separate task)
        // Serial.print("Status: 0b");
        // for(int i=7; i>=0; i--) Serial.print((Patrol_data[0] >> i) & 0x01);
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

void actionsPID(int status){
  switch (status)
  {
  case 0:
    //////////////////// PID 
    portENTER_CRITICAL(&pidMux);
    linePosition = position;
    portEXIT_CRITICAL(&pidMux);
    ////////////////////
    break;
  
  default:
    break;
  }

}