#include "LineFol.h"
#include "DriveMotor.h"
#include "LinePosition.h"

// FreeRTOS timing constants
static TickType_t xLastRequestTime = 0;
const TickType_t xResponseTimeout = pdMS_TO_TICKS(10); // keep it bigger than taskLineFSense.getIntervalms()
const TickType_t xRequestInterval = pdMS_TO_TICKS(10); // For these example i used *5 and *10 respectively

HardwareSerial SerialLine(1);
uint8_t line_data[3];

bool inIntersection = false;

void LineSense(void *pvParameters)
{
    SerialLine.begin(9600, SERIAL_8N1, lineRX, lineTX); // Fixed baud rate to 9600 for this model
    Serial.begin(115200);
    Serial.println("Sensor Monitoring Started");

    sendRequest(); // Initial request

    while (true)
    {
        processSensorData();

        vTaskDelay(taskLineSense.getIntervalms() / portTICK_PERIOD_MS);
    }
}

void sendRequest()
{
    SerialLine.write(0x57);
    xLastRequestTime = xTaskGetTickCount();
}

// int calculatePosition(uint8_t status)
// {
//     // Convert status byte to sensor activations (0 = active)
//     bool sensors[8];
//     for (int i = 0; i < 8; i++)
//     {
//         sensors[i] = !((status >> (7 - i)) & 0x01); // Bit 7 = sensor 0 (leftmost)
//     }

//     // Calculate weighted average
//     float sum = 0;
//     int count = 0;

//     for (int i = 0; i < 8; i++)
//     {
//         if (sensors[i])
//         {
//             sum += i;
//             count++;
//         }
//     }

//     position = (sum / count) * 1000;

//     if (actionDone)
//     {
//         if (count == 0)
//         {
//             statusLine = 1; // Handle no line detected (turn right possibly)
//             lost_count++;
//             return position = 0;
//         }
//         else if (count >= 5)
//         {
//             statusLine = 2; // Handle intersection

//             if (!inIntersection)
//             {
//                 inter_count++;
//                 inIntersection = true;
//             }
//         }
//         else
//         {
//             statusLine = 0; // Normal situation compute PID
//             inIntersection = false;
//             lost_count = 0;
//         }
//     }

//     // Serial.println(inter_count);

//     // Return scaled position 0-7000
//     return position;
// }

int calculatePosition(uint8_t status)
{
    // Convert status byte to sensor activations (0 = active)
    bool sensors[8];
    for (int i = 0; i < 8; i++)
    {
        sensors[i] = !((status >> (7 - i)) & 0x01); // Bit 7 = sensor 0 (leftmost)
    }

    // Calculate weighted average
    float sum = 0;
    int count = 0;

    for (int i = 0; i < 8; i++)
    {
        if (sensors[i])
        {
            sum += i;
            count++;
        }
    }

    // Return scaled position
    position = (sum / count) * 1000;
    if (count == 0)
    {
        statusLine = 1; // No line
        lost_count++;
        position = 0;
    }
    else if (count >= 5)
    {
        statusLine = 2; // Intersection
        if (!inIntersection)
        {
            inter_count++;
            inIntersection = true;
        }
    }
    else
    {
        statusLine = 0; // Normal line
        inIntersection = false;
        lost_count = 0;
    }

    return position;
}

void processSensorData()
{
    TickType_t xCurrentTime = xTaskGetTickCount();

    // Handle timeout
    if ((xCurrentTime - xLastRequestTime) > xResponseTimeout)
    {
        while (SerialLine.available() > 0)
            SerialLine.read();
        sendRequest();
        return;
    }

    // Wait for complete packet
    if (SerialLine.available())
    {
        line_data[0] = SerialLine.read();
        line_data[1] = SerialLine.read();
        line_data[2] = SerialLine.read();

        // Process data
        calculatePosition(line_data[0]);

        // Debug output (consider moving to separate task)
        // Serial.print("Status: 0b");
        // for(int i=7; i>=0; i--) Serial.print((line_data[0] >> i) & 0x01);
        // Serial.print(" | Position: ");
        // Serial.println(position); // Right 0, Left 7000

        // Rate-limited requests
        if ((xTaskGetTickCount() - xLastRequestTime) >= xRequestInterval)
        {
            sendRequest();
        }
    }
}
