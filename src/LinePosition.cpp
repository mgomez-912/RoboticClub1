#include "LineFol.h"
#include "DriveMotor.h"
#include "LinePosition.h"

// FreeRTOS timing constants
static TickType_t xLastRequestTime = 0;
const TickType_t xResponseTimeout = pdMS_TO_TICKS(5); // keep it bigger than taskLineFSense.getIntervalms()
const TickType_t xRequestInterval = pdMS_TO_TICKS(5); // For these example i used *5 and *10 respectively

HardwareSerial SerialLine(1);
uint8_t line_data[3];

// Add these variables with your other global variables
const int NUM_READINGS = 3;
static float position_readings[NUM_READINGS];
static int read_index = 0;
static int total_readings = 0;

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


/**
 * @brief Calculates a moving average of position readings.
 * @param new_position The most recent position measurement to be added to the average.
 * @return The new averaged position.
 */
float getAveragePosition(float new_position)
{
    // Store the new reading in a circular buffer
    position_readings[read_index] = new_position;

    // Advance the index for the next reading, wrapping around if necessary
    read_index = (read_index + 1) % NUM_READINGS;

    // Keep track of how many readings have been stored, up to the maximum
    if (total_readings < NUM_READINGS)
    {
        total_readings++;
    }

    // Calculate the sum of the readings in the buffer
    float sum = 0;
    for (int i = 0; i < total_readings; i++)
    {
        sum += position_readings[i];
    }

    // Return the average
    return sum / total_readings;
}

int calculatePosition(uint8_t status)
{
    // Convert status byte to sensor activations (0 = active)
    for (int i = 0; i < 8; i++)
    {
        sensors[i] = !((status >> (7 - i)) & 0x01); // Bit 7 = sensor 0 (leftmost)
    }

    // Calculate weighted average for the CURRENT reading
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

    float current_position = 0;
    // Calculate a raw position only if sensors are detected
    if (count > 0)
    {
        current_position = (sum / count) * 1000;
    }

    // Get the running average of the last 3 readings and save it globally
    position = getAveragePosition(current_position);

    // Decision-making logic based on the CURRENT sensor state
    if (actionDone)
    {
        if (count == 0) // Lost the line
        {
            statusLine = 1; // Handle no line detected
            lost_count++;
            
            // When line is lost, you might want to reset the average buffer
            // and enforce a position of 0.
            total_readings = 0; // Reset average calculation
            read_index = 0;
            return position = 0; // Return 0 as per original logic
        }
        else if (count >= 5) // Intersection detected
        {
            statusLine = 2; // Handle intersection
            if (!inIntersection)
            {
                inter_count++;
                inIntersection = true;
            }
        }
        else // Normal line following
        {
            statusLine = 0; // Normal situation, compute PID
            inIntersection = false;
            lost_count = 0;
        }
    }

    // Return the final, smoothed position
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
