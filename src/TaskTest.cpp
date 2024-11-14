#include "taskTest.h"

void taskTest(void *pvParameters) {
    pinMode(test, OUTPUT);
    
    while(true){
        digitalWrite(test, HIGH);
        delay(500);
        digitalWrite(test, LOW);
        delay(500);

        vTaskDelay(taskTesting.getIntervalms() / portTICK_PERIOD_MS);  // Delay for the blink time
    }

}