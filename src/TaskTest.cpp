#include "TaskTest.h"

void taskTest(void *pvParameters) {
    pinMode(test, OUTPUT);
    
    while(true){
        // digitalWrite(motorTest, HIGH);
        // delay(500);
        // digitalWrite(motorTest, LOW);
        // delay(500);
        
        stopMotors();
        analogWrite(M1F, 250);
        delay(1500);

        stopMotors();
        analogWrite(M1R, 250);
        delay(1500);

        // analogWrite(M2F, LOW);
        // analogWrite(M2R, LOW);
        
        vTaskDelay(taskTesting.getIntervalms() / portTICK_PERIOD_MS);  // Delay for the blink time
    }

}

void stopMotors() {
    analogWrite(M1F, LOW);
    analogWrite(M1R, LOW);
    analogWrite(M2F, LOW);
    analogWrite(M2R, LOW);
    delay(1500);
}
