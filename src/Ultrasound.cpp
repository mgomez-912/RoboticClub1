#include "Ultrasound.h"

const int trigPin = 13;
const int echoPin = 14;

float duration;

void distanceUltra(void *pvParameters) {
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    Serial.begin(115200);
    
    while(true) {
        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);

        duration = pulseIn(echoPin, HIGH);
        distance = (duration*.0343)/2;
        Serial.print("Distance: ");
        Serial.println(distance);
        vTaskDelay(taskUltrasound.getIntervalms() / portTICK_PERIOD_MS);  // Short delay to limit serial output rate
    }
}