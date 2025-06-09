// #include "Ultrasound.h"

// const int trigPin = 47;
// const int echoPin = 48;

// float duration, distance;
// long scaledDistance, mappedValue;

// void distanceUltra(void *pvParameters) {
//     pinMode(trigPin, OUTPUT);
//     pinMode(echoPin, INPUT);
//     Serial.begin(115200);
    
//     while(true) {
//         digitalWrite(trigPin, LOW);
//         delayMicroseconds(2);
//         digitalWrite(trigPin, HIGH);
//         delayMicroseconds(10);
//         digitalWrite(trigPin, LOW);

//         duration = pulseIn(echoPin, HIGH);
//         distance = (duration*.0343)/2;
//         Serial.print("Distance: ");
//         Serial.println(distance);

//         scaledDistance = (long)(distance * 100);
//         mappedValue = map(scaledDistance, 500, 3000, 100, 0); // 5cm to 30cm → 100 to 0
//         mappedValue = constrain(mappedValue, 0, 100);
//         vTaskDelay(taskRXRead.getIntervalms() / portTICK_PERIOD_MS);  // Short delay to limit serial output rate
//     }
// }