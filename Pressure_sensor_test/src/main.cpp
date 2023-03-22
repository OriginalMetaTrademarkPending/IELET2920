#include <Arduino.h>

const int pressurePin = A0;
int pressureVal;
const unsigned long sampleTime = 1000;

void setup() {
  Serial.begin(9600);
  pinMode(pressurePin, INPUT);
}

unsigned long sampleStartTime = micros();

void loop() {
    if((micros() - sampleStartTime) > sampleTime){
      pressureVal = analogRead(pressurePin);
      Serial.print(pressureVal);
      Serial.print(",");
      sampleStartTime = micros();
    }
}