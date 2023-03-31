#include <Arduino.h>

const int pressurePin = A0;
int pressureVal;
const unsigned long sampleTime = 100;
float kgVal = 0.0;

void setup() {
  Serial.begin(9600);
  pinMode(pressurePin, INPUT);
}

unsigned long sampleStartTime = millis();

void loop() {
    if((millis() - sampleStartTime) >= sampleTime){
      pressureVal = analogRead(pressurePin);
      kgVal = static_cast<float>(pressureVal*50.0/1023);
      Serial.print(String(kgVal));
      Serial.print(",");
      sampleStartTime = millis();
    }
}