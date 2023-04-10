#include <Arduino.h>
#define CONVERSION 50.0/1023.0
const int pressurePinIndex = A0;
const int pressurePinMiddle = A1;
int pressureVal1;
int pressureVal2;
const unsigned long sampleTime = 100;
float kgVal1 = 0.0;
float kgVal2 = 0.0;

void setup() {
  Serial.begin(9600);
  pinMode(pressurePinIndex, INPUT);
  pinMode(pressurePinMiddle, INPUT);
}

unsigned long sampleStartTime = millis();

void loop() {
    if((millis() - sampleStartTime) >= sampleTime){
      pressureVal1 = analogRead(pressurePinIndex);
      pressureVal2 = analogRead(pressurePinMiddle);
      kgVal1 = static_cast<float>(pressureVal1*CONVERSION);
      kgVal2 = static_cast<float>(pressureVal2*CONVERSION);
      Serial.print(String(kgVal1));
      Serial.print(",");
      Serial.print(String(kgVal2));
      Serial.print("nl");
      sampleStartTime = millis();
    }
}