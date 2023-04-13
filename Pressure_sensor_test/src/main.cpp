#include <Arduino.h>
#define CONVERSION 50.0/1023.0
#define WIN_SIZE 10
const int pressurePinIndex = A0;
const int pressurePinMiddle = A1;
const unsigned long sampleTime = 100;

// Moving average filter variables
int index = 0;
float sum = 0.0;
float readings[WIN_SIZE];

float movAvgFilter(float &filterSum, float *window, int &index, const int winSize);

void setup() {
  Serial.begin(9600);
  pinMode(pressurePinIndex, INPUT);
}

unsigned long sampleStartTime = millis();

void loop() {
    if((millis() - sampleStartTime) >= sampleTime){
      float filterOutput = movAvgFilter(sum, readings, index, WIN_SIZE);
      Serial.print(String(filterOutput));
      Serial.print(",");
      sampleStartTime = millis();
    }
}

float movAvgFilter(float &filterSum, float *window, int &index, const int winSize){
  filterSum -= window[index];
  int pressureVal1 = analogRead(pressurePinIndex);
  int pressureVal2 = analogRead(pressurePinMiddle);
  float valueIndex = static_cast<float>(pressureVal1*CONVERSION);
  float valueMiddle = static_cast<float>(pressureVal2*CONVERSION);
  // Serial.print(String(valueIndex));
  // Serial.print(",");
  // Serial.print(String(valueMiddle));
  // Serial.println();
  float value = (valueIndex+valueMiddle)/2;
  window[index] = value;
  filterSum += value;
  index = (index+1) % winSize;
  return filterSum/winSize;
} 