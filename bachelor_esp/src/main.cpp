#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>

//screen setup and naming
TFT_eSPI tft = TFT_eSPI();

// Sensorpin
const int pin = 32;

// Sensor value
int reading;

// Sensorvalues for filtering
int prevRead[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

// Not zero sensor
int proper;

// kg/newton converter
float kg = 0;

// timeline of measurments
int pixels[241];

// graph mode or newton mode
bool graphing = true;

// TTGO builtin buttons
int button1Pin = 0;
int button2Pin = 35;



void setup() {
  // put your setup code here, to run once:
  // Screen startup
  tft.init();
  tft.setRotation(3);
  tft.invertDisplay(true);
  tft.fillScreen(TFT_BLACK);

  pinMode(pin, INPUT);
  //Serial.begin(115200);

  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
}

void loop() {
  // put your main code here, to run repeatedly:
  reading = analogRead(pin);

  // clear display if last reading was not zero, and is now zero
  if (proper && reading == 0) {
    tft.fillScreen(TFT_BLACK);
  }

  // Change between graphing display or Newton display
  if (digitalRead(button1Pin) == 0 || digitalRead(button2Pin) == 0)  {
      delay(500);
      graphing = !graphing;
      Serial.println(graphing);
      delay(500);
    }


  // basic filtering
  for (int i = 11; i >= 0; i--) {
    prevRead[i+1] = prevRead[i];
    //Serial.print(prevRead[i]);
    //Serial.print(",  ");
  }
  prevRead[0] = reading;

  // add more of the commented statements to compensate for noisy zero readings
  if (prevRead[0]  != 0) { //  && prevRead[1] && prevRead[2] && prevRead[3] && prevRead[4] && prevRead[5] && prevRead[6] && prevRead[7] && prevRead[8] && prevRead[9] && prevRead[10]
    proper = true;
  }
  else{
    proper = false;
  }


  // tft display
  tft.setTextSize(5);
  tft.setTextColor(TFT_WHITE,TFT_BLACK);

  // Newton display
  if (graphing)  {
    kg = (reading*500);
    kg = kg/4095;
    if (proper) {
      //    flicker protection
      if (prevRead[1] >= 1000 && prevRead[0] < 1000)  {
        tft.fillScreen(TFT_BLACK);
        tft.drawFloat(kg,2,10,5);
      }
      else if (prevRead[1] >= 100 && prevRead[0] < 100)  {
        tft.fillScreen(TFT_BLACK);
        tft.drawFloat(kg,2,10,5);
      }
      else if (prevRead[1] >= 10 && prevRead[0] < 10)  {
        tft.fillScreen(TFT_BLACK);
        tft.drawFloat(kg,2,10,5);
      }
      else{
        tft.drawFloat(kg,2,10,5);
      }
    }
    else{
      tft.drawFloat(0,2,10,5);
    }

    tft.drawString("Newton",2,70);

  
  }
  
  
  // grafing
  else {
    // current -> previous
    for (int i = 240; i >= 1; i--)  {
      pixels[i] = pixels[i-1];
    }
    if (proper) {
      pixels[0] = map(reading, 0, 2000,130,1);
    }
    else  {
      pixels[0] = 130;
    }

    // plotting the green line
    for (int i = 0; i < 240; i++)  {
      tft.drawLine(i,pixels[i],i,pixels[i+1],TFT_GREENYELLOW);
    }

    // not using fillscreen to avoid flickering
    // blacking out the rest of the display to avoid flickering
    for (int i = 0; i < 240; i++)  {
      if (pixels[i] < pixels[i+1])  {
        tft.drawLine(i,0,i,pixels[i]-2,TFT_BLACK);
        tft.drawLine(i,134,i,pixels[i+1]+1,TFT_BLACK);
      }
      else{
        tft.drawLine(i,0,i,pixels[i+1]-1,TFT_BLACK);
        tft.drawLine(i,134,i,pixels[i]+1,TFT_BLACK);
      }
    }
  }
  delay(1);
  

}