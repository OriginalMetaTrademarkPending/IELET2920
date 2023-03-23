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
int i = 0;

// graph mode or newton mode
bool graphing = false;

// TTGO builtin buttons
int button1Pin = 0;
int button2Pin = 35;

const int scales[3] = {130, 2000, 4095};

int picker = 0;


// Kalman testing
float change = 0.;            // expected change between each measurment (model)
float process_var = 1.;       // variance for process model
float measurment_std = 4.;    // standard deviation in measurments
int measurment;

float prior[2];
float x[] = {0.,100.}; // inital state

float process_model[] = {change,process_var};
float gaus_mes[] = {float(reading),measurment_std*measurment_std};



float update(float prior[2], float measurment[2], bool returner)  {
  float x = prior[0];         // mean and variance of prior
  float P = prior[1];

  float z = measurment[0];    // mean and variance of measurment
  float R = measurment[1];    

  float y = z - x;            // residual
  float K = P / (P + R);      // Kalman gain

  x = x + K*y;                // posterior
  P = (1-K) * P;              // posterior variance

  float gaus;

  if (returner) {
    gaus = x;
  }
  else{
    gaus = P;
  }
  
  return gaus;
}

float predict(float posterior[2], float movement[2], bool returner)  {
  float x = posterior[0];     // mean and variance of posterior
  float P = posterior[1];

  float dx = movement[0];     // mean and variance of movement
  float Q = movement[1];

  x = x + dx;
  P = P + Q;

  float gaus;

  if (returner) {
    gaus = x;
  }
  else{
    gaus = P;
  }
  
  return gaus;
}



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


  // square process var
  process_var = process_var * process_var;
  float process_model[2] = {change,process_var};
}

void loop() {
  // put your main code here, to run repeatedly:
  reading = analogRead(pin);

  // clear display if last reading was not zero, and is now zero
  if (proper && reading == 0 && graphing) {
    tft.fillScreen(TFT_BLACK);
  }

  // Change between graphing display or Newton display
  if (digitalRead(button1Pin) == 0)  {
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
  
  // kalman try
  gaus_mes[0] = float(reading);

  prior[0] = predict(x,process_model,true);
  prior[1] = predict(x,process_model,false);
  x[0] = update(prior,gaus_mes,true);
  x[1] = update(prior,gaus_mes,false);
  
  measurment = int(round(x[0]));

  

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
    if (digitalRead(button2Pin) == 0) {
      picker++;
      while (digitalRead(button2Pin) == 0)  {}
      i = 0;
      tft.fillScreen(TFT_BLACK);
      if (picker >= 3) {
        picker = 0;
      }
    }
    tft.setTextSize(1);
    i++;
    if (i > 241)  {
      i = 0;
      tft.fillScreen(TFT_BLACK);
    }
    //tft.drawString("current timer: " + String(i),0,0);
    tft.drawString("Scale: " + String(scales[picker]),155,0);
    tft.fillCircle(145,15,2,TFT_RED);
    tft.drawString("reading: " + String(reading) + "                ",150,10);
    tft.fillCircle(145,25,2,TFT_GREENYELLOW);
    tft.drawString("fitlter: " + String(measurment) + "             ",150,20);
    tft.setTextSize(1);
    pixels[3] = pixels[2];
    pixels[2] = map(reading, 0, scales[picker],130,1);
    tft.drawLine(i,pixels[2],i,pixels[3],TFT_RED);
    pixels[1] = pixels[0];
    pixels[0] = map(measurment, 0, scales[picker],130,1);
    tft.drawLine(i,pixels[0],i,pixels[1],TFT_GREENYELLOW);
    

  }
  
  delay(25);
  

}