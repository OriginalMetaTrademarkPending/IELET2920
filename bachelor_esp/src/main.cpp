#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>

//screen setup and naming
TFT_eSPI tft = TFT_eSPI();

// Sensorpin
const int bottomSensorPin = 32;
const int bottomMiddleSensorPin = 33;
const int topMiddleSensorPin = 25;
const int topSensorPin = 26;

// Sensor value
int bottomReading;
int bottomMidReading;
int topMidReading;
int topReading;

// Sensorvalues for filtering
int prevBotRead[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int prevBotMidRead[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int prevTopMidRead[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int prevTopRead[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


// kg/newton converter
float kg = 0;


// timeline of measurments
int botPixels[4];
int botMidPixels[4];
int topMidPixels[4];
int topPixels[4];
int i = 0;

// graph mode or newton mode
bool graphing = false;

// TTGO builtin buttons
int button1Pin = 0;
int button2Pin = 35;

const int scales[3] = {130, 2000, 4095};

int picker = 0;


int proper = 0;

// Kalman testing
float change = 0.;            // expected change between each measurment (model)
float process_var = 1.;       // variance for process model
float measurment_std = 4.;    // standard deviation in measurments
int botMeasurment;
int botMidMeasurment;
int topMidMeasurment;
int topMeasurment;

float prior[2];
float priorBot[2];
float priorBotMid[2];
float priorTopMid[2];
float priorTop[2];
float xBot[] = {0.,100.}; // inital state
float xBotMid[] = {0.,100.}; // inital state
float xTopMid[] = {0.,100.}; // inital state
float xTop[] = {0.,100.}; // inital state

float process_model[] = {change,process_var};
float gaus_mes[] = {float(bottomReading),measurment_std*measurment_std};



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

  pinMode(bottomSensorPin, INPUT);
  pinMode(bottomMiddleSensorPin, INPUT);
  pinMode(topMiddleSensorPin, INPUT);
  pinMode(topSensorPin, INPUT);
  Serial.begin(115200);

  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);


  // square process var
  process_var = process_var * process_var;
  float process_model[2] = {change,process_var};
}

void loop() {
  // put your main code here, to run repeatedly:
  bottomReading = analogRead(bottomSensorPin);
  bottomMidReading = analogRead(bottomMiddleSensorPin);
  topMidReading = analogRead(topMiddleSensorPin);
  topReading = analogRead(topSensorPin);
  Serial.println("bottom: " + String(bottomReading));
  Serial.println("botmid: " + String(bottomMidReading));
  Serial.println("topmid: " + String(topMidReading));
  Serial.println("top: " + String(topReading));
  

  // clear display if last reading was not zero, and is now zero
  if (proper && bottomReading && bottomMidReading && topMidReading && topReading== 0 && graphing) {
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
    prevBotRead[i+1] = prevBotRead[i];
    prevBotMidRead[i+1] = prevBotMidRead[i];
    prevTopMidRead[i+1] = prevTopMidRead[i];
    prevTopRead[i+1] = prevTopRead[i];
    //Serial.print(prevRead[i]);
    //Serial.print(",  ");
  }
  prevBotRead[0] = bottomReading;
  prevBotMidRead[0] = bottomMidReading;
  prevTopMidRead[0] = topMidReading;
  prevTopRead[0] = topReading;

  // add more of the commented statements to compensate for noisy zero readings
  if (prevBotRead[0] && prevBotMidRead[0] && prevTopMidRead[0] && prevTopRead[0] != 0) { //  && prevRead[1] && prevRead[2] && prevRead[3] && prevRead[4] && prevRead[5] && prevRead[6] && prevRead[7] && prevRead[8] && prevRead[9] && prevRead[10]
    proper = true;
  }
  else{
    proper = false;
  }
  
  // kalman try
  //  bottom sensor
  gaus_mes[0] = float(bottomReading);

  prior[0] = predict(xBot,process_model,true);
  prior[1] = predict(xBot,process_model,false);
  xBot[0] = update(prior,gaus_mes,true);
  xBot[1] = update(prior,gaus_mes,false);
  
  botMeasurment = int(round(xBot[0]));

  //  bottom  middle sensor
  gaus_mes[0] = float(bottomMidReading);

  prior[0] = predict(xBotMid,process_model,true);
  prior[1] = predict(xBotMid,process_model,false);
  xBotMid[0] = update(prior,gaus_mes,true);
  xBotMid[1] = update(prior,gaus_mes,false);
  
  botMidMeasurment = int(round(xBotMid[0]));

  //  top  middle sensor
  gaus_mes[0] = float(topMidReading);

  prior[0] = predict(xTopMid,process_model,true);
  prior[1] = predict(xTopMid,process_model,false);
  xTopMid[0] = update(prior,gaus_mes,true);
  xTopMid[1] = update(prior,gaus_mes,false);
  
  topMidMeasurment = int(round(xTopMid[0]));


  //  top sensor
  gaus_mes[0] = float(topReading);

  prior[0] = predict(xTop,process_model,true);
  prior[1] = predict(xTop,process_model,false);
  xTop[0] = update(prior,gaus_mes,true);
  xTop[1] = update(prior,gaus_mes,false);
  
  topMeasurment = int(round(xTop[0]));

  

  // tft display
  tft.setTextSize(5);
  tft.setTextColor(TFT_WHITE,TFT_BLACK);

  // Newton display
  if (graphing)  {
    kg = (bottomReading*500);
    kg = kg/4095;
    if (proper) {
      //    flicker protection
      if (prevBotRead[1] >= 1000 && prevBotRead[0] < 1000)  {
        tft.fillScreen(TFT_BLACK);
        tft.drawFloat(kg,2,10,5);
      }
      else if (prevBotRead[1] >= 100 && prevBotRead[0] < 100)  {
        tft.fillScreen(TFT_BLACK);
        tft.drawFloat(kg,2,10,5);
      }
      else if (prevBotRead[1] >= 10 && prevBotRead[0] < 10)  {
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
    tft.drawString("reading: " + String(bottomReading) + "                ",150,10);
    tft.fillCircle(145,25,2,TFT_GREENYELLOW);
    tft.drawString("fitlter: " + String(botMeasurment) + "             ",150,20);
    tft.setTextSize(1);

    // bottom sensor
    botPixels[3] = botPixels[2];
    botPixels[2] = map(bottomReading, 0, scales[picker],130,1);
    tft.drawLine(i,botPixels[2],i,botPixels[3],TFT_RED);
    botPixels[1] = botPixels[0];
    botPixels[0] = map(botMeasurment, 0, scales[picker],130,1);
    tft.drawLine(i,botPixels[0],i,botPixels[1],TFT_GREENYELLOW);

    // bottom middle sensor
    botMidPixels[3] = botMidPixels[2];
    botMidPixels[2] = map(bottomMidReading, 0, scales[picker],130,1);
    tft.drawLine(i,botMidPixels[2],i,botMidPixels[3],TFT_RED);
    botMidPixels[1] = botMidPixels[0];
    botMidPixels[0] = map(botMidMeasurment, 0, scales[picker],130,1);
    tft.drawLine(i,botMidPixels[0],i,botMidPixels[1],TFT_ORANGE);

    // topmiddle sensor
    topMidPixels[3] = topMidPixels[2];
    topMidPixels[2] = map(topMidReading, 0, scales[picker],130,1);
    tft.drawLine(i,topMidPixels[2],i,topMidPixels[3],TFT_RED);
    topMidPixels[1] = topMidPixels[0];
    topMidPixels[0] = map(topMidMeasurment, 0, scales[picker],130,1);
    tft.drawLine(i,topMidPixels[0],i,topMidPixels[1],TFT_CYAN);

    // tope sensor
    topPixels[3] = topPixels[2];
    topPixels[2] = map(topReading, 0, scales[picker],130,1);
    tft.drawLine(i,topPixels[2],i,topPixels[3],TFT_RED);
    topPixels[1] = topPixels[0];
    topPixels[0] = map(topMeasurment, 0, scales[picker],130,1);
    tft.drawLine(i,topPixels[0],i,topPixels[1],TFT_YELLOW);
    
  }
  
  delay(100);
  

}