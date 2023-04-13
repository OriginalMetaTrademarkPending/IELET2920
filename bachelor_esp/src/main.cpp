#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>

//screen setup and naming
TFT_eSPI tft = TFT_eSPI();

// make true to serial write the raw sensorvalues
const bool printRaw = false;

// make true to display the raw sensordata in red
const bool displayRaw = false;

// make false to not display the filtered sensors
const bool displayFiltered = true;

// make true/false to enable/disable sensors
const bool botsens = false;
const bool botmidsens = false;
const bool topmidsens = false;
const bool topsens = true;

// Sensorpins
const int bottomSensorPin = 32;
const int bottomMiddleSensorPin = 33;
const int topMiddleSensorPin = 25;
const int topSensorPin = 26;

// Sensor values
int bottomReading;
int bottomMidReading;
int topMidReading;
int topReading;


// timeline of measurments for display
int botPixels[4];
int botMidPixels[4];
int topMidPixels[4];
int topPixels[4];

//  current graphing pixel
int i = 0;

//  scale of display
const int scales[4] = {130, 1000, 2000, 4095};
int picker = 0;

// TTGO builtin buttons
int button1Pin = 0;
int button2Pin = 35;



// Kalman testing
float change = 0.;            // expected change between each measurment (model)
float process_var = 1.;       // variance for process model
float measurment_std = 4.;    // standard deviation in measurments
int botMeasurment;
int botMidMeasurment;
int topMidMeasurment;
int topMeasurment;

float prior[2];
float xBot[] = {0.,100.}; // inital state
float xBotMid[] = {0.,100.}; // inital state
float xTopMid[] = {0.,100.}; // inital state
float xTop[] = {0.,100.}; // inital state

float process_model[] = {change,process_var};
float gaus_mes[] = {float(bottomReading),measurment_std*measurment_std};


// pressure test

const unsigned long sampleTime = 100;
#define CONVERSION 50.0/4095.0
unsigned long sampleStartTime = millis();


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
  tft.setTextSize(1);

  // Pinmodes
  pinMode(bottomSensorPin, INPUT);
  pinMode(bottomMiddleSensorPin, INPUT);
  pinMode(topMiddleSensorPin, INPUT);
  pinMode(topSensorPin, INPUT);
  
  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);

  if (printRaw) {Serial.begin(115200);}
  Serial.begin(115200);

  // square process var
  process_var = process_var * process_var;
  float process_model[2] = {change,process_var};
}


void loop() {
  // reading sensors
  if (botsens)    {bottomReading = analogRead(bottomSensorPin);}
  if (botmidsens) {bottomMidReading = analogRead(bottomMiddleSensorPin);}
  if (topmidsens) {topMidReading = analogRead(topMiddleSensorPin);}
  if (topsens)    {topReading = analogRead(topSensorPin);}
  
  if((millis() - sampleStartTime) >= sampleTime){
      float result = static_cast<float>(topReading*CONVERSION);
      Serial.print(String(result));
      Serial.print(",");
      sampleStartTime = millis();
    }

  // print raw sensors
  if (printRaw) {
    if (botsens)    {Serial.println("bottom: " + String(bottomReading));}
    if (botmidsens) {Serial.println("botmid: " + String(bottomMidReading));}
    if (topmidsens) {Serial.println("topmid: " + String(topMidReading));}
    if (topsens)    {Serial.println("top:    " + String(topReading));}
  }

  


  // basic kalman
  //  bottom sensor
  if (botsens)  {
    gaus_mes[0] = float(bottomReading);

    prior[0] = predict(xBot,process_model,true);
    prior[1] = predict(xBot,process_model,false);
    xBot[0] = update(prior,gaus_mes,true);
    xBot[1] = update(prior,gaus_mes,false);
    
    botMeasurment = int(round(xBot[0]));
  }

  //  bottom middle sensor
  if (botmidsens) {
    gaus_mes[0] = float(bottomMidReading);

    prior[0] = predict(xBotMid,process_model,true);
    prior[1] = predict(xBotMid,process_model,false);
    xBotMid[0] = update(prior,gaus_mes,true);
    xBotMid[1] = update(prior,gaus_mes,false);
    
    botMidMeasurment = int(round(xBotMid[0]));
  }

  //  top  middle sensor
  if (topmidsens) {
    gaus_mes[0] = float(topMidReading);

    prior[0] = predict(xTopMid,process_model,true);
    prior[1] = predict(xTopMid,process_model,false);
    xTopMid[0] = update(prior,gaus_mes,true);
    xTopMid[1] = update(prior,gaus_mes,false);
    
    topMidMeasurment = int(round(xTopMid[0]));
  }

  //  top sensor
  if (topsens)  {
    gaus_mes[0] = float(topReading);

    prior[0] = predict(xTop,process_model,true);
    prior[1] = predict(xTop,process_model,false);
    xTop[0] = update(prior,gaus_mes,true);
    xTop[1] = update(prior,gaus_mes,false);
    
    topMeasurment = int(round(xTop[0]));
  }

  

  // grafing
  // change display scale and reset display
  if (digitalRead(button2Pin) == 0) {
    picker++;
    while (digitalRead(button2Pin) == 0)  {}  // button debouncer
    i = 0;
    tft.fillScreen(TFT_BLACK);
    if (picker >= 4) {
      picker = 0;
    }
  }


  // scrolling thru screen
  i++;
  if (i > 241)  {
    i = 0;
    tft.fillScreen(TFT_BLACK);
  }

  
  // display the picked scale and the raw and filtered bottomsensor in numbers
  tft.drawString("Scale: " + String(scales[picker]),155,0);

  // bottom sensor
  if (botsens)  {
  tft.fillCircle(145,15,2,TFT_RED);
  tft.drawString("reading: " + String(bottomReading) + "                ",150,10);
  tft.fillCircle(145,25,2,TFT_GREENYELLOW);
  tft.drawString("fitlter: " + String(botMeasurment) + "             ",150,20);
  
    if (displayRaw) {
      botPixels[3] = botPixels[2];
      botPixels[2] = map(bottomReading, 0, scales[picker],130,1);
      tft.drawLine(i,botPixels[2],i,botPixels[3],TFT_RED);
    }
    if (displayFiltered) {
      botPixels[1] = botPixels[0];
      botPixels[0] = map(botMeasurment, 0, scales[picker],130,1);
      tft.drawLine(i,botPixels[0],i,botPixels[1],TFT_GREENYELLOW);
    }
  }

  // bottom middle sensor
  if (botmidsens) {
    if (displayRaw) {
      botMidPixels[3] = botMidPixels[2];
      botMidPixels[2] = map(bottomMidReading, 0, scales[picker],130,1);
      tft.drawLine(i,botMidPixels[2],i,botMidPixels[3],TFT_RED);
    }

    if (displayFiltered) {
      botMidPixels[1] = botMidPixels[0];
      botMidPixels[0] = map(botMidMeasurment, 0, scales[picker],130,1);
      tft.drawLine(i,botMidPixels[0],i,botMidPixels[1],TFT_ORANGE);
    }
  }

  // topmiddle sensor
  if (topmidsens) {
    if (displayRaw) {
      topMidPixels[3] = topMidPixels[2];
      topMidPixels[2] = map(topMidReading, 0, scales[picker],130,1);
      tft.drawLine(i,topMidPixels[2],i,topMidPixels[3],TFT_RED);
    }

    if (displayFiltered) {
      topMidPixels[1] = topMidPixels[0];
      topMidPixels[0] = map(topMidMeasurment, 0, scales[picker],130,1);
      tft.drawLine(i,topMidPixels[0],i,topMidPixels[1],TFT_CYAN);
    }
  }

  // top sensor
  if (topsens)  {
    if (displayRaw) {
      topPixels[3] = topPixels[2];
      topPixels[2] = map(topReading, 0, scales[picker],130,1);
      tft.drawLine(i,topPixels[2],i,topPixels[3],TFT_RED);
    }

    if (displayFiltered) {
      topPixels[1] = topPixels[0];
      topPixels[0] = map(topMeasurment, 0, scales[picker],130,1);
      tft.drawLine(i,topPixels[0],i,topPixels[1],TFT_BLUE);
    }
  }
  
  delay(0);
  

}