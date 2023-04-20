#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

//screen setup and naming
TFT_eSPI tft = TFT_eSPI();

// make true to serial write the raw sensorvalues
const bool printRaw = false;

// make true to display the raw sensordata in red
const bool displayRaw = false;

// make false to not display the filtered sensors
const bool displayFiltered = true;

// make true/false to enable/disable sensors
const bool topsens = true;

// Sensorpins
const int topSensorPin = 26;

// Sensor values
int topReading;


// timeline of measurments for display
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
int topMeasurment;

float prior[2];

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


void readSensors()  {
  if (botsens)    {bottomReading = analogRead(bottomSensorPin);}
  if (botmidsens) {bottomMidReading = analogRead(bottomMiddleSensorPin);}
  if (topmidsens) {topMidReading = analogRead(topMiddleSensorPin);}
  if (topsens)    {topReading = analogRead(topSensorPin);}
}

void singleTopKalman()  {
  gaus_mes[0] = float(topReading);

  prior[0] = predict(xTop,process_model,true);
  prior[1] = predict(xTop,process_model,false);
  xTop[0] = update(prior,gaus_mes,true);
  xTop[1] = update(prior,gaus_mes,false);
  
  topMeasurment = int(round(xTop[0]));
}

void topDisplay() {
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

  /*
  BLA::Matrix<2,3> A = {1,2,3,4,5,6};
  BLA::Matrix<3,2> B = {10,11,20,21,30,31};
  BLA::Matrix<2,2> C = A*B;
  BLA::Matrix<2,2> C_inv = C;
  
  // making sure the matrix is not singular
  bool is_nonsingular = Invert(C_inv);
  Serial << "C inverse: " << C_inv << '\n';
  */
}

unsigned long sampleStartTime = millis();

void loop() {
  // reading sensors
  readSensors();
  
  // basic kalman
  //  top sensor
  if (topsens)  {
    singleTopKalman();
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

  // top sensor
  if (topsens)  {
    topDisplay();
  }
  
  
  delay(0);
}
