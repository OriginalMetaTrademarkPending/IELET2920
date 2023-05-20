#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

// screen setup and naming
TFT_eSPI tft = TFT_eSPI();


const long sampletime = 5; 
const long samplePrintTime = 10;

struct KalmanFilter
{
  // Sensor setup
  int pin;
  int reading = 0;
  
  // phi parameters
  const float M = 13.8384;
  const float phi_af = 0.9979;
  const float phi_ar = 0.9777;
  const float phi_ra = 0.0589;
  const float phi_fa = 0.9987;
  

  
  struct state {
    BLA::Matrix<2, 2> P = {1., 0., 0., 1.};
    BLA::Matrix<2, 1> x = {0.,0.};
  };


  // initializing matrixes
  state prediction;
  state estimate;
  
  BLA::Matrix<2, 2> F = {1., 0.0, 0., 0.};

  BLA::Matrix<1, 1> y = {0.};
  BLA::Matrix<1, 1> z = {0.};
  BLA::Matrix<1, 1> z_prev = {0.};
  BLA::Matrix<1, 1> S = {1.};
  BLA::Matrix<2, 1> K = {0., 0.};
  const BLA::Matrix<2, 2> Q = {0.05, 0.1, 0.1, 0.4};  // variance of movement - unchanged
  const BLA::Matrix<1, 1> R = {0.005};                 // Variance of measurement - unchanged
  const BLA::Matrix<1, 2> H = {1., 0.0}; 
  BLA::Matrix<2, 1> B = {phi_ra * M, 0};
  const BLA::Matrix<2, 2> I = {1., 0., 0., 1.}; 

  void read()
  {
    z_prev = z;
    reading = analogRead(pin);
    z(0) = {static_cast<float>(13.8384*reading)/4095};
  }

  void predict()
  { 
    bool flagDer = (z(0) - z_prev(0))/sampletime > -10;
    bool flagLow = z(0) > 0.1;
    float last_pred = 0.;

    if (flagDer and flagLow)
    {
      F = {phi_af - phi_ra, 1 - phi_fa - phi_ra,
          1 - phi_af, phi_fa};
      
      prediction.x = (F * estimate.x) + (B);
    }
    else
    { 
      F = {phi_af - phi_ar, 1 - phi_fa,
            1 - phi_af, phi_fa};

      last_pred = prediction.x(1);
      prediction.x = (F * estimate.x);
      
      if (last_pred < prediction.x(1))  {
        prediction.x(1) = last_pred;
      }
      
    }
    prediction.P = (F * estimate.P * ~F) + (Q);
  }

  void update()
  {
    // Calculate residual
    y = z - (H * prediction.x);

    // Calculate innovation matrix
    S = (H * prediction.P * ~H) + R;

    // Calculate the kalman gain
    K = (prediction.P * ~H) * Invert(S);

    // Calculate the estimates and state covariance matrix
    estimate.x = prediction.x + (K * y);
    estimate.P = (I - (K * H)) * prediction.P * ~(I - (K * H)) + ((K * R) * ~K);
  }

  void readPredUpd()
  {
    read();
    predict();
    update();
  }
};


// array of current and last measurement for display
int topPixels[2];
int topMidPixels[2];
int botMidPixels[2];
int botPixels[2];

// Current graphing pixel
int i = 0;

// Scales of display
const int scales[4] = {130, 1000, 2000, 4095};
int picker = 0;

// TTGO builtin buttons
int button1Pin = 0;
int button2Pin = 35;


void Display();

// initialize sensors
KalmanFilter topSensor;
KalmanFilter topMidSensor;
KalmanFilter botMidSensor;
KalmanFilter botSensor;

void setup()
{
  // Screen startup
  tft.init();
  tft.setRotation(3);
  tft.invertDisplay(true);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);

  // Pin assignment
  topSensor.pin = 26;
  topMidSensor.pin = 25;
  botMidSensor.pin = 33;
  botSensor.pin = 32;
  
  // Pinmodes
  pinMode(topSensor.pin, INPUT);
  pinMode(topMidSensor.pin, INPUT);
  pinMode(botMidSensor.pin, INPUT);
  pinMode(botSensor.pin, INPUT);

  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);
  
  Serial.begin(115200);
}

unsigned long prevprint = millis();
unsigned long prevSample = millis();
unsigned long sampleStartTime = millis();

void loop()
{ // readings and predictions
  if (millis() - prevSample >= sampletime) {
    topSensor.readPredUpd();
    topMidSensor.readPredUpd();
    botMidSensor.readPredUpd();
    botSensor.readPredUpd();
    prevSample = millis();
  }
  
  
  // print for datacollection to be read by computer
  if((millis() - sampleStartTime) >= samplePrintTime){
    Serial.print(String(static_cast<float>(topSensor.estimate.x(0))));
    Serial.print(",");
    Serial.print(String(static_cast<float>(topMidSensor.estimate.x(0))));
    Serial.print(",");
    Serial.print(String(static_cast<float>(botMidSensor.estimate.x(0))));
    Serial.print(",");
    Serial.print(String(static_cast<float>(botSensor.estimate.x(0))));
    Serial.print(";");
    sampleStartTime = millis();
  }
  


  // print for Serial plotter debugging
  /*
  if (millis() - prevprint >= 20) {
    if (picker == 0 or picker == 3)  {
      Serial.print("Fatigued_muscle_mass: "); Serial.print(topSensor.estimate.x(1)); Serial.print("  ");
      Serial.print("Active_muscle_mass: "); Serial.print(topSensor.estimate.x(0)); Serial.print("  ");
      Serial.print("Measurement: "); Serial.print(topSensor.z(0)); Serial.print("  ");
    }
    if (picker == 1 or picker == 3)  {
      Serial.print("Residual: "); Serial.print(topSensor.y(0)); Serial.print("  ");
      Serial.print("3_sigma: "); Serial.print(sqrt(topSensor.estimate.P(0))*3); Serial.print("  ");
      Serial.print("-3_sigma: "); Serial.print(-sqrt(topSensor.estimate.P(0))*3); Serial.print("  ");
    }
    Serial.println("uT"); // DO NOT REMOVE
    
    prevprint = millis();
  }
  */
  


  // grafing
  // change display scale and reset display
  if (digitalRead(button2Pin) == 0) {
    picker++;
    // wait for button release (debounce)
    while (digitalRead(button2Pin) == 0)  {}
    i = 0;
    tft.fillScreen(TFT_BLACK);
    if (picker >= 4) {
      picker = 0;
    }
  }
  // scroll through screen every 20ms
  if (millis() - prevprint >= 20) {
    i++;
    if (i > 241)  {
      i = 0;
      tft.fillScreen(TFT_BLACK);
    }

    // display the picked scale and the raw and filtered bottomsensor in numbers
    tft.drawString("Scale: " + String(scales[picker]),155,0);
    Display();
    prevprint = millis();
  }

  
  
  
}


void Display()
{
  // Draw lines between last and current point
  topPixels[1] = topPixels[0];
  topPixels[0] = map((4095*topSensor.estimate.x(0))/13.8384, 0, scales[picker], 130, 1);
  tft.drawLine(i, topPixels[0], i, topPixels[1], TFT_BLUE);

  topMidPixels[1] = topMidPixels[0];
  topMidPixels[0] = map((4095*topMidSensor.estimate.x(0))/13.8384, 0, scales[picker], 130, 1);
  tft.drawLine(i, topMidPixels[0], i, topMidPixels[1], TFT_GREENYELLOW);

  botMidPixels[1] = botMidPixels[0];
  botMidPixels[0] = map((4095*botMidSensor.estimate.x(0))/13.8384, 0, scales[picker], 130, 1);
  tft.drawLine(i, botMidPixels[0], i, botMidPixels[1], TFT_ORANGE);

  botPixels[1] = botPixels[0];
  botPixels[0] = map((4095*botSensor.estimate.x(0))/13.8384, 0, scales[picker], 130, 1);
  tft.drawLine(i, botPixels[0], i, botPixels[1], TFT_RED);
}

