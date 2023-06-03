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
  const BLA::Matrix<1, 2> H = {1, 0.0}; 
  const BLA::Matrix<2, 1> B = {phi_ra * M, 0};
  const BLA::Matrix<2, 2> I = {1., 0., 0., 1.}; 

  void read()
  {
    z_prev = z;
    reading = analogRead(pin);
    z(0) = {static_cast<float>(13.8384*reading)/4095};
  }

void predict()  { 
  bool flagDer = (z(0) - z_prev(0))/sampletime > derThresh;
  bool flagLow = z(0) > threshold;

  if (flagDer and flagLow)  {
    F = FPressed;

    prediction.x = (F * estimate.x) + (B);
  }
  else  { 
    F = FReleased;

    prediction.x = (F * estimate.x);
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





// make true to display the raw sensordata in red
const bool displayRaw = true;

// make false to not display the filtered sensors
const bool displayFiltered = true;

// make true/false to enable/disable sensors
const bool topsens = true;

// timeline of measurement for display
int topPixels[4];
int topMidPixels[4];
int botMidPixels[4];
int botPixels[4];

//  current graphing pixel
int i = 0;

//  scale of display
const int scales[4] = {130, 1000, 2000, 4095};
int picker = 0;

// TTGO builtin buttons
int button1Pin = 0;
int button2Pin = 35;


void topDisplay();

KalmanFilter topSensor;
KalmanFilter topMidSensor;
KalmanFilter botMidSensor;
KalmanFilter botSensor;

void setup()
{
  // Screen startup
  tft.init();

  // Pin assignment
  topSensor.pin = 26;
  topMidSensor.pin = 25;
  botMidSensor.pin = 33;
  botSensor.pin = 32;
  
}



void loop()
{ // readings and predictions
  if (millis() - prevSample >= sampletime) {
    topSensor.readPredUpd();
    topMidSensor.readPredUpd();
    botMidSensor.readPredUpd();
    botSensor.readPredUpd();
    prevSample = millis();
  }
  
  
  // print for datacollection
  if((millis() - sampleStartTime) >= samplePrintTime){
    //static_cast<float>
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
  



  /*
  if (millis() - prevprint >= 20) {
    //Serial.print("x musselmasstop: "); Serial.print(topSensor.estimate.x(1)); Serial.print("  ");
    //Serial.print("x pred: "); Serial.print(topSensor.prediction.x(0)); Serial.print("  ");
    Serial.print("x pred musselmasstop: "); Serial.print(topSensor.prediction.x(1)); Serial.print("  ");
    //Serial.print("y top: "); Serial.print(topSensor.y(0)); Serial.print("  ");
    //Serial.print("P top: "); Serial.print(sqrt(topSensor.estimate.P(0))*5); Serial.print("  ");
    //Serial.print("P bot: "); Serial.print(-sqrt(topSensor.estimate.P(0))*5); Serial.print("  ");
    Serial.print("x_top: "); Serial.print(topSensor.estimate.x(0)); Serial.print("  ");
    Serial.print("z_top: "); Serial.print(topSensor.z(0)); Serial.print("  ");
    //Serial.print("x_topMid: "); Serial.print(topMidSensor.estimate.x(0)); Serial.print("  ");
    //Serial.print("z_topMid: "); Serial.print(topMidSensor.z(0)); Serial.print("  ");
    //Serial.print("x_botMid: "); Serial.print(botMidSensor.estimate.x(0)); Serial.print("  ");
    //Serial.print("z_botMid: "); Serial.print(botMidSensor.z(0)); Serial.print("  ");
    //Serial.print("x_bot: "); Serial.print(botSensor.estimate.x(0)); Serial.print("  ");
    //Serial.print("z_bot: "); Serial.print(botSensor.z(0)); Serial.print("  ");
    Serial.println("uT");
    prevprint = millis();
  }*/
  
  /*
  // debuggingprinting
  Serial.println(i);
  Serial << "predicted Z value: " << topSensor.z << '\n';
  Serial << "predicted X value: " << topSensor.estimate.x << '\n';
  Serial << "predicted P value: " << topSensor.estimate.P << '\n';
  Serial << "predicted P_pred : " << topSensor.prediction.P << '\n';
  Serial << "predicted K value: " << topSensor.K << '\n';
  Serial << "predicted S value: " << topSensor.S << '\n';
  i++;
  */
 
  // grafing
  if (millis() - prevprint >= 20) {
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
      Display();
    }
    prevprint = millis();
  }

  
  
  
}


void Display()
{
  if (displayFiltered)
  {
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
}
