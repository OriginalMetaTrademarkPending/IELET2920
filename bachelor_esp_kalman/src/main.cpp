#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;



struct KalmanFilter {
  // Sensor setup
  int pin;
  int reading = 0;

  BLA::Matrix<3,3> P = {1.,0.,0.,
                        0.,1.,0.,
                        0.,0.,1.};              // variance of prior
  BLA::Matrix<3,3> P_pred = {1.,0.,0.,
                        0.,1.,0.,
                        0.,0.,1.};              // variance of prior

  BLA::Matrix<3,3> F = {1.,0.3,0.,              // unchanged
                        0.,1.,0.3,
                        0.,0.,1.};

  BLA::Matrix<3,1> x = {0.,                 
                        0.01,
                        0.};              
  BLA::Matrix<3,1> x_pred = {0.,            
                        0.01,
                        0.};                    // mean of prior
  BLA::Matrix<1,1> y = {0.};                
  BLA::Matrix<1,1> z = {0.};                
  BLA::Matrix<3,3> Q = {0.1, 0.001,0.,            
                        0.001,0.1,0.001,
                        0.,0.001,0.1};             // variance of movement - unchanged
  BLA::Matrix<1,1> R = {20.};                    // Variance of measurment - unchanged
  BLA::Matrix<1,1> S = {1.};            
  BLA::Matrix<3,1> K = {0.,0.,0.};            
  BLA::Matrix<1,3> H = {1.0,0.0,0.0};            // unchanged
  BLA::Matrix<3,3> I = {1.,0.,0.,
                        0.,1.,0.,
                        0.,0.,1.};              // unchanged
  


  void read() {
    reading = analogRead(pin);
    z = {float(reading)};
  }

  void predict()  {
    x_pred = F*x;
    P_pred = F*P*~F+Q;
  }

  void update() {
    S = H*P_pred*~H + R;
    K = P_pred*~H*Invert(S);
    y = z - H*x_pred;
    x = x_pred + K*y;
    P = (I-K*H)*P_pred;
  }

  void readPredUpd()  {
    read();
    predict();
    update();
    /*
    if (x(0) < 0)  {
      x(0) = 0;
    }
    else if (x(0) > 4095)  {
      x(0) = 4095;
    }
    if (x(1) < -4095) {
      x(1) = -4095;
    }
    else if (x(1) > 4095) {
      x(1) = 4095;
    }
    */
   
  }
};




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
float gaus_mes[] = {float(0),measurment_std*measurment_std};




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


void readSensors();

void singleTopKalman();

void topDisplay();

KalmanFilter topSensor;

void setup() {
  // put your setup code here, to run once:
  // Screen startup
  tft.init();
  tft.setRotation(3);
  tft.invertDisplay(true);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(1);


  topSensor.pin = 26;
  // Pinmodes
  pinMode(topSensor.pin, INPUT);
  
  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);

  if (printRaw) {Serial.begin(115200);}
  Serial.begin(115200);

  // square process var
  process_var = process_var * process_var;
  float process_model[2] = {change,process_var};

  topSensor.predict();
  Serial << "C inverse: " << topSensor.x << '\n';
  topSensor.predict();
  Serial << "C inverse: " << topSensor.x << '\n';
  topSensor.predict();
  Serial << "C inverse: " << topSensor.x << '\n';
  topSensor.predict();
  Serial << "C inverse: " << topSensor.x << '\n';
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
topSensor.readPredUpd();

Serial.print("x: ");
Serial.print(topSensor.x(0));
Serial.print(",z: ");
Serial.println(topSensor.z(0));

/*
Serial.println(i);
Serial << "predicted X value: " << topSensor.x << '\n';
Serial << "predicted P value: " << topSensor.P << '\n';
Serial << "predicted P_pred : " << topSensor.P_pred << '\n';
Serial << "predicted K value: " << topSensor.K << '\n';
Serial << "predicted S value: " << topSensor.S << '\n';
i++;
*/
delay(20);


  /*
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
  */
}

void readSensors()  {
  topSensor.read();
}

void singleTopKalman()  {
  gaus_mes[0] = float(topSensor.reading);

  prior[0] = predict(xTop,process_model,true);
  prior[1] = predict(xTop,process_model,false);
  xTop[0] = update(prior,gaus_mes,true);
  xTop[1] = update(prior,gaus_mes,false);
  
  topMeasurment = int(round(xTop[0]));
}

void topDisplay() {
  if (displayRaw) {
    topPixels[3] = topPixels[2];
    topPixels[2] = map(topSensor.reading, 0, scales[picker],130,1);
    tft.drawLine(i,topPixels[2],i,topPixels[3],TFT_RED);
  }

  if (displayFiltered) {
    topPixels[1] = topPixels[0];
    topPixels[0] = map(topMeasurment, 0, scales[picker],130,1);
    tft.drawLine(i,topPixels[0],i,topPixels[1],TFT_BLUE);
  }
}
