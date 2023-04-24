#include <Arduino.h>
#include <TFT_eSPI.h>
#include <SPI.h>
#include <BasicLinearAlgebra.h>
using namespace BLA;

struct KalmanFilter
{
  // Sensor setup
  int pin;
  int reading = 0;
  /*
  const float phi_af = 0.9989;
  const float phi_ar = 0.8871;
  const float phi_ra = 0.1978;
  const float phi_fa = 0.9992;
  */
  const float M = 30.;

  
  const float phi_af = 0.9979;
  const float phi_ar = 0.9777;
  const float phi_ra = 0.0589;
  const float phi_fa = 0.9987;
  

  

  struct state {
    BLA::Matrix<2, 2> P = {1., 0., 0., 1.};
    BLA::Matrix<2, 1> x = {0.,0.};

    void maxpoints() {
      /*
      for (int i = 0; i < 4; i++) {
        if (P(i) > 1E+06)  {
          P(i) = 1E+06;
        }
        else if (P(i) < 1E-06)  {
          P(i) = 1E-06;
        }
        else if (abs(P(i)) < 0.0001)  {
          P(i) = 0.0001;
        }
      }
      for (int i = 0; i < 2; i++) {
        if (x(i) > 1E+10)  {
          x(i) = 1E+10;
        }
        else if (x(i) < 1E-10)  {
          x(i) = 1E-10;
        }
        else if (abs(x(i)) < 0.0001)  {
          x(i) = 0.0001;
        }
      }
      */
    }
  };

  state prediction;
  state estimate;

  BLA::Matrix<2, 2> F = {1., 0.0, 
                          0., 0.};

  BLA::Matrix<1, 1> y = {0.};
  BLA::Matrix<1, 1> z = {0.};
  const BLA::Matrix<2, 2> Q = {0.05, 0.1,
                         0.1, 0.4}; // variance of movement - unchanged
  const BLA::Matrix<1, 1> R = {0.08};        // Variance of measurement - unchanged
  BLA::Matrix<1, 1> S = {1.};
  BLA::Matrix<2, 1> K = {0., 0.};
  const BLA::Matrix<1, 2> H = {1., 0.0}; // unchanged
  const BLA::Matrix<2, 1> B = {phi_ra * M,
                                0};
  const BLA::Matrix<2, 2> I = {1., 0.,
                                0., 1.}; // unchanged

  void read()
  {
    reading = analogRead(pin);
    z(0) = {float(reading)};
  }

  void predict()
  {
    if (z(0) == 0.)
    {
      F = {phi_af - phi_ar, 1 - phi_fa,
            1 - phi_af, phi_fa};

      prediction.x = (F * estimate.x);
    }
    else
    {
      F = {phi_af - phi_ra, 1 - phi_fa - phi_ra,
            1 - phi_af, phi_fa};

      prediction.x = (F * estimate.x) + (B);
    }
    prediction.P = (F * estimate.P * ~F) + (Q);
    prediction.maxpoints();
  }

  void update()
  {
    // Calculate residual
    y = z - H * prediction.x;

    // Calculate innovation matrix
    S = H * prediction.P * ~H + R;

    // Calculate the kalman gain
    K = prediction.P * ~H * Invert(S);

    // Calculate the estimates and state covariance matrix
    estimate.x = prediction.x + K * y;
    estimate.P = (I - (K * H)) * prediction.P * ~(I - (K * H)) + (K * R * ~K);
    estimate.maxpoints();
  }

  void readPredUpd()
  {
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

// screen setup and naming
TFT_eSPI tft = TFT_eSPI();

// make true to serial write the raw sensorvalues
const bool printRaw = false;

// make true to display the raw sensordata in red
const bool displayRaw = false;

// make false to not display the filtered sensors
const bool displayFiltered = true;

// make true/false to enable/disable sensors
const bool topsens = true;

// timeline of measurement for display
int topPixels[4];

//  current graphing pixel
int i = 0;

//  scale of display
const int scales[4] = {130, 1000, 2000, 4095};
int picker = 0;

// TTGO builtin buttons
int button1Pin = 0;
int button2Pin = 35;


void readSensors();

void topDisplay();

KalmanFilter topSensor;

void setup()
{
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

  if (printRaw)
  {
    Serial.begin(115200);
  }
  Serial.begin(115200);
}

unsigned long sampleStartTime = millis();

void loop()
{
  topSensor.readPredUpd();
  
  Serial.print("x: ");
  Serial.print(topSensor.estimate.x(0));
  Serial.print(",z: ");
  Serial.println(topSensor.z(0));
  
  /*
  Serial.println(i);
  Serial << "predicted Z value: " << topSensor.z << '\n';
  Serial << "predicted X value: " << topSensor.estimate.x << '\n';
  Serial << "predicted P value: " << topSensor.estimate.P << '\n';
  Serial << "predicted P_pred : " << topSensor.prediction.P << '\n';
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

void readSensors()
{
  topSensor.read();
}

void topDisplay()
{
  if (displayRaw)
  {
    topPixels[3] = topPixels[2];
    topPixels[2] = map(topSensor.reading, 0, scales[picker], 130, 1);
    tft.drawLine(i, topPixels[2], i, topPixels[3], TFT_RED);
  }

  if (displayFiltered)
  {
    topPixels[1] = topPixels[0];
    topPixels[0] = map(0, 0, scales[picker], 130, 1);
    tft.drawLine(i, topPixels[0], i, topPixels[1], TFT_BLUE);
  }
}
