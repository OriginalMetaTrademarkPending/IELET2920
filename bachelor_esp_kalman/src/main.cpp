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

  const float phi_af = 0.9989;
  const float phi_ar = 0.8871;
  const float phi_ra = 0.1978;
  const float phi_fa = 0.9992;
  const float M = 30.;

  /*
  const float phi_af = 0.9979;
  const float phi_ar = 0.9777;
  const float phi_ra = 0.0589;
  const float phi_fa = 0.9987;
  */

  BLA::Matrix<2, 2>
      P = {1., 0.,
           0., 1.}; // variance of prior
  BLA::Matrix<2, 2> P_pred = {1., 0.,
                              0., 1.}; // variance of prior

  BLA::Matrix<2, 2> F = {1., 0.3, // unchanged
                         0., 1.};

  BLA::Matrix<2, 1> x = {
      0.,
      0.01,
  };
  BLA::Matrix<2, 1> x_pred = {0.,
                              0.01}; // mean of prior
  BLA::Matrix<1, 1> y = {0.};
  BLA::Matrix<1, 1> z = {0.};
  BLA::Matrix<2, 2> Q = {0.1, 0.001,
                         0.001, 0.1}; // variance of movement - unchanged
  BLA::Matrix<1, 1> R = {20.};        // Variance of measurement - unchanged
  BLA::Matrix<1, 1> S = {1.};
  BLA::Matrix<2, 1> K = {0., 0.};
  BLA::Matrix<1, 2> H = {1.0, 0.0}; // unchanged
  BLA::Matrix<2, 1> B = {phi_ra * M,
                         0};
  BLA::Matrix<2, 2> I = {1., 0.,
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

      B(0) = 0;
    }
    else
    {
      F = {phi_af - phi_ra, 1 - phi_fa - phi_ra,
           1 - phi_af, phi_fa};

      B(0) = phi_ra * M;
    }
    x_pred = (F * x) + (B);
    P_pred = (F * P * ~F) + (Q);
  }

  void update()
  {
    // Calculate residual
    y = z - H * x_pred;

    // Calculate innovation matrix
    S = H * P_pred * ~H + R;

    // Calculate the kalman gain
    K = P_pred * ~H * Invert(S);

    // Calculate the estimates and state covariance matrix
    x = x_pred + K * y;
    P = (I - (K * H)) * P_pred * ~(I - (K * H)) + (K * R * ~K);
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

// Kalman testing
float change = 0.;          // expected change between each measurement (model)
float process_var = 1.;     // variance for process model
float measurement_std = 4.; // standard deviation in measurement
int topmeasurement;

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

  // square process var
  process_var = process_var * process_var;
  float process_model[2] = {change, process_var};

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

void loop()
{
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
