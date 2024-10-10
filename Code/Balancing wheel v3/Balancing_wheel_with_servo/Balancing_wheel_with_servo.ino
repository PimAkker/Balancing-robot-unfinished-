// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <PID_v1.h>
#include <Wire.h>
#include <AccelStepper.h>
double Setpoint, Input, Output;
int filterWindow = 5;
const int MPU_addr = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
PID myPID(&Input, &Output, &Setpoint, 1, 0, 0, DIRECT);


// Define stepper motor connections and motor interface type. Motor interface type must be set to 1 when using a driver:
#define dirPin 2
#define stepPin 3
#define motorInterfaceType 1

// Create a new instance of the AccelStepper class:
AccelStepper stepper = AccelStepper(motorInterfaceType, stepPin, dirPin);
int deadangle = 5;

int minVal = 265;
int maxVal = 402;

double x;
double y;
double z;


const int potPin = A0;
const int maxValPot = 5;
double potValue;

// filter settings
#define WINDOW_SIZE 5

int INDEX = 0;
int VALUE = 0;
int SUM = 0;
int READINGS[WINDOW_SIZE];
int AVERAGED = 0;

void setup() {
  stepper.setMaxSpeed(1000);

  pinMode(potPin, INPUT);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(115200);  // set up Serial library at 9600 bps
  Serial.println("Starting");




  Setpoint = 180;

  myPID.SetOutputLimits(0, 255);
  myPID.SetMode(AUTOMATIC);
}
void getReadings() {
  for (int i = 0; i < 6; i++) {
    readSensor();
    Serial.print("--AngleX= ");
    Serial.print(x);
    SUM = SUM - READINGS[INDEX];        // Remove the oldest entry from the sum
    VALUE = x;                          // Read the next sensor value
    READINGS[INDEX] = VALUE;            // Add the newest reading to the window
    SUM = SUM + VALUE;                  // Add the newest reading to the sum
    INDEX = (INDEX + 1) % WINDOW_SIZE;  // Increment the index, and wrap to 0 if it exceeds the window size

    x = SUM / WINDOW_SIZE;  // Divide the sum of the window by the window size for the result
  }
  Serial.print("--AngleXFiltered= ");
  Serial.print(x);
}
void readSensor() {

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  int xAng = map(AcX, minVal, maxVal, -90, 90);
  int yAng = map(AcY, minVal, maxVal, -90, 90);
  int zAng = map(AcZ, minVal, maxVal, -90, 90);

  x = RAD_TO_DEG * (atan2(-yAng, -zAng) + PI);
  y = RAD_TO_DEG * (atan2(-xAng, -zAng) + PI);
  z = RAD_TO_DEG * (atan2(-yAng, -xAng) + PI);

  if (x > 180) {
    Input = 180 - (x - 180);
  } else {
    Input = x;
  }
}
void computePID() {

  // potValue = analogRead(potPin);
  // potValue = map(potValue, 0, 1023, 0, maxValPot * 8);
  // // Serial.print("---potvalue: ");
  // potValue = potValue / 8;
  // // Serial.print(potValue);

  myPID.SetTunings(1, 0.1, 0.5);
  myPID.Compute();
  Serial.print("  PID output: ");
  Serial.print(Output);
}

void loop() {
  unsigned long loopStartTime = millis();
  getReadings();
  uint8_t i;
  computePID();

  if (x < 180 - deadangle) {

    stepper.setSpeed(Output * 4);
    // stepper.runSpeed();

    Serial.print(" --Backward-- ");


  }

  else if (x > 180 + deadangle) {

    stepper.setSpeed(1000);
    // stepper.runSpeed();
    // stepper.setSpeed(-Output*4);

    Serial.print(" --Forward-- ");

  } else {
    Serial.print(" --Off-- ");
    stepper.setSpeed(0);
    // stepper.runSpeed();
  }
  Serial.print("---this loop took: ");
  Serial.print(millis() - loopStartTime);
  Serial.print("-- Stepper speed: ");

  Serial.println(stepper.speed());
}