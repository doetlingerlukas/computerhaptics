// Based on: https://hapkit.stanford.edu/files/HapkitLab4SolutionsForWebsite.ino

// Includes
#include <math.h>

// Pins
int pwmPin = 3;
int dirPin = 12;
int sensorPin = A2;

// Constants for force-torque conversion
double rp = 0.004191;
double rs = 0.073152;
double rh = 0.065659;

// Force output variables
double force = 0;           // force at the handle
double Tp = 0;              // torque of the motor pulley
double duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor

// Values specific to the prototype
int minVal = 48;
int maxVal = 972;
const double absPosPerDegree = 72.13;
const double handle_radius = 0.065659;   //[m]

// Position tracking variables
int absPos = 0;
int rawPos = 0;
int lastRawPos = 0;
int flipNumber = 0;
const int flipThresh = 300;
double angle = 0;
double posMeters = 0;
double lastPosMeters = 0;
double velocity = 0;
double lastVelocity = 0;
double lastLastVelocity = 0;
double lastTimeAtSurface = 0;
double lastTime = 0;
double errorSum = 0;

// iteration counter
int i = 0;

void setup() {
  // Set up serial communication
  Serial.begin(57600);

  // Set PWM frequency
  // setPwmFrequency(pwmPin, 1);

  // Input pins
  pinMode(sensorPin, INPUT); // set MR sensor pin to be an input

  // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor
  pinMode(dirPin, OUTPUT);  // dir pin for motor

  // Initialize motor
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction

  // Initialize position valiables
  lastRawPos = analogRead(sensorPin);

  // Used for hard surface rendering
  lastTimeAtSurface = 0;
}

void loop() {
  i++;
  updatePos();

  // Volume knob ranges from -25 to 25 degrees
  double volume = abs(calculateVolume(-30.0, 30.0));

  if (i % 75 == 0)
    Serial.println(volume);
}

// Returns value for the volume between 0 and 1
double calculateVolume(double start, double end) {
  double abs_volume = ((angle + start) / ((end - start) / 100.0));
  return abs_volume / 100.0;
}

void updatePos() {
  // Get voltage output by MR sensor
  rawPos = analogRead(sensorPin);  //current raw position from MR sensor

  // Update min and max values of MR sensor
  if (rawPos > maxVal) {
    maxVal = rawPos;
  }
  if (rawPos < minVal) {
    minVal = rawPos;
  }

  // Calculate position difference
  int rawDiff = rawPos - lastRawPos;
  int rawOffset = abs(rawDiff);

  // Update absolute position
  if((rawOffset > flipThresh)) {
    if(rawDiff > 0) {
      flipNumber--;
      absPos = absPos + (rawPos - maxVal) + (minVal - lastRawPos);
    } else {
      flipNumber++;
      absPos = absPos + (maxVal - lastRawPos) + (rawPos - minVal);
    }
  } else {
    absPos += rawDiff;
  }

  // Set last raw position
  lastRawPos = rawPos;

  // Calculate angle
  angle = absPos / absPosPerDegree;

  // Calculate Position in meters
  double radiant = (angle * PI) / 180;
  posMeters = radiant * handle_radius;

  // Calculate velocity
  velocity = -(.95*.95) * lastLastVelocity + 2*.95*lastVelocity + (1-.95)*(1-.95)*(posMeters - lastPosMeters)/.0001;
  lastPosMeters = posMeters;
  lastLastVelocity = lastVelocity;
  lastVelocity = velocity; 
}