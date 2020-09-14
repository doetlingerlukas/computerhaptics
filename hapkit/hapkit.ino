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

// Magnetic sensor values specific to the prototype
int minVal = 48;
int maxVal = 972;
const double absPosPerDegree = 72.13;

// Position tracking variables
int absPos = 0;
int rawPos = 0;
int lastRawPos = 0;
int flipNumber = 0;
const int flipThresh = 300;
double angle = 0;

void setup() {
  // Set up serial communication
  Serial.begin(57600);

  // Set PWM frequency
  setPwmFrequency(pwmPin, 1);

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
}

void loop() {
  updatePos();
  calcAngle();
  
  forceRendering();
  motorControl();
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

  //Serial.print("Pos: ");
  //Serial.print(absPos);
  //Serial.print("\n");

  // Set last raw position
  lastRawPos = rawPos;
}

void calcAngle() {
  angle = absPos / absPosPerDegree;

  //Serial.print("Angle: ");
  //Serial.print(angle);
  //Serial.print("\n");
}

/*
    calPosMeter()
*/
void calPosMeter() {
 
}
/*
    forceRendering()
*/
void forceRendering() {
  wallForceRendering();

  // Failsave
  if ((angle > 35.0) || (angle < -35.0)) {
    force = 0;
  }

  Serial.print(absPos);
  Serial.print(" ");
  Serial.print(force);
  Serial.print("\n");
}

void springForceRendering() {

  // Spring constant k
  // Unstable if > 0.006;
  const double springConstant = 0.002;

  // Angle where the spring starts
  const double startAngle = 0.0;

  // Calculation of spring
  double calculatedForce = absPos * springConstant;

  // Apply force
  if ((angle > startAngle) || (angle < -startAngle)) {
    force = calculatedForce;
  } else {
    force = 0;
  }
}

void wallForceRendering() {

  // Wall constant k
  // Unstable if > 1.25;
  const double wallConstant = 0.75;

  // Wall is at 5mm, which is an angle of 4.76, as the radius is 60mm
  const double wallStart = -4.76;

  // Apply force
  if ((angle < wallStart)) {
    force = absPos * wallConstant;
  } else {
    force = 0;
  }
}

/*
      Output to motor
*/
void motorControl() {
  Tp = rp / rs * rh * force;  // Compute the require motor pulley torque (Tp) to generate that force
  // Determine correct direction for motor torque
  // You may need to reverse the digitalWrite functions according to your motor connections
  if (force < 0) {
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty = sqrt(abs(Tp) / 0.03);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {
    duty = 1;
  } else if (duty < 0) {
    duty = 0;
  }
  output = (int)(duty * 255);  // convert duty cycle to output signal
  
  analogWrite(pwmPin, output); // output the signal
}

/*
   setPwmFrequency
*/
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if (pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if (pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if (pin == 3 || pin == 11) {
    switch (divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
