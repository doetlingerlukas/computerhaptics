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

  // Used for hard surface rendering
  lastTimeAtSurface = 0;
}

void loop() {
  updatePos();
  
  //forceRendering();

  control();
  
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

/*
    Control
*/
void control() {
  const double refInput = 20;
  double error = angle - refInput;

  pd(error);

  Serial.print(millis());
  Serial.print(" ");
  Serial.print(force);
  Serial.print("\n");
}

// P-Type Control
void p(double error) {
  const double Kp = 0.15;

  force = error * Kp + 0.7 * sgn(error);
}
// PD-Type Control
void pd(double error) {
  const double Kp = 0.04;
  const double Kd = 0.15;

  force = (error * Kp + 0.4 * sgn(error)) - velocity * Kd;
}
// PID-Type Control
void pid(double error) {
  const double Kp = 0.025;
  const double Kd = 0.05;
  const double Ki = 0.00001;
  
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);

  errorSum += (error * timeChange);

  force = (error * Kp + 0.4 * sgn(error)) - Kd * velocity + Ki * errorSum;
  
  lastTime = now;
}

/*
    forceRendering()
*/
void forceRendering() {
  // Function to render the desired force
  // springForceRendering();

  // Failsave
  if ((angle > 35.0) || (angle < -35.0)) {
    force = 0;
  }

  //Serial.print(-absPos);
  //Serial.print(", ");
  Serial.print(-force);
  Serial.print(", ");
  Serial.print(-velocity);
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

void frictionRendering(boolean coulomb) {

  // Constant defining the force caused by friction
  const double frictionForce = 1;
  const double viscousForce = 0.75;

  if (coulomb) {
    // Render coulomb fricition
    force = frictionForce * sgn(velocity);
  } else {
    // Render viscous friction
    force = viscousForce * velocity;
  }

  // Prevent instability
  if (!(velocity > 0.2 || velocity < -0.2 )) {
    force = 0;
  }
}

void hardSurfaceRendering() {

  double seconds = (double) millis() / 1000.0;

  // Wall force in N/m
  const double wallForce = 100;

  const double maxVibration = 100;
  const double t = 2.0;

  // Surface is at 5mm, which is an angle of 4.76, as the radius is 60mm
  const double surfaceStart = -4.76;

  // Apply force of surface
  if ((angle < surfaceStart)) {

    // Calculate t
    if (lastTimeAtSurface == 0) {
      lastTimeAtSurface = seconds;
    }
    seconds -= lastTimeAtSurface;

    // Applay wibration to the force
    double vibrationForce = maxVibration * exp(-seconds) * cos(2 * PI * seconds);
    force = sgn(absPos) * wallForce + vibrationForce;
  } else {
    force = 0;
  }
}

void textureRendering() {

  // Width of damping area
  double w = .01;
  double damper = 1;
  
  for (int i=0; i<9; i=i+2) {
    if ((abs(posMeters) > i*w) && (abs(posMeters) < (i+1)*w)) {
      force = -damper * velocity;
    } else if ((abs(posMeters) > (i+1)*w) && (abs(posMeters) < (i+2)*w)){
      force = 0;
    }
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

int sgn(double x) {
  if (x == 0)
    return 0;
  else
    return (x > 0) ? 1 : -1;
}
