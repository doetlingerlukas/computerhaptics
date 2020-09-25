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
int initPos = 0;
int rawPos = 0;
int lastRawPos = 0;
int flipNumber = 0;
double angle = 0;
double posMeters = 0;
double lastPosMeters = 0;
double velocity = 0;
double lastVelocity = 0;
double lastLastVelocity = 0;
double lastTimeAtSurface = 0;
double lastTime = 0;
double errorSum = 0;
boolean flipped = false;

// iteration counter
int i = 0;

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
  initPos = lastRawPos;

  // Used for hard surface rendering
  lastTimeAtSurface = 0;
}

void loop() {
  i++;
  updatePos();

  const double startAngle = -30.0;
  const double endAngle = 30.0;

  // Volume knob ranges from -25 to 25 degrees
  double volume = abs(calculateVolume(startAngle, endAngle));

  force =  calculateWallForce(startAngle, endAngle);
  force += calculateTextureForce(startAngle, endAngle, volume);

  if (i % 75 == 0) {
    Serial.println(volume);
  }

  motorControl();
}

// Returns value for the volume between 0 and 1
double calculateVolume(double start, double end) {
  if (angle >= end) {
    return 0.0;
  } else if (angle <= start) {
    return 1.0;
  }
  
  double absVolume = ((angle + start) / ((end - start) / 100.0));
  return absVolume / 100.0;
}

double calculateWallForce(double start, double end) {
  const double wallConstant = 0.15;

  // Apply wall force
  if (angle < start-2 || angle > end+2) {
    return angle * wallConstant;
  }
  return 0.0;
}

double calculateTextureForce(double start, double end, double volume) {
  if (angle <= start || angle >= end) {
    return 0.0;
  }

  const double damper = 2;
  int absVolume = (int)(volume * 100.0);

  // Apply texture force
  if (absVolume % 10 == 0) {
    return -damper * velocity;
  }
  return 0.0;
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

  int maxDistance = maxVal - minVal;
  int threshold = maxDistance * 0.3;

  // Calculate position difference
  int rawDiff = rawPos - lastRawPos;
  int rawOffset = abs(rawDiff);

  if (rawOffset > threshold && !flipped) {
    if(rawDiff > 0) {
      flipNumber--;
    } else {
      flipNumber++;
    }

    flipped = true;
  } else {
    flipped = false;
  }

  // Set last raw position
  lastRawPos = rawPos;

  // Update absolute position
  absPos = rawPos + flipNumber * maxDistance - initPos;

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
