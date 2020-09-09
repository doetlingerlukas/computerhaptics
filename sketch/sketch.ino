int pwmPin = 3;
int dirPin = 12;

int incData = 0;

int sensorPin = A2;

void setup() {
  Serial.begin(9600);
  
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);

  pinMode(sensorPin, INPUT);
}

void loop() {
  if (Serial.available() > 0) {
    // read the incoming byte:
    incData = Serial.read();

    // say what you got:
    Serial.print("I received: ");
    Serial.println(incData, DEC);
  }
  
  int sensorValue = analogRead(sensorPin);

  // digitalWrite(dirPin, HIGH);
  //analogWrite(pwmPin, 50);

  delay(3000);

  // digitalWrite(dirPin, LOW);

  // delay(3000);

  //analogWrite(pwmPin, 0);

  delay(7000);

  Serial.println(sensorValue);
  
}
