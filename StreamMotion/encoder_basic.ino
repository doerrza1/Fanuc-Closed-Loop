// Define pins
const int encoderPinA = 2;
const int encoderPinB = 3;

// Variables for encoder state
volatile long encoderValue = 0;
volatile bool lastA = LOW;
volatile bool lastB = LOW;

void setup() {
  Serial.begin(9600);

  pinMode(encoderPinA, INPUT);
  pinMode(encoderPinB, INPUT);

  // Attach interrupt to pin 2 and 3
  attachInterrupt(digitalPinToInterrupt(encoderPinA), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), readEncoder, CHANGE);
}

void loop() {
  // Print the encoder value
  Serial.println(encoderValue);
  //delay(5);
}

void readEncoder() {
  bool currentA = digitalRead(encoderPinA);
  bool currentB = digitalRead(encoderPinB);

  if (currentA != lastA || currentB != lastB) {
    if ((lastA == LOW && currentA == HIGH) || (lastA == HIGH && currentA == LOW)) {
      encoderValue += (lastB == currentB) ? 1 : -1;
    }

  lastA = currentA;
  lastB = currentB;
}
