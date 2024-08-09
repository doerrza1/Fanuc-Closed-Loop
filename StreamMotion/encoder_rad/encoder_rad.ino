#include <Encoder.h>

// Define pins
const int encoderPinA = 2;
const int encoderPinB = 3;

// Create an Encoder object
Encoder myEncoder(encoderPinA, encoderPinB);

// Counts per revolution for your encoder
const float CPR = 8000.0; // Replace with your encoder's CPR

// Conversion factor from counts to radians
const float countsToRadians = 2 * PI / CPR;

// Variables for velocity and acceleration calculation
long lastPositionCounts = 0;
unsigned long lastTime = 0;
float velocity = 0;
float acceleration = 0;

void setup() {
  Serial.begin(115200);
  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  long currentPositionCounts = myEncoder.read();
  unsigned long timeInterval = currentTime - lastTime;

  // Convert position from counts to radians
  float currentPosition = currentPositionCounts * countsToRadians;

  // Calculate velocity (radians per second)
  if (timeInterval > 0) {
    velocity = ((currentPositionCounts - lastPositionCounts) * countsToRadians) / (timeInterval / 1000.0);
  }

  // Calculate acceleration (radians per second squared)
  static float lastVelocity = 0;
  acceleration = (velocity - lastVelocity) / (timeInterval / 1000.0);
  lastVelocity = velocity;

  // Print the position, velocity, and acceleration
  Serial.print(currentPosition);
  Serial.print(",");
  Serial.print(velocity, 6);
  Serial.print(",");
  Serial.println(acceleration, 6);

  // Update last position and time
  lastPositionCounts = currentPositionCounts;
  lastTime = currentTime;

}
