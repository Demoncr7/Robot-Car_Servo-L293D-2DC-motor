//Jose De Jesus Alarcon
// Robot car with Servo, L293D, 2 DC Motors, Ultrasonic Sensor
#include <Servo.h>

// Define the motor control pins for Motor A
const int ENA = 2;  // Enable Motor A (Connect to EN1 on L293D)
const int IN1 = 6;  // Input 1 (Connect to IN1 on L293D)
const int IN2 = 7;  // Input 2 (Connect to IN2 on L293D)

// Define the motor control pins for Motor B
const int ENB = 3;  // Enable Motor B (Connect to EN2 on L293D)
const int IN3 = 8;  // Input 3 (Connect to IN3 on L293D)
const int IN4 = 9;  // Input 4 (Connect to IN4 on L293D)

// Define the ultrasonic sensor pins
const int trigPin = 5;  // Ultrasonic sensor trigger pin
const int echoPin = 4;  // Ultrasonic sensor echo pin

// Define the servo motor
Servo servoMotor;

void setup() {
  // Set motor control pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initialize motor control pins
  digitalWrite(ENA, HIGH);  // Enable Motor A
  digitalWrite(ENB, HIGH);  // Enable Motor B

  // Initialize the servo motor
  servoMotor.attach(9);  // Connect servo control wire to D9

  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Measure distance using ultrasonic sensor
  long duration, cm;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  cm = duration / 58;  // Calculate distance in centimeters

  // Check if an obstacle is within 20cm
  if (cm < 20) {
    // Stop the robot
    stopRobot();
    delay(500);  // Pause for half a second
    
    // Reverse at a slower speed
    reverseRobot();
    delay(500);  // Pause for half a second
    
    // Look left and right to find a clearing
    servoMotor.write(0);  // Look left
    delay(1000);  // Wait for the servo to move
    servoMotor.write(180);  // Look right
    delay(1000);  // Wait for the servo to move
    servoMotor.write(90);  // Look forward

    // Continue moving forward
    moveForward();
  } else {
    // If no obstacle, continue moving forward
    moveForward();
  }
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void stopRobot() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void reverseRobot() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}
