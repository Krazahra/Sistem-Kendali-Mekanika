#include <PID_v1.h>

// Define pins for the motor control
const int LeftMotorForward = 5;
const int LeftMotorBackward = 4;
const int RightMotorForward = 3;
const int RightMotorBackward = 2;
const int ENA = 10; // Enable pin for motor A
const int ENB = 11; // Enable pin for motor B

// Define pins for the ultrasonic sensor
const int trigPin = A3;
const int echoPin = A2;

// Define variable to store the duration and distance
long duration;
int distance;

// Define PID variables
double Setpoint, Input, Output;
double Kp = 14, Ki = 0.02, Kd = 20; 

// Initialize the PID controller
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  // Initialize the serial communication
  Serial.begin(9600);
  
  // Set the trigger pin as output
  pinMode(trigPin, OUTPUT);
  
  // Set the echo pin as input
  pinMode(echoPin, INPUT);

  // Set motor control pins as outputs
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorForward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);

  // Set enable pins as outputs
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Initialize the PID controller
  Setpoint = 15; // Desired distance from the obstacle
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255); // Output range to control motor speed
}

void moveForward(int speed) {
  // Move the robot forward
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(RightMotorBackward, LOW);
}

void moveBackward(int speed) {
  // Move the robot backward
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBackward, HIGH);
}

void stopMoving() {
  // Stop the robot
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}

void loop() {
  // Clear the trigPin by setting it LOW
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Set the trigPin HIGH for 10 microseconds to send the ultrasound wave
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.034 / 2;

  // Set the PID input to the measured distance
  Input = distance;

  // Compute the PID output
  myPID.Compute();

  // Print the distance and PID output on the serial monitor and for Serial Plotter
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print(" cm, PID Output: ");
  Serial.println(Output);

  // Send data to Serial Plotter
  Serial.println(distance);
  
  // Control 
  if (distance <= 4) { 
    moveBackward(Output);
  } else if (distance <= 9) { 
    stopMoving();
  } else if (distance > 10) { 
    moveForward(Output-50);
  } 

  // delay
  delay(50);
}
