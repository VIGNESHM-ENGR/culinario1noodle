// Pin Definitions
// BLDC Motor
const int bldcEnablePin = 3;
const int bldcDirPin = 4;
const int bldcPWMPin = 5;

// Pump Relays
const int pump1RelayPin = 6;
const int pump2RelayPin = 7;

// DC Motor
const int dcDirPin = 8;
const int dcPWMPin = 9;

// Servo Motor
#include <Servo.h>
Servo myServo;
const int servoPin = 10;

// Electromagnet Relay
const int magnetRelayPin = 11;

// Variables for control
int bldcSpeed = 0;    // PWM value for BLDC motor (0-255)
int dcMotorSpeed = 0; // PWM value for DC motor (0-255)
int servoAngle = 0;   // Servo angle (0-180)
bool magnetState = false; // Electromagnet state

void setup() {
  // Initialize all pins
  pinMode(bldcEnablePin, OUTPUT);
  pinMode(bldcDirPin, OUTPUT);
  pinMode(bldcPWMPin, OUTPUT);
  pinMode(pump1RelayPin, OUTPUT);
  pinMode(pump2RelayPin, OUTPUT);
  pinMode(dcDirPin, OUTPUT);
  pinMode(dcPWMPin, OUTPUT);
  pinMode(magnetRelayPin, OUTPUT);

  // Initialize Servo
  myServo.attach(servoPin);

  // Set initial states
  digitalWrite(bldcEnablePin, LOW); // Disable BLDC motor at start
  digitalWrite(pump1RelayPin, LOW); // Pump off
  digitalWrite(pump2RelayPin, LOW); // Pump off
  digitalWrite(magnetRelayPin, LOW); // Electromagnet off

  // Start serial communication
  Serial.begin(9600);
}

void loop() {
  // Check for serial input
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    handleCommand(command);
  }
}

// Function to handle commands
void handleCommand(String command) {
  command.trim(); // Remove any leading/trailing whitespace

  if (command.startsWith("PA")) {
    int duration = command.substring(2).toInt(); // Extract duration in seconds
    controlPump(1, true);
    delay(duration * 1000); // Wait for the specified duration
    controlPump(1, false);
  } 
  else if (command == "DCF") {
    dcMotorControl(true, 150); // Forward direction, 60% speed
  }
  else if (command == "DCB") {
    dcMotorControl(false, 150); // Backward direction, 60% speed
  }
  else if (command == "DBF") {
    bldcControl(true, 240); // Forward direction, 95% speed
  }
  else if (command == "DBB") {
    bldcControl(false, 240); // Backward direction, 95% speed
  }
  else if (command == "M") {
    magnetState = !magnetState; // Toggle magnet state
    digitalWrite(magnetRelayPin, magnetState ? HIGH : LOW);
  }
  else if (command.startsWith("S")) {
    int angle = command.substring(1).toInt(); // Extract angle for the servo
    angle = constrain(angle, 0, 180); // Ensure angle is within valid range
    myServo.write(angle);
  }
  else {
    Serial.println("Unknown command");
  }
}

// Function to control BLDC motor
void bldcControl(bool direction, int speed) {
  digitalWrite(bldcDirPin, direction); // Set direction (1 for forward, 0 for reverse)
  analogWrite(bldcPWMPin, speed); // Set speed (0-255)
  digitalWrite(bldcEnablePin, HIGH); // Enable BLDC motor
}

// Function to control Pumps
void controlPump(int pumpNumber, bool state) {
  if (pumpNumber == 1) {
    digitalWrite(pump1RelayPin, state ? HIGH : LOW);
  } else if (pumpNumber == 2) {
    digitalWrite(pump2RelayPin, state ? HIGH : LOW);
  }
}

// Function to control DC Motor
void dcMotorControl(bool direction, int speed) {
  digitalWrite(dcDirPin, direction); // Set direction (1 for forward, 0 for reverse)
  analogWrite(dcPWMPin, speed); // Set speed (0-255)
}

// Function to control Electromagnet
void controlMagnet(bool state) {
  digitalWrite(magnetRelayPin, state ? HIGH : LOW);
}
