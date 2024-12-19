/* Property of RASOI ROBOTICS
   Operation Noodle - Prototype 1
   This code runs the entire sequence based on the provided workflow.
*/

#include <Arduino.h>
#include <Servo.h> // Include the Servo library

// Define pins to control

// Ingredients
#define ISP 12 // INGREDIENT SERVO PWM
#define IMP 10 // INGREDIENT MOTOR PWM
#define IMD 9  // INGREDIENT MOTOR DIRECTION

// Spices
#define SSP 11 // SPICE SERVO PWM
#define SMP 8  // SPICE MOTOR PWM
#define SMD 7  // SPICE MOTOR DIRECTION

// Stirrer BLDC
#define BLP 6  // BLDC PWM
#define BLD 5  // BLDC DIRECTION
#define BLE 4  // BLDC ENABLE

// Pumps
#define PO 27  // PUMP OIL
#define PW 26  // PUMP WATER

// Magnet
#define EMS 2  // ELECTRO MAGNET SPICE
#define EMI 3  // ELECTRO MAGNET INGREDIENT

#define ISS 22 // SLOT INGREDIENTS
#define SSS 23 // SLOT SPICE



// Servo objects for ingredient and spice
Servo ingredientServo;
Servo spiceServo;

void setup() {
  Serial.begin(115200);

  // Initialize all pins as OUTPUTs
  pinMode(IMP, OUTPUT);
  pinMode(IMD, OUTPUT);
  pinMode(SMP, OUTPUT);
  pinMode(SMD, OUTPUT);
  pinMode(BLP, OUTPUT);
  pinMode(BLD, OUTPUT);
  pinMode(BLE, OUTPUT);
  pinMode(PO, OUTPUT);
  pinMode(PW, OUTPUT);
  pinMode(EMS, OUTPUT);
  pinMode(EMI, OUTPUT);
  pinMode(ISS,INPUT_PULLUP); // ingredient tray slot sensor
  pinMode(SSS,INPUT_PULLUP); // spice tray slot sensor

  // Attach servos to their corresponding pins
  ingredientServo.attach(ISP);
  spiceServo.attach(SSP);
  ingredientServo.write(0);
  spiceServo.write(0);
  // Set initial state to LOW
  resetAll();


Serial.println("MACHINE SELF TEST ON -STARTING IN 5 SECS");

delay(5000);
  continuousStirring(3000);
  delay(1000);
   Serial.println("Pouring OIL...");
  digitalWrite(PO, HIGH);
  delay(1200); //60SEC = 500ML SAY OIL TIME
  digitalWrite(PO, LOW);
  delay(1000);
//HEAT OIL
  Serial.println("Dropping veggies");
  Serial.println("lower heat 1100w ");
  moveTray(IMD, IMP, ISS); // Move ingredient tray for 1 second
  delay(1000);
  controlMagnet(EMI, true); // Attach magnet for ingredient
  delay(1000);
  controlServo(ingredientServo, 0, 110); // Control ingredient servo (0 to 90 degrees)
  delay(1000);
  controlServo(ingredientServo, 110, 70);
  controlServo(ingredientServo, 60, 120); // Control ingredient servo (0 to 90 degrees)
  controlServo(ingredientServo, 120, 0); // Move servo back (90 to 0 degrees)
  delay(1000);
  controlMagnet(EMI, false); // Detach magnet after drop
  delay(1000);
  digitalWrite(IMD, HIGH);  // Set direction forward
  analogWrite(IMP, 128); 
  delay(750);
  analogWrite(IMP, 0);
  Serial.println("veggies sequence complete"); 
//DROP SPICES 
 Serial.println("Dropping spices mix");
 controlServo(spiceServo, 0, 20);
  moveTray(SMD, SMP, SSS);// Move spice tray for 1 second
  digitalWrite(SMD, HIGH);  // Sensor offset
  analogWrite(SMP, 128); 
  delay(350);
  analogWrite(SMP, 0);
  delay(1000);
  controlMagnet(EMS, true); // Attach magnet for spice
   controlServo(spiceServo, 20, 0);
  delay(1000);
  controlServo(spiceServo, 0, 90); // Control spice servo (0 to 90 degrees)
  delay(1000);
  controlServo(spiceServo, 90, 0); // Move servo back (90 to 0 degrees)
  delay(1000);
  controlMagnet(EMS, false); // Detach magnet after drop
  delay(1000);
  digitalWrite(SMD, HIGH);  // Set direction forward
  analogWrite(SMP, 128); 
  delay(750);
  analogWrite(SMP, 0);
  Serial.println("spice mix sequence complete");
 
  Serial.println("Pouring water...");
  digitalWrite(PW, HIGH);
  delay(5000); // Pump ON for 60 seconds
  digitalWrite(PW, LOW);
  delay(1000);

  Serial.println("Dropping noodle cake...");
  moveTray(IMD, IMP, ISS); // Move ingredient tray for 1 second
  delay(1000);
  controlMagnet(EMI, true); // Attach magnet for ingredient
  delay(1000);
  controlServo(ingredientServo, 0, 110); // Control ingredient servo (0 to 90 degrees)
  delay(1000);
  controlServo(ingredientServo, 110, 70);
  controlServo(ingredientServo, 70, 120); // Control ingredient servo (0 to 90 degrees)
  controlServo(ingredientServo, 120, 0); // Move servo back (90 to 0 degrees)
  delay(1000);
  controlMagnet(EMI, false); // Detach magnet after drop
  delay(1000);

//DROP SPICES 
  Serial.println("Dropping NOODLES spices...");
 controlServo(spiceServo, 0, 20);
  moveTray(SMD, SMP, SSS);// Move spice tray for 1 second
  digitalWrite(SMD, HIGH);  // sensor offset
  analogWrite(SMP, 128); 
  delay(450);
  analogWrite(SMP, 0);
  delay(1000);
  controlMagnet(EMS, true); // Attach magnet for spice
   controlServo(spiceServo, 20, 0);
  delay(1000);
  controlServo(spiceServo, 0, 90); // Control spice servo (0 to 90 degrees)
  delay(1000);
  controlServo(spiceServo, 90, 3); // Move servo back (90 to 0 degrees)
  delay(1000);
  controlMagnet(EMS, false); // Detach magnet after drop
  delay(1000);
  resetAll(); // Turn off all actuators
  delay(1000);
}

void loop() {
  
 // PUT NOTHING HERE ELSE IT WILL RESTART
}

// General function to move a tray for a set amount of time
void moveTray(int directionPin, int pwmPin, int slotSensorPin) {
  digitalWrite(directionPin, HIGH);  // Set direction forward
  analogWrite(pwmPin, 128);          // Move at 50% speed
  
  // Keep moving the tray until the slot sensor is triggered
  while (digitalRead(slotSensorPin) == HIGH) {
    // Wait until the sensor reads LOW (indicating the tray has reached the slot)
    // The motor will keep moving as long as the slot sensor is not triggered
  }
  
  // Stop the motor when the sensor is triggered (slot detected)
  analogWrite(pwmPin, 0);            // Stop motor
}

// General function to control a servo motor using degrees
void controlServo(Servo &servo, int startDegree, int endDegree) {
  servo.write(startDegree); // Move to start degree
  delay(1000);              // Wait for servo to move
  servo.write(endDegree);   // Move to end degree
  delay(1000);              // Wait for servo to move
}

// General function to control magnet (attach/detach)
void controlMagnet(int magnetPin, bool attach) {
  if (attach) {
    digitalWrite(magnetPin, HIGH);  // Attach magnet
  } else {
    digitalWrite(magnetPin, LOW);   // Detach magnet
  }
}

void resetAll() {
  // Set all actuators and motors to LOW
  digitalWrite(PW, LOW);
  digitalWrite(PO, LOW);
  analogWrite(IMP, 0);
  analogWrite(SMP, 0);
  analogWrite(BLP, 0);
  digitalWrite(BLE, LOW);
  digitalWrite(EMS, LOW);
  digitalWrite(EMI, LOW);
}

void stirAtIntervals(int count) {
  // Stirring at 15-second intervals for the given count
  for (int i = 0; i < count; i++) {
    startStirring(6);      // Stir for 15 seconds
    delay(15000);          // Wait for 15 seconds before next stir
  }
}

void continuousStirring(int duration) {
  // Continuous stirring for a given duration in seconds
  digitalWrite(BLE, HIGH);   // Enable BLDC
  digitalWrite(BLD, HIGH);   // Set direction
  analogWrite(BLP, 128);     // 50% speed
  delay(duration);    // Stir for 'duration' seconds
  analogWrite(BLP, 0);       // Stop stirring
  digitalWrite(BLE, LOW);    // Disable BLDC
}

void startStirring(int revolutions) {
  // Function to perform a number of stirring revolutions
  digitalWrite(BLE, HIGH);   // Enable BLDC
  digitalWrite(BLD, HIGH);   // Set direction
  analogWrite(BLP, 128);     // 50% speed
  delay(revolutions * 2000); // Stirring for 'revolutions' number of cycles (2 sec per rev)
  analogWrite(BLP, 0);       // Stop stirring
  digitalWrite(BLE, LOW);    // Disable BLDC
}
