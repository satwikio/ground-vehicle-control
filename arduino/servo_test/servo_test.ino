//#include <Servo.h>  // Include the Servo library
//
//Servo myServo;  // Create a Servo object
//
//void setup() {
//  myServo.attach(8);  // Attach the servo to digital pin 9
//}
//
//void loop() {
//  // Move the servo to 0 degrees
//  myServo.write(0);
//  delay(1000);  // Wait for 1 second
//
//  // Move the servo to 90 degrees
//  myServo.write(90);
//  delay(1000);  // Wait for 1 second
//
//  // Move the servo to 180 degrees
//  myServo.write(200);
//  delay(1000);  // Wait for 1 second
//
//  // Sweep from 0 to 180 degrees
//  for (int pos = 0; pos <= 180; pos++) {
//    myServo.write(pos);
//    delay(15);  // Wait 15 milliseconds for the servo to reach the position
//  }
//
//  // Sweep from 180 to 0 degrees
//  for (int pos = 180; pos >= 0; pos--) {
//    myServo.write(pos);
//    delay(15);  // Wait 15 milliseconds for the servo to reach the position
//  }
//}


// #include <Servo.h>  // Include the Servo library
//
//// Define the number of servos
//#define NUM_SERVOS 4
//
//// Create an array of Servo objects
//Servo myServos[NUM_SERVOS];
//
//// Define an array of pin numbers where the servos are connected
//int servoPins[NUM_SERVOS] = {7,8, 9, 10};
//
//void setup() {
//   
//  // Attach each servo to its corresponding pin
//  for (int i = 0; i < NUM_SERVOS; i++) {
//    myServos[i].attach(servoPins[i]);
//  }
//}
//
//
//void loop() {
//  // Example of moving all servos to 0 degrees
//  for (int i = 0; i < NUM_SERVOS; i++) {
//    myServos[i].write(0);
//  }
//  delay(1000);  // Wait for 1 second
//
//  // Example of moving all servos to 90 degrees
//  for (int i = 0; i < NUM_SERVOS; i++) {
//    myServos[i].write(90);
//  }
//  delay(1000);  // Wait for 1 second
//
//  // Example of moving all servos to 180 degrees
//  for (int i = 0; i < NUM_SERVOS; i++) {
//    myServos[i].write(180);
//  }
//  delay(1000);  // Wait for 1 second
//
//  // Sweep all servos from 0 to 180 degrees
//  for (int pos = 0; pos <= 180; pos++) {
//    for (int i = 0; i < NUM_SERVOS; i++) {
//      myServos[i].write(pos);
//    }
//    delay(15);  // Wait 15 milliseconds for the servo to reach the position
//  }
//
//  // Sweep all servos from 180 to 0 degrees
//  for (int pos = 180; pos >= 0; pos--) {
//    for (int i = 0; i < NUM_SERVOS; i++) {
//      myServos[i].write(pos);
//    }
//    delay(15);  // Wait 15 milliseconds for the servo to reach the position
//  }
//}





 #include <Servo.h>  // Include the Servo library

// Define the number of servos
#define NUM_SERVOS 4

// Create an array of Servo objects
Servo myServos[NUM_SERVOS];

// Define an array of pin numbers where the servos are connected
int servoPins[NUM_SERVOS] = {7,8, 9, 10};
int zeroAngle[NUM_SERVOS] = {97,90,96,90};
//int targetAngle[NUM_SERVOS] = {66,66,66,66};
int targetAngle[NUM_SERVOS] = {50,50,50,50};

void setup() {
   
  // Attach each servo to its corresponding pin
  for (int i = 0; i < NUM_SERVOS; i++) {
    myServos[i].attach(servoPins[i]);
  }
}


void loop() {
  // Example of moving all servos to 0 degrees
  for (int i = 0; i < NUM_SERVOS; i++) {
    myServos[i].write(zeroAngle[i] - targetAngle[i]);
  }
}
