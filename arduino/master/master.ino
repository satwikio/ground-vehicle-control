#include <Wire.h>

void setup() {
  Wire.begin(); // Join I2C bus as master
  Serial.begin(9600);
}

void loop() {
  // Communicate with slave at address 8
  Wire.beginTransmission(8);
  Wire.write("Hello, Slave 8!");
  Wire.endTransmission();
  
  delay(10);

//  // Communicate with slave at address 9
//  Wire.beginTransmission(9);
//  Wire.write("Hello, Slave 9!");
//  Wire.endTransmission();
//
//  delay(500);
}
