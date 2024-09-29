//#include <IBusBM.h>
//IBusBM IBus;
//
//int ch1 = 0;
//int ch2 = 0;
//int ch3 = 0;
//int ch4 = 0;
//int ch5 = 0;
//int ch6 = 0;
//void setup() {
//  // initialize digital pin LED_BUILTIN as an output.
//  Serial.begin(115200);
//  IBus.begin(Serial);
//  
//}
//
//// the loop function runs over and over again forever
//void loop() {
//  ch1 = IBus.readChannel(0);
//  ch2 = IBus.readChannel(1);
//  ch3 = IBus.readChannel(2);
//  ch4 = IBus.readChannel(3);
//  ch5 = IBus.readChannel(4);
//  ch6 = IBus.readChannel(5);
//
//  Serial.print(ch1);
//  Serial.print(",");
//  Serial.print(ch2);
//  Serial.print("'");
//  Serial.print(ch3);
//  Serial.print(",");
//  Serial.print(ch4);
//  Serial.print("'");
//  Serial.print(ch5);
//  Serial.print(",");
//  Serial.print(ch6);
//  Serial.print("'");
//  }



#include "PWM.hpp"

PWM ch1(2); // Setup pin 2 for input
PWM ch2(3); // Setup pin 3 for input
PWM ch3(18); // Setup pin 18 for input
PWM ch4(19); // Setup pin 19 for input
PWM ch5(20); // Setup pin 20 for input
PWM ch6(21); // Setup pin 21 for input

void setup() {
    Serial.begin(115200); // Serial for debug
    ch1.begin(true); // ch1 on pin 2 reading PWM HIGH duration
    ch2.begin(true); // ch2 on pin 3 reading PWM HIGH duration
    ch3.begin(true); // ch3 on pin 18 reading PWM HIGH duration
    ch4.begin(true); // ch4 on pin 19 reading PWM HIGH duration
    ch5.begin(true); // ch5 on pin 20 reading PWM HIGH duration
    ch6.begin(true); // ch6 on pin 21 reading PWM HIGH duration
}

void loop() {
    Serial.print(ch1.getValue());
    Serial.print("\t");
    Serial.print(ch2.getValue());
    Serial.print("\t");
    Serial.print(ch3.getValue());
    Serial.print("\t");
    Serial.print(ch4.getValue());
    Serial.print("\t");
    Serial.print(ch5.getValue());
    Serial.print("\t");
    Serial.print(ch6.getValue());
    Serial.print("\t");
    Serial.println();
    delay(100);
}
