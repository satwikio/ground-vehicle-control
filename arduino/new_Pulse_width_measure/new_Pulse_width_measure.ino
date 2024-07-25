const int speedPin = 22;
const unsigned long refreshTime = 50000;
unsigned long prevTime = 0;
unsigned long impulse1 = 0;
unsigned long impulse2 = 0;
unsigned long impulse3 = 0;
unsigned long impulse4 = 0;
int prevSignal = 0;
unsigned long prevPulseWidth = 0;
unsigned long currPulseWidth = 0;
float rpm;


void setup() {
  Serial.begin(250000);
  pinMode(speedPin, INPUT);
}

void loop() {
  int currSignal = digitalRead(speedPin);
  unsigned long currTime = micros();
  if (currSignal != prevSignal){
    if (impulse1 != 0 && impulse2 != 0 && impulse3 != 0){
    prevSignal = currSignal;
    impulse1 = impulse2;
    impulse2 = impulse3;
    impulse3 = impulse4;
    impulse4 = currTime;
    currPulseWidth = impulse4 - impulse1;
    prevPulseWidth = currPulseWidth;
    prevTime = currTime;}

   if (impulse1 == 0 && impulse2 != 0 && impulse3 != 0){
    prevSignal = currSignal;
    impulse1 = impulse2;
    impulse2 = impulse3;
    impulse3 = impulse4;
    impulse4 = currTime;
    currPulseWidth = 1.5*(impulse4 - impulse2);
    prevPulseWidth = currPulseWidth;
    prevTime = currTime;}

    if (impulse1 == 0 && impulse2 == 0 && impulse3 != 0){
    prevSignal = currSignal;
    impulse1 = impulse2;
    impulse2 = impulse3;
    impulse3 = impulse4;
    impulse4 = currTime;
    currPulseWidth = 3*(impulse4 - impulse2);
    prevPulseWidth = currPulseWidth;}
  
    if (impulse1 == 0 && impulse2 == 0 && impulse3 == 0){
    prevSignal = currSignal;
    impulse1 = impulse2;
    impulse2 = impulse3;
    impulse3 = impulse4;
    impulse4 = currTime;
    currPulseWidth = 0;
    prevPulseWidth = currPulseWidth;}
  }
  else{
    if(currTime - prevTime < refreshTime){
      currPulseWidth = prevPulseWidth;}
    if (currTime - prevTime > refreshTime){
      currPulseWidth = 0;
      prevPulseWidth = currPulseWidth;
      impulse1 = 0;
      impulse2 = 0;
      impulse3 = 0;
      impulse4 = 0;
      }
    }
  if (currPulseWidth !=0){
    rpm = 697511.00 / currPulseWidth;}
  else{
    rpm = 0;}
  Serial.println(rpm);
  //delayMicroseconds(400);
}
