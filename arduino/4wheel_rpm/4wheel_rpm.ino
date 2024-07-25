const int pulseCount = 6;
const int numMotor = 4;
const int speedPinArray[numMotor] = {22,23,24,25};
const unsigned long refreshTime = 300000000;
unsigned long prevTimeArray[numMotor] = {0,0,0,0};
unsigned long prevPulseWidth[numMotor] = {0,0,0,0};
unsigned long currPulseWidth[numMotor] = {0,0,0,0};
int prevSignalArray[numMotor] = {0,0,0,0};
unsigned long currTime;
int currSignalArray[numMotor];
unsigned long impulseTimeArray[numMotor][pulseCount] = {
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0},
  {0,0,0,0,0,0}};
float rpmArray[numMotor];

void calcPulseWidth(int motor, int cases){
  prevSignalArray[motor] = currSignalArray[motor];
  for(int i=0; i<pulseCount-1; i++){
    impulseTimeArray[motor][i] = impulseTimeArray[motor][i+1];}
  impulseTimeArray[motor][pulseCount-1] = currTime;
  if (cases == 5){currPulseWidth[motor]=0;Serial.println(5);}
  if (cases != 5){currPulseWidth[motor] = ((pulseCount-1)/(pulseCount-cases-1))*(impulseTimeArray[motor][pulseCount-1] - impulseTimeArray[motor][cases]);}
//  Serial.println(currPulseWidth[3]);
  prevPulseWidth[motor] = currPulseWidth[motor];
  prevTimeArray[motor] = currTime;}

void setup() {
  Serial.begin(2000000);
  pinMode(speedPinArray[0], INPUT);
  pinMode(speedPinArray[1], INPUT);
  pinMode(speedPinArray[2], INPUT);
  pinMode(speedPinArray[3], INPUT);

}

void loop() {
  for (int i=0; i<numMotor; i++){
    currSignalArray[i] = digitalRead(speedPinArray[i]);}
  currTime = micros();

  for(int motor=0; motor<numMotor; motor++){
    if(currSignalArray[motor] != prevSignalArray[motor]){
      if (impulseTimeArray[motor][0] != 0 && impulseTimeArray[motor][1] != 0 && impulseTimeArray[motor][2] != 0 && impulseTimeArray[motor][3] != 0 && impulseTimeArray[motor][4] != 0){
        calcPulseWidth(motor,0);
//        Serial.println(0);
}
        
      if (impulseTimeArray[motor][0] == 0 && impulseTimeArray[motor][1] != 0 && impulseTimeArray[motor][2] != 0 && impulseTimeArray[motor][3] != 0 && impulseTimeArray[motor][4] != 0){
        calcPulseWidth(motor,1);
//        Serial.println(1);
}
        
      if (impulseTimeArray[motor][0] == 0 && impulseTimeArray[motor][1] == 0 && impulseTimeArray[motor][2] != 0 && impulseTimeArray[motor][3] != 0 && impulseTimeArray[motor][4] != 0){
        calcPulseWidth(motor,2);
//        Serial.println(2);
}
        
      if (impulseTimeArray[motor][0] == 0 && impulseTimeArray[motor][1] == 0 && impulseTimeArray[motor][2] == 0 && impulseTimeArray[motor][3] != 0 && impulseTimeArray[motor][4] != 0){
        calcPulseWidth(motor,3);
//        Serial.println(3);
}
        
      if (impulseTimeArray[motor][0] == 0 && impulseTimeArray[motor][1] == 0 && impulseTimeArray[motor][2] == 0 && impulseTimeArray[motor][3] == 0 && impulseTimeArray[motor][4] != 0){
        calcPulseWidth(motor,4);
//        Serial.println(4);
}
        
      if (impulseTimeArray[motor][0] == 0 && impulseTimeArray[motor][1] == 0 && impulseTimeArray[motor][2] == 0 && impulseTimeArray[motor][3] == 0 && impulseTimeArray[motor][4] == 0){
        calcPulseWidth(motor,5);
//        Serial.println(5);
}
          
        }
    else{
    if(currTime - prevTimeArray[motor] < refreshTime){
      currPulseWidth[motor] = prevPulseWidth[motor];}
    if (currTime - prevTimeArray[motor] > refreshTime){
      currPulseWidth[motor] = 0;
      Serial.println(0);
      prevPulseWidth[motor] = currPulseWidth[motor];
//      impulseTimeArray = {
//  {0,0,0,0,0,0},
//  {0,0,0,0,0,0},
//  {0,0,0,0,0,0},
//  {0,0,0,0,0,0}};
      }
      }
  }

  for(int motor=0; motor<numMotor; motor++){
    if (currPulseWidth[motor] != 0){
      rpmArray[motor] = 3487555.00 / currPulseWidth[motor];
    }
    else{
      rpmArray[motor] = 0;
    }
  }
 Serial.println(rpmArray[0]);
  }
