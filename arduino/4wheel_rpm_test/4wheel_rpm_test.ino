const int pulseCount = 7;
const int numMotor = 4;
const int sampleCount = 30;
const int speedPinArray[numMotor] = {22,23,24,25};
const unsigned long refreshTime = 3000000;
unsigned long prevTimeArray[numMotor] = {0,0,0,0};
unsigned long prevPulseWidth[numMotor] = {0,0,0,0};
unsigned long currPulseWidth[numMotor] = {0,0,0,0};
int prevSignalArray[numMotor] = {0,0,0,0};
unsigned long currTime;
int currSignalArray[numMotor];
unsigned long impulseTimeArray[numMotor][pulseCount] = {};
float rpmArray[numMotor][sampleCount];
float rpm;

void calcPulseWidth(int motor, int cases){
  prevSignalArray[motor] = currSignalArray[motor];
  for(int i=0; i<pulseCount-1; i++){
    impulseTimeArray[motor][i] = impulseTimeArray[motor][i+1];}
  impulseTimeArray[motor][pulseCount-1] = currTime;
  if (cases == pulseCount-1){currPulseWidth[motor]=0;Serial.println(5);}
  if (cases != pulseCount-1){currPulseWidth[motor] = ((pulseCount-1)/(pulseCount-cases-1))*(impulseTimeArray[motor][pulseCount-1] - impulseTimeArray[motor][cases]);}
//  Serial.println(currPulseWidth[3]);
  prevPulseWidth[motor] = currPulseWidth[motor];
  prevTimeArray[motor] = currTime;}

void filterRpm(int motor, float rpm){
  if (rpmArray[motor][0] != 0){
//    rpm = 0.2*rpm + 0.15*rpmArray[motor][5] + 0.15*rpmArray[motor][4] + 0.15*rpmArray[motor][3] + 0.15*rpmArray[motor][2] + 0.1*rpmArray[motor][1] + 0.1*rpmArray[motor][5];
      rpm = rpmArray[motor][sampleCount-1] + ((rpm-rpmArray[motor][0])/sampleCount);
  }
  for(int i=0; i<sampleCount-1; i++){
    rpmArray[motor][i] = rpmArray[motor][i+1];}
  rpmArray[motor][sampleCount-1] = rpm;
}

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
        
      if (impulseTimeArray[motor][0] == 0 && impulseTimeArray[motor][1] != 0 && impulseTimeArray[motor][2] != 0 && impulseTimeArray[motor][3] != 0 && impulseTimeArray[motor][4] != 0 && impulseTimeArray[motor][4] != 0){
        calcPulseWidth(motor,1);
//        Serial.println(1);
}
        
      if (impulseTimeArray[motor][0] == 0 && impulseTimeArray[motor][1] == 0 && impulseTimeArray[motor][2] != 0 && impulseTimeArray[motor][3] != 0 && impulseTimeArray[motor][4] != 0 && impulseTimeArray[motor][4] != 0){
        calcPulseWidth(motor,2);
//        Serial.println(2);
}
        
      if (impulseTimeArray[motor][0] == 0 && impulseTimeArray[motor][1] == 0 && impulseTimeArray[motor][2] == 0 && impulseTimeArray[motor][3] != 0 && impulseTimeArray[motor][4] != 0 && impulseTimeArray[motor][4] != 0){
        calcPulseWidth(motor,3);
//        Serial.println(3);
}
        
      if (impulseTimeArray[motor][0] == 0 && impulseTimeArray[motor][1] == 0 && impulseTimeArray[motor][2] == 0 && impulseTimeArray[motor][3] == 0 && impulseTimeArray[motor][4] != 0 && impulseTimeArray[motor][4] != 0){
        calcPulseWidth(motor,4);
//        Serial.println(4);
}
        
      if (impulseTimeArray[motor][0] == 0 && impulseTimeArray[motor][1] == 0 && impulseTimeArray[motor][2] == 0 && impulseTimeArray[motor][3] == 0 && impulseTimeArray[motor][4] == 0 && impulseTimeArray[motor][4] != 0){
        calcPulseWidth(motor,5);
//        Serial.println(5);
      }
      if (impulseTimeArray[motor][0] == 0 && impulseTimeArray[motor][1] == 0 && impulseTimeArray[motor][2] == 0 && impulseTimeArray[motor][3] == 0 && impulseTimeArray[motor][4] == 0 && impulseTimeArray[motor][4] == 0){
        calcPulseWidth(motor,6);
}
          
        }
    else{
    if(currTime - prevTimeArray[motor] < refreshTime){
      currPulseWidth[motor] = prevPulseWidth[motor];}
    if (currTime - prevTimeArray[motor] > refreshTime){
      currPulseWidth[motor] = 0;
//      Serial.println(0);
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
      rpm = 1395022.00 / currPulseWidth[motor];
    }
    else{
      rpm = 0;
    }
    filterRpm(motor,rpm);
  }
 Serial.println(rpmArray[0][0]);
  }
