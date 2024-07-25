const int pulseCount = 7;
const int numMotor = 4;
const int sampleCount = 40;
const int speedPinArray[numMotor] = {22,23,24,25};
const int motorPinArray[numMotor] = {2,3,4,5};
const int reversePinArray[numMotor] = {26,27,28,29};
const int brakePinArray[numMotor] = {30,31,32,33};
const unsigned long refreshTime = 300000;
unsigned long prevTimeArray[numMotor] = {0,0,0,0};
unsigned long prevPulseWidth[numMotor] = {0,0,0,0};
unsigned long currPulseWidth[numMotor] = {0,0,0,0};
int prevSignalArray[numMotor] = {0,0,0,0};
//bool brakeArray[numMotor] = {false,false,false,false};
unsigned long currTime;
int currSignalArray[numMotor];
unsigned long impulseTimeArray[numMotor][pulseCount] = {};
float rpmArray[numMotor][sampleCount];
float rpm;

//float targetRpmArray[numMotor] = {200.0, 200.00, 200.00, 200.00}; // Desired RPM, can be positive or negative
//const float absTargetRpmArray[numMotor]; for(int i; i<numMotor; i++){fabs(targetRpmArray[i]);} // Absolute value of the desired RPM
//bool brakeArray[numMotor] = {false,false,false,false}; // Variable to control the brake state

//const float kp = 1.5; // Proportional gain for PID controller
//const float ki = 0.5; // Integral gain for PID controller
//const float kd = 0.0; // Derivative gain for PID controller (not used)

//float integralArray[numMotor] = {}; // Integral term for PID controller
//float errorArray[numMotor]; // Current error between desired RPM and actual RPM
//float previousErrorArray[numMotor] = {}; // Previous error for derivative term
unsigned long previousTimeArray[numMotor] = {}; // Previous time for time difference calculation
//float outputArray[numMotor]; // Output value of the PID controller
//int pwmArray[numMotor] = {}; // PWM value to control the motor (0-255 for analogWrite)

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
      Serial.println(0);
      prevPulseWidth[motor] = currPulseWidth[motor];
      for (int i=0; i<numMotor; i++ ){
        impulseTimeArray[motor][i] = 0;
      }
      }
      }
  }

  for(int motor=0; motor<1; motor++){
    if (currPulseWidth[motor] != 0){
      rpm = 4068814.00 / currPulseWidth[motor];
      Serial.println(rpm);
    }
    else{
      rpm = 0;
    }
    filterRpm(motor,rpm);
  }
  Serial.println(rpmArray[0][pulseCount-1]);
}
