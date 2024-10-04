#include <Servo.h>  // Include the Servo library
#include <math.h>
float l = 0.59;
float b = 0.41;
float v = 1;
float omega = 0;
float alpha = 0;
float wheelRadius = 0.185;
const int numMotor = 4;
float servoRate = 3.5;
float minOmega = 0.000001;
Servo wheelServos[numMotor];
const int pulseCount = 7;
const int sampleCount = 10;
const int speedPinArray[numMotor] = {22,23,24,25};
const int motorPinArray[numMotor] = {3,4,5,6};
const int reversePinArray[numMotor] = {26,27,28,29};
const int brakePinArray[numMotor] = {30,31,32,33};
const int servoPinArray[numMotor] = {6,7,8,9};
const unsigned long refreshTime = 300000;
unsigned long prevTimeArray[numMotor] = {0,0,0,0};
unsigned long prevPulseWidth[numMotor] = {0,0,0,0};
unsigned long currPulseWidth[numMotor] = {0,0,0,0};
int prevSignalArray[numMotor] = {0,0,0,0};
bool brakeArray[numMotor] = {false,false,false,false};
unsigned long currTime;
int currSignalArray[numMotor];
unsigned long impulseTimeArray[numMotor][pulseCount] = {};
float rpmArray[numMotor][sampleCount];
float rpm;
//
//float targetRpmArray[numMotor] = {100,100,100,100}; // Desired RPM, can be positive or negative
//int targetServoArray[numMotor] = {100,100,100,100};
float targetRpmArray[numMotor] = {}; // Desired RPM, can be positive or negative
int targetServoArray[numMotor] = {};
int prevServoArray[numMotor] = {};
int diffServoArray[numMotor] = {};
const float kp = 1.5; // Proportional gain for PID controller
const float ki = 0.5; // Integral gain for PID controller
const float kd = 0.0; // Derivative gain for PID controller (not used)

float integralArray[numMotor] = {}; // Integral term for PID controller
float derivativeArray[numMotor]={};
float errorArray[numMotor]; // Current error between desired RPM and actual RPM
float previousErrorArray[numMotor] = {}; // Previous error for derivative term
unsigned long previousTimeArray[numMotor] = {}; // Previous time for time difference calculation
float outputArray[numMotor]; // Output value of the PID controller
int pwmArray[numMotor] = {}; // PWM value to control the motor

void calcSpeedAngle(float v, float omega,float alpha){
  float r = fabs(v)/omega;
  if (fabs(r)>=b/2){
  if (fabs(omega)>minOmega){
  float fac;
  for (int motor = 0; motor<numMotor; motor++){
    if (motor==0 || motor ==1){fac = 1;}else{fac=-1;} 
    float wheelVec[2] = {fac*(b/2) + r*cos(alpha), pow(-1,motor)*(l/2) - r*sin(alpha)};
//    float wheelVel = omega/(pow(pow(wheelVec[0],2) + pow(wheelVec[1],2),0.5);
    float wheelVel = (v/fabs(v))*fabs(omega) * sqrt(pow(wheelVec[0], 2) + pow(wheelVec[1], 2));
    if(v<0){targetRpmArray[motor] = -(wheelVel/wheelRadius)*(30/PI);}
    else{targetRpmArray[motor] = (wheelVel/wheelRadius)*(30/PI);}
//    Serial.println(targetRpmArray[0]);
//    if (omega<0){targetServoArray[motor-2] = (180/PI)*atan(wheelVec[1]/wheelVec[0]);}
    {targetServoArray[motor] = (180/PI)*atan(wheelVec[1]/wheelVec[0]);}
    diffServoArray[motor] = fabs(targetServoArray[motor]-prevServoArray[motor]);
    prevServoArray[motor] = targetServoArray[motor];
    wheelServos[motor].write(targetServoArray[motor]+90);}
  }
  else if (fabs(omega)<minOmega){
    float rpm = (v/wheelRadius)*(30/PI);
    for (int motor = 0; motor<numMotor; motor++){
      targetRpmArray[motor] = rpm;
      targetServoArray[motor] = 0;
      diffServoArray[motor] = fabs(targetServoArray[motor]-prevServoArray[motor]);
      prevServoArray[motor] = targetServoArray[motor];
      wheelServos[motor].write(targetServoArray[motor]+90);
    }
  }}
  else if (fabs(r)<(b/2)){
    if (fabs(omega)>minOmega){
  float fac;
  for (int motor = 0; motor<numMotor; motor++){
    if (motor==0 || motor ==1){fac = 1;}else{fac=-1;} 
    float wheelVec[2] = {fac*(b/2) + r*cos(alpha), pow(-1,motor)*(l/2) - r*sin(alpha)};
//    float wheelVel = omega/(pow(pow(wheelVec[0],2) + pow(wheelVec[1],2),0.5);
    float wheelVel = fac*omega* sqrt(pow(wheelVec[0], 2) + pow(wheelVec[1], 2));
    if(v<0){targetRpmArray[motor] = -(wheelVel/wheelRadius)*(30/PI);}
    else{targetRpmArray[motor] = (wheelVel/wheelRadius)*(30/PI);}
//    Serial.println(targetRpmArray[0]);
//    if (omega<0){targetServoArray[motor-2] = (180/PI)*atan(wheelVec[1]/wheelVec[0]);}
    {targetServoArray[motor] = (180/PI)*atan(wheelVec[1]/wheelVec[0]);}
    diffServoArray[motor] = fabs(targetServoArray[motor]-prevServoArray[motor]);
    prevServoArray[motor] = targetServoArray[motor];
    wheelServos[motor].write(targetServoArray[motor]+90);}
  }
  }
}

void calcPulseWidth(int motor, int cases){
  prevSignalArray[motor] = currSignalArray[motor];
  for(int i=0; i<pulseCount-1; i++){
    impulseTimeArray[motor][i] = impulseTimeArray[motor][i+1];}
  impulseTimeArray[motor][pulseCount-1] = currTime;
  if (cases == pulseCount-1){currPulseWidth[motor]=0;}
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

int getPwm(int motor) {
  errorArray[motor] = targetRpmArray[motor] - rpmArray[motor][sampleCount-1]; // Calculate the current error
//  Serial.println(1);
//  if (rpm == 0) {
//    output = 0.1 * error; // Simple proportional control if RPM is zero
//  } else {
    integralArray[motor] += errorArray[motor] * currPulseWidth[motor] / 1000000; // Update integral term
    derivativeArray[motor] = (errorArray[motor] - previousErrorArray[motor]) /( currPulseWidth[motor]/1000000); // Calculate derivative term
    previousErrorArray[motor] = errorArray[motor]; // Update previous error
    outputArray[motor] = kp * errorArray[motor] + ki * integralArray[motor] ; //+ kd * derivativeArray[motor]; // Calculate PID output
    pwmArray[motor] = constrain(outputArray[motor], 0, 255); // Constrain the output to valid PWM range
//    Serial.println(pwmArray[motor]);
    analogWrite(motorPinArray[motor], pwmArray[motor]);
//    Serial.println(pwmArray[motor]);
//  }

//void writePwm(){
//  for (int motor=0; i<numMotor; i++){
//    getPWM(motor);
//  }
}

void calcRpm(){
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
      rpm = 4068814.00 / currPulseWidth[motor];
      Serial.println(rpm);
    }
    else{
      rpm = 0;
    }
    filterRpm(motor,rpm);
  }  
}

void setup() {
  Serial.begin(2000000);

  // Attach each servo to its corresponding pin
  for (int i = 0; i < numMotor; i++) {
    wheelServos[i].attach(servoPinArray[i]);
  }
  
  pinMode(speedPinArray[0], INPUT);
  pinMode(speedPinArray[1], INPUT);
  pinMode(speedPinArray[2], INPUT);
  pinMode(speedPinArray[3], INPUT);

  pinMode(motorPinArray[0], OUTPUT);
  pinMode(motorPinArray[1], OUTPUT);
  pinMode(motorPinArray[2], OUTPUT);
  pinMode(motorPinArray[3], OUTPUT);
  
  pinMode(reversePinArray[0], OUTPUT);
  pinMode(reversePinArray[1], OUTPUT);
  pinMode(reversePinArray[2], OUTPUT);
  pinMode(reversePinArray[3], OUTPUT);
  
  pinMode(brakePinArray[0], OUTPUT);
  pinMode(brakePinArray[1], OUTPUT);
  pinMode(brakePinArray[2], OUTPUT);
  pinMode(brakePinArray[3], OUTPUT);

}

void loop() {
    calcSpeedAngle(v, omega,alpha);
   delay(diffServoArray[3]*servoRate);
   calcRpm();
   
  for (int motor=0; motor<numMotor; motor++){
//    Serial.println(2);
    if (brakeArray[motor] == false) { // If the brake is not engaged
    digitalWrite(brakePinArray[motor], LOW); // Disable the brake
    if (targetRpmArray[motor] > 0) {
      digitalWrite(reversePinArray[motor], HIGH); // Set motor direction to forward
      getPwm(motor); // Update motor speed
//      Serial.println(1);
    } else if (targetRpmArray[motor] < 0) {
      targetRpmArray[motor] = -targetRpmArray[motor];
      digitalWrite(reversePinArray[motor], LOW); // Set motor direction to reverse
      getPwm(motor); // Update motor speed
//      Serial.println(0);
    }
  } else if (brakeArray[motor] == true) { // If the brake is engaged
    digitalWrite(brakePinArray[motor], HIGH); // Enable the brake
  }
  
  }
// Serial.println(rpmArray[0][pulseCount-1]);
  }
