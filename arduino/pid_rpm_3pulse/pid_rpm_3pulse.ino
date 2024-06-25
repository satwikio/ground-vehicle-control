const int speedPin = 2; // Pin to measure
const int motorPin = 3;
const int reversePin = 4;
const int brakePin = 5;
unsigned long startTime;
unsigned long endTime;
unsigned long highDuration ;
float rpm = 0;

const float targetRPM1 = 200.0; // Desired RPM
const float targetRPM = fabs(targetRPM1);
bool brake = false;

const float kp = 1.5; // Proportional gain
const float ki = 0.5; // Integral gain
const float kd = 0.0; // Derivative gain
float integral = 0; // Integral term
float error;
float previousError = 0; // Previous error for derivative term
unsigned long previousTime = 0; // Previous time for time difference
float output;
int pwmValue = 0; // PWM value to control the motor (0-255 for analogWrite)

unsigned long getPulseWidth(){
    // Wait for the pin to go HIGH
  while (digitalRead(speedPin) == LOW);
  startTime = micros(); // Get the start time

  while (digitalRead(speedPin) == HIGH); // Wait for the pin to go LOW
  endTime = micros(); // Get the end time

  while (digitalRead(speedPin) == LOW);
  endTime = micros(); // Get the end time

  while (digitalRead(speedPin) == HIGH);
  endTime = micros(); // Get the end time

  while (digitalRead(speedPin) == LOW);
  endTime = micros(); // Get the end time

  while (digitalRead(speedPin) == HIGH);
  endTime = micros(); // Get the end time

  while (digitalRead(speedPin) == LOW);
  endTime = micros(); // Get the end time

  unsigned long highDuration1 = endTime - startTime; // Calculate the HIGH duration
  return highDuration1;
}

int getPWM(float rpm){
  error = targetRPM - rpm;
  if (rpm == 0){
    output = 0.1*error;
  }
  else{
  integral += error*highDuration/1000000;
  float derivative = 1000000*(error - previousError)/highDuration;
  previousError = error;
  output = kp*error + ki*integral + kd*derivative;
  }
  pwmValue = constrain(output, 0, 255);
}

void writePWM(){
  getPWM(rpm);
  analogWrite (motorPin, pwmValue);
  highDuration = getPulseWidth(); // Calculate the HIGH duration
  rpm = 3920000/highDuration;
  //Serial.println(rpm);
}

void setup() {
  Serial.begin(9600); // Initialize serial communication
  pinMode(speedPin, INPUT); // Set pin as input
  pinMode(motorPin, OUTPUT);
  pinMode(reversePin, OUTPUT);
  pinMode(brakePin, OUTPUT);
}

void loop() {
  if (brake == false){
    digitalWrite(brakePin, LOW);
    if (targetRPM1 > 0){
      digitalWrite(reversePin, HIGH);
      writePWM();
    }
    else if (targetRPM1 <0 ){
      digitalWrite(reversePin, LOW);
      writePWM();
    }
  }
  else if (brake == true){
    digitalWrite(brakePin, HIGH);
  }

}
