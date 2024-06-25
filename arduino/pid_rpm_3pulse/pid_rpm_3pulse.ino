const int pin = 2; // Pin to measure
const int motorPin = 3;
unsigned long startTime;
unsigned long startTime1;
unsigned long startTime2;
unsigned long endTime;
unsigned long endTime3;
unsigned long endTime2;
unsigned long highDuration ;
unsigned long endTime1;
float rpm = 0;


const float targetRPM = 200.0; // Desired RPM
const float kp = 1.5; // Proportional gain
const float ki = 0.5; // Integral gain
const float kd = 0.0; // Derivative gain

int pwmValue = 0; // PWM value to control the motor (0-255 for analogWrite)

float integral = 0; // Integral term
float error;
float previousError = 0; // Previous error for derivative term
unsigned long previousTime = 0; // Previous time for time difference
float output;

unsigned long getPulseWidth(){
    // Wait for the pin to go HIGH
  while (digitalRead(pin) == LOW);
  startTime = micros(); // Get the start time

  // Wait for the pin to go LOW
  while (digitalRead(pin) == HIGH);
  endTime = micros(); // Get the end time

  while (digitalRead(pin) == LOW);
  endTime1 = micros(); // Get the end time

  // Wait for the pin to go LOW
  while (digitalRead(pin) == HIGH);
  endTime1 = micros(); // Get the end time

  while (digitalRead(pin) == LOW);
  endTime2 = micros(); // Get the end time

  // Wait for the pin to go LOW
  while (digitalRead(pin) == HIGH);
  endTime2 = micros(); // Get the end time

  while (digitalRead(pin) == LOW);
  endTime3 = micros(); // Get the end time

  unsigned long highDuration1 = endTime3 - startTime; // Calculate the HIGH duration
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
  //Serial.println(pwmValue);
}
void setup() {
  Serial.begin(9600); // Initialize serial communication
  pinMode(pin, INPUT); // Set pin as input
  pinMode(motorPin, OUTPUT);
}

void loop() {
  getPWM(rpm);
  analogWrite (motorPin, pwmValue);
  highDuration = getPulseWidth(); // Calculate the HIGH duration
  Serial.println(highDuration);
  rpm = 3920000/highDuration;

  // getPWM(rpm);
  // analogWrite (motorPin, pwmValue);


  //Serial.print("High Duration: ");
  //Serial.println(pwmValue);
  //Serial.println(pwmValue);
  //Serial.println(" microseconds");
  // highDuration = getPulseWidth(); // Calculate the HIGH duration

}
