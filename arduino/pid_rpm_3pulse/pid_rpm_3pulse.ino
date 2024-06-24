const int pin = 2; // Pin to measure
const int motorPin = 3;
unsigned long startTime;
unsigned long startTime1;
unsigned long startTime2;
unsigned long endTime;
unsigned long endTime3;
unsigned long endTime2;
unsigned long highDuration;
unsigned long endTime1;
float rpm;

const float targetRPM = 200.0; // Desired RPM
const float kp = 1.5; // Proportional gain
const float ki = 0.5; // Integral gain
const float kd = 0.0; // Derivative gain

int pwmValue = 0; // PWM value to control the motor (0-255 for analogWrite)

float integral = 0; // Integral term
float previousError = 0; // Previous error for derivative term
unsigned long previousTime = 0; // Previous time for time difference

void setup() {
  Serial.begin(9600); // Initialize serial communication
  pinMode(pin, INPUT); // Set pin as input
  pinMode(motorPin, OUTPUT);
}

void loop() {
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

  highDuration = endTime3 - startTime; // Calculate the HIGH duration
  rpm = 3920000/highDuration;

  float error = targetRPM - rpm;
  integral += error*highDuration/1000000;
  float derivative = 1000000*(error - previousError)/highDuration;

  float output = kp*error + ki*integral + kd*derivative;

  pwmValue = constrain(output, 0, 255);
  analogWrite(motorPin, pwmValue);
  previousError = error;
  //previousTime = currentTime;

  //Serial.print("High Duration: ");
  Serial.println(rpm);
  //Serial.println(pwmValue);
  //Serial.println(" microseconds");

  //delay(1000); // Wait for 1 second before measuring again
}
